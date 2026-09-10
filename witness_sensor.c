#define _POSIX_C_SOURCE 200809L

/*
 * ==================================================
 * 관찰차량용 BNO086 heading 센서 프로세스
 * ==================================================
 *
 * 관찰차량은 고정된 테스트베드 좌표와 BNO086의 상대 heading만
 * witness_filter에 전송한다. PAA5100JE, 위치 적분, 사고 판정은 이
 * 프로세스의 역할이 아니다.
 *
 * Thread 역할:
 *
 *     BNO086 I/O Thread
 *          - BNO086을 유일하게 소유
 *          - startup, feature 설정, SH-2 packet drain
 *
 *     Main / IPC Thread
 *          - 최신 BNO snapshot으로 상대 heading 계산
 *          - 고정 X/Y와 heading을 witness_filter에 20 Hz로 전송
 *
 * 사고차량 프로세스와 역할을 섞지 않는다. 관찰차량에는 PAA5100JE가 없고
 * 위치 적분이나 충돌 검출도 하지 않는다. 배치 때 확정한 고정 좌표에 현재
 * 관측 방향만 결합한 vehicle_state_t를 witness_filter 전용 socket으로 보낸다.
 * BNO I/O와 IPC를 두 흐름으로 나누어 filter 연결 지연이 SHTP packet drain을
 * 막거나 IMU report를 stale하게 만들지 않게 한다.
 *
 * 예를 들어 관찰차량을 testbed (1200, 300) mm에 고정하고 처음 +Y 방향을
 * 보게 했다면 X/Y는 실행 내내 그대로이고, 사용자가 차량을 오른쪽으로 돌렸을
 * 때 heading만 변한다. 이 프로세스가 보내는 것은 "카메라가 현재 어느 위치에서
 * 어느 방향을 보고 있는가"이며 사고차량의 위치를 계산하는 값이 아니다.
 */

#include "bno086.h"
#include "position_fusion.h"
#include "ipc.h"
#include "protocol.h"
#include "sensor_protocol.h"

#include <errno.h>
#include <float.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

/*
 * TODO(testbed): 차량을 배치한 뒤 관찰차량별 고정 X/Y와 시작 정면
 * heading을 아래 값에 확정한다. witness_filter의 좌표계는 0°=+Y,
 * 90°=+X이다. 실차 장착 후 좌/우 ±90° 회전으로 BNO 축 배치와 부호를
 * 확인하고 WITNESS_RELATIVE_HEADING_SIGN을 +1 또는 -1로 확정해야 한다.
 * 고정 X/Y는 센서가 측정하는 값이 아니며 각 관찰차량 binary를 배포하기 전에
 * 실제 testbed 좌표를 직접 넣는다.
 */
#define WITNESS_FIXED_X_MM                    0.0
#define WITNESS_FIXED_Y_MM                    0.0
#define WITNESS_INITIAL_HEADING_DEG           0.0
#define WITNESS_RELATIVE_HEADING_SIGN          1.0

#define WITNESS_FILTER_SOCKET_PATH         "/tmp/witness_sensor.sock"
#define WITNESS_RV_INTERVAL_US             10000U
#define WITNESS_MAG_INTERVAL_US            20000U
#define WITNESS_BNO_UPDATE_INTERVAL_US      2000U
#define WITNESS_PUBLISH_INTERVAL_NS     50000000ULL
#define WITNESS_RECONNECT_INTERVAL_NS  500000000ULL
#define WITNESS_SEND_TIMEOUT_MS              200L
#define WITNESS_RV_STALE_NS            200000000ULL
#define WITNESS_MAG_STALE_NS           500000000ULL
#define WITNESS_MIN_ACCURACY                     2U

/*
 * RV는 100 Hz로 방향 변화를 받고 Mag는 calibration 절차에 맞춰 50 Hz로 받는다.
 * filter에는 20 Hz로 최신 상태를 보내므로 센서 packet을 모두 IPC로 복제하지는
 * 않는다. stale 제한은 publish 주기와 별개로 실제 센서 흐름이 끊겼는지 본다.
 */

typedef struct
{
    /* BNO를 유일하게 소유하고 startup 결과와 실제 적용 report interval을 공개한다. */
    bno086_t imu;
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t start_condition;

    bool thread_created;
    bool start_finished;
    bool ready;
    bool stop_requested;

    uint32_t rotation_interval_us;
    uint32_t mag_interval_us;
    char start_error[BNO086_ERROR_TEXT_SIZE];
} witness_imu_worker_t;

typedef struct
{
    /* 사용자 지정 시작 방향을 quaternion reference로 보존하는 runtime 상태다. */
    relative_heading_tracker_t tracker;
    bool reference_valid;
    uint64_t reference_generation;
} witness_heading_state_t;

typedef enum
{
    WITNESS_HEADING_UNAVAILABLE = 0,
    WITNESS_HEADING_REFERENCE_CAPTURED,
    WITNESS_HEADING_AVAILABLE
} witness_heading_result_t;

static volatile sig_atomic_t witness_stop_requested = 0;

static void witness_handle_signal(int signal_number)
{
    (void)signal_number;
    witness_stop_requested = 1;
}

static uint64_t witness_monotonic_ns(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
    {
        return 0U;
    }

    return
        (uint64_t)now.tv_sec * UINT64_C(1000000000)
        + (uint64_t)now.tv_nsec;
}

static void witness_sleep_microseconds(unsigned int microseconds)
{
    struct timespec duration;

    duration.tv_sec = (time_t)(microseconds / 1000000U);
    duration.tv_nsec = (long)(microseconds % 1000000U) * 1000L;

    while (nanosleep(&duration, &duration) != 0 && errno == EINTR)
    {
        if (witness_stop_requested)
        {
            break;
        }
    }
}

static double witness_normalize_heading_deg(double heading_deg)
{
    double normalized = fmod(heading_deg, 360.0);

    /* filter 계약은 [0,360)이므로 -10도는 같은 방향인 350도로 바꾼다. */
    if (normalized < 0.0)
    {
        normalized += 360.0;
    }
    if (normalized >= 360.0)
    {
        normalized -= 360.0;
    }

    return normalized;
}

static bool witness_testbed_config_is_valid(void)
{
    return isfinite(WITNESS_FIXED_X_MM)
        && isfinite(WITNESS_FIXED_Y_MM)
        && fabs(WITNESS_FIXED_X_MM) <= (double)FLT_MAX
        && fabs(WITNESS_FIXED_Y_MM) <= (double)FLT_MAX
        && isfinite(WITNESS_INITIAL_HEADING_DEG)
        && (WITNESS_RELATIVE_HEADING_SIGN == 1.0
            || WITNESS_RELATIVE_HEADING_SIGN == -1.0);
}

static void witness_convert_snapshot(
    const bno086_snapshot_t *source,
    position_imu_sample_t *destination
)
{
    memset(destination, 0, sizeof(*destination));

    destination->startup_ready = source->startup_ready;
    destination->orientation_sequence = source->orientation_sequence;
    destination->mag_sequence = source->mag_sequence;
    destination->orientation_time_s =
        (double)source->last_orientation_update_ns / 1000000000.0;
    destination->mag_time_s =
        (double)source->last_mag_update_ns / 1000000000.0;
    destination->quaternion.real = source->quat_real;
    destination->quaternion.i = source->quat_i;
    destination->quaternion.j = source->quat_j;
    destination->quaternion.k = source->quat_k;
    destination->rotation_accuracy = source->rotation_accuracy;
    destination->mag_accuracy = source->mag_accuracy;
}

static bool witness_timestamp_is_fresh(
    uint64_t timestamp_ns,
    uint64_t maximum_age_ns,
    uint64_t now_ns
)
{
    return timestamp_ns != 0U
        && now_ns >= timestamp_ns
        && now_ns - timestamp_ns <= maximum_age_ns;
}

static bool witness_snapshot_is_fresh(
    const bno086_snapshot_t *snapshot,
    uint64_t now_ns
)
{
    double norm_squared;

    /* RV와 Mag가 모두 최근 report여야 기준/출력 heading의 시간 근거가 유효하다. */
    if (snapshot == NULL
        || !snapshot->startup_ready
        || snapshot->orientation_sequence == 0U
        || snapshot->mag_sequence == 0U
        || !witness_timestamp_is_fresh(
                snapshot->last_orientation_update_ns,
                WITNESS_RV_STALE_NS,
                now_ns
            )
        || !witness_timestamp_is_fresh(
                snapshot->last_mag_update_ns,
                WITNESS_MAG_STALE_NS,
                now_ns
            ))
    {
        return false;
    }

    norm_squared =
        snapshot->quat_real * snapshot->quat_real
        + snapshot->quat_i * snapshot->quat_i
        + snapshot->quat_j * snapshot->quat_j
        + snapshot->quat_k * snapshot->quat_k;

    return isfinite(norm_squared) && norm_squared > 1.0e-12;
}

static bool witness_snapshot_accuracy_ready(
    const bno086_snapshot_t *snapshot
)
{
    return snapshot != NULL
        && snapshot->rotation_accuracy >= WITNESS_MIN_ACCURACY
        && snapshot->mag_accuracy >= WITNESS_MIN_ACCURACY;
}

static void witness_heading_state_initialize(witness_heading_state_t *state)
{
    memset(state, 0, sizeof(*state));
}

/*
 * accuracy gate는 reference를 잡을 때만 적용한다. 운영 중 accuracy가
 * 일시적으로 2 미만이 되는 것만으로 기준을 폐기하지 않는다.
 * startup loss, generation 변경, RV/Mag stale은 기준을 폐기한다.
 * 기준이 폐기된 동안 IPC도 중지하고, 센서가 회복되더라도 사용자가 차량을
 * 알려진 초기 방향에 다시 맞춘 후에만 새 reference를 잡는다.
 * 예를 들어 운행 중 accuracy가 2에서 1로 잠깐 내려가도 heading 연속성은
 * 유지한다. 반면 BNO reset으로 generation이 바뀌면 이전 quaternion과 새
 * quaternion의 기준 관계가 사라졌으므로 자동으로 이어 붙이지 않는다.
 */
static witness_heading_result_t witness_heading_update(
    witness_heading_state_t *state,
    const bno086_snapshot_t *snapshot,
    uint64_t now_ns,
    vehicle_state_t *vehicle_state
)
{
    position_imu_sample_t sample;
    double relative_heading_deg;
    bool reference_captured = false;

    if (state == NULL || snapshot == NULL || vehicle_state == NULL)
    {
        return WITNESS_HEADING_UNAVAILABLE;
    }

    if (!witness_snapshot_is_fresh(snapshot, now_ns)
        || (state->reference_valid
            && snapshot->bno_generation != state->reference_generation))
    {
        state->reference_valid = false;
        return WITNESS_HEADING_UNAVAILABLE;
    }

    witness_convert_snapshot(snapshot, &sample);

    if (!state->reference_valid)
    {
        if (!witness_snapshot_accuracy_ready(snapshot))
        {
            return WITNESS_HEADING_UNAVAILABLE;
        }

        /* 사용자가 맞춘 이 자세를 상대 0도로 저장하고 testbed 초기각을 더한다. */
        relative_heading_reset(&state->tracker, &sample);
        state->reference_generation = snapshot->bno_generation;
        state->reference_valid = true;
        reference_captured = true;
        relative_heading_deg = 0.0;
    }
    else
    {
        /* sample timestamp 기준 heading을 전송해 payload 시각과 맞춘다. */
        relative_heading_deg = relative_heading_at_deg(
            &state->tracker,
            &sample,
            sample.orientation_time_s
        );
    }

    /* 고정 배치 좌표에 시작 절대각 + 부호가 반영된 상대 yaw를 결합한다. */
    vehicle_state->detection_time =
        (int64_t)snapshot->last_orientation_update_ns;
    vehicle_state->x_mm = (float)WITNESS_FIXED_X_MM;
    vehicle_state->y_mm = (float)WITNESS_FIXED_Y_MM;
    vehicle_state->heading_deg = (float)witness_normalize_heading_deg(
        WITNESS_INITIAL_HEADING_DEG
        + WITNESS_RELATIVE_HEADING_SIGN * relative_heading_deg
    );

    return reference_captured
        ? WITNESS_HEADING_REFERENCE_CAPTURED
        : WITNESS_HEADING_AVAILABLE;
}

static bool witness_imu_stop_is_requested(witness_imu_worker_t *worker)
{
    bool stop_requested;

    pthread_mutex_lock(&worker->mutex);
    stop_requested = worker->stop_requested;
    pthread_mutex_unlock(&worker->mutex);

    return stop_requested;
}

static void witness_imu_publish_start(
    witness_imu_worker_t *worker,
    bool ready,
    const char *error
)
{
    pthread_mutex_lock(&worker->mutex);
    worker->ready = ready;
    worker->start_finished = true;
    if (error != NULL)
    {
        snprintf(worker->start_error, sizeof(worker->start_error), "%s", error);
    }
    pthread_cond_broadcast(&worker->start_condition);
    pthread_mutex_unlock(&worker->mutex);
}

static int witness_setup_feature(
    bno086_t *imu,
    const char *name,
    uint8_t sensor_id,
    uint32_t requested_interval_us,
    uint32_t *applied_interval_us,
    unsigned int report_timeout_ms,
    char *error,
    size_t error_size
)
{
    /* FC 설정 확인과 실제 input report 확인을 모두 통과해야 feature가 준비됐다. */
    if (bno086_enable_feature(
            imu,
            sensor_id,
            requested_interval_us,
            750U
        ) <= 0)
    {
        snprintf(error, error_size, "%s FC 설정 응답 실패", name);
        return -1;
    }

    if (bno086_wait_for_sensor_report(
            imu,
            sensor_id,
            report_timeout_ms
        ) <= 0)
    {
        snprintf(error, error_size, "%s 실제 input report 실패", name);
        return -1;
    }

    if (bno086_get_feature_interval(
            imu,
            sensor_id,
            applied_interval_us
        ) <= 0
        || *applied_interval_us == 0U)
    {
        snprintf(error, error_size, "%s 실제 FC interval 없음", name);
        return -1;
    }

    return 0;
}

static void *witness_imu_thread_main(void *argument)
{
    witness_imu_worker_t *worker = argument;
    char error[BNO086_ERROR_TEXT_SIZE] = "";
    char summary[BNO086_ERROR_TEXT_SIZE] = "";

    if (bno086_begin(&worker->imu) <= 0)
    {
        (void)bno086_status_summary(&worker->imu, summary, sizeof(summary));
        snprintf(error, sizeof(error), "BNO086 begin 실패: %.120s", summary);
        witness_imu_publish_start(worker, false, error);
        bno086_close(&worker->imu);
        return NULL;
    }

    /* 관찰 방향에 필요한 RV/Mag만 켜며 PAA/가속도 feature는 요청하지 않는다. */
    if (witness_setup_feature(
            &worker->imu,
            "Rotation Vector",
            BNO086_SENSOR_ROTATION_VECTOR,
            WITNESS_RV_INTERVAL_US,
            &worker->rotation_interval_us,
            750U,
            error,
            sizeof(error)
        ) != 0
        || witness_setup_feature(
            &worker->imu,
            "Magnetometer",
            BNO086_SENSOR_MAGNETOMETER_CALIBRATED,
            WITNESS_MAG_INTERVAL_US,
            &worker->mag_interval_us,
            1000U,
            error,
            sizeof(error)
        ) != 0
        || bno086_start_dynamic_calibration(&worker->imu) <= 0)
    {
        if (error[0] == '\0')
        {
            snprintf(error, sizeof(error), "동적 보정 명령 응답 실패");
        }
        witness_imu_publish_start(worker, false, error);
        bno086_close(&worker->imu);
        return NULL;
    }

    /* 여기서 ready가 되면 두 feature 모두 FC와 실제 report까지 확인된 상태다. */
    witness_imu_publish_start(worker, true, NULL);

    while (!witness_imu_stop_is_requested(worker))
    {
        (void)bno086_update(&worker->imu);
        witness_sleep_microseconds(WITNESS_BNO_UPDATE_INTERVAL_US);
    }

    bno086_close(&worker->imu);
    return NULL;
}

static int witness_imu_worker_start(witness_imu_worker_t *worker)
{
    bno086_config_t config;
    int result;

    memset(worker, 0, sizeof(*worker));
    if (pthread_mutex_init(&worker->mutex, NULL) != 0)
    {
        return -1;
    }
    if (pthread_cond_init(&worker->start_condition, NULL) != 0)
    {
        pthread_mutex_destroy(&worker->mutex);
        return -1;
    }

    bno086_default_config(&config);
    if (bno086_init(&worker->imu, &config) <= 0)
    {
        pthread_cond_destroy(&worker->start_condition);
        pthread_mutex_destroy(&worker->mutex);
        return -1;
    }

    result = pthread_create(
        &worker->thread,
        NULL,
        witness_imu_thread_main,
        worker
    );
    if (result != 0)
    {
        fprintf(stderr, "[-] BNO086 Thread 생성 실패: %s\n", strerror(result));
        bno086_deinit(&worker->imu);
        pthread_cond_destroy(&worker->start_condition);
        pthread_mutex_destroy(&worker->mutex);
        return -1;
    }
    worker->thread_created = true;

    /* caller는 startup 성공/실패가 확정될 때까지 기다려 미완성 snapshot을 쓰지 않는다. */
    pthread_mutex_lock(&worker->mutex);
    while (!worker->start_finished)
    {
        pthread_cond_wait(&worker->start_condition, &worker->mutex);
    }
    result = worker->ready ? 0 : -1;
    pthread_mutex_unlock(&worker->mutex);

    return result;
}

static void witness_imu_worker_stop(witness_imu_worker_t *worker)
{
    if (!worker->thread_created)
    {
        return;
    }

    pthread_mutex_lock(&worker->mutex);
    worker->stop_requested = true;
    pthread_cond_broadcast(&worker->start_condition);
    pthread_mutex_unlock(&worker->mutex);

    pthread_join(worker->thread, NULL);
    bno086_deinit(&worker->imu);
    pthread_cond_destroy(&worker->start_condition);
    pthread_mutex_destroy(&worker->mutex);
    worker->thread_created = false;
}

static int witness_wait_for_enter_or_quit(void)
{
    char input[32];

    while (!witness_stop_requested)
    {
        struct pollfd descriptor = {
            .fd = STDIN_FILENO,
            .events = POLLIN,
            .revents = 0
        };
        int result = poll(&descriptor, 1U, 100);

        if (result < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return -1;
        }
        if (result == 0)
        {
            continue;
        }
        if ((descriptor.revents & (POLLIN | POLLHUP | POLLERR)) == 0)
        {
            continue;
        }
        if (fgets(input, sizeof(input), stdin) == NULL
            || input[0] == 'q' || input[0] == 'Q')
        {
            return -1;
        }
        return 0;
    }

    return -1;
}

/*
 * 하드코딩한 초기 heading은 차량이 실제로 그 방향을 볼 때만 의미가 있다.
 * 따라서 calibration 중 임의 자세에서 자동 reference를 잡지 않고,
 * RV/Mag 준비 후 사용자가 차량을 지정 방향에 맞춘 경계 뒤의 새 RV를 쓴다.
 * Enter 직전 snapshot도 사용하지 않는다. Enter 뒤 orientation sequence가 증가한
 * fresh sample을 기다려 사용자가 자세를 맞춘 실제 경계를 reference로 삼는다.
 * 이 수동 단계가 필요한 이유는 BNO가 testbed의 +Y가 어디인지 스스로 알 수
 * 없기 때문이다. 센서는 절대 yaw를 주지만 설치된 차량의 기준 방향은 사람이
 * 알려줘야 고정 좌표계와 일치한다.
 */
static int witness_wait_for_reference(
    witness_imu_worker_t *worker,
    witness_heading_state_t *heading_state,
    vehicle_state_t *vehicle_state,
    bno086_snapshot_t *captured_snapshot
)
{
    bno086_snapshot_t snapshot;
    uint64_t next_status_ns = 0U;
    uint64_t previous_orientation_sequence;

    heading_state->reference_valid = false;
    puts("[*] RV/Mag calibration 준비를 기다립니다.");

    while (!witness_stop_requested)
    {
        uint64_t now_ns = witness_monotonic_ns();

        memset(&snapshot, 0, sizeof(snapshot));
        if (bno086_get_snapshot(&worker->imu, &snapshot) > 0
            && witness_snapshot_is_fresh(&snapshot, now_ns)
            && witness_snapshot_accuracy_ready(&snapshot))
        {
            break;
        }

        if (now_ns >= next_status_ns)
        {
            printf(
                "\r    startup=%s RV/Mag=%u/%u reports=%s      ",
                snapshot.startup_ready ? "READY" : "WAIT",
                snapshot.rotation_accuracy,
                snapshot.mag_accuracy,
                witness_snapshot_is_fresh(&snapshot, now_ns)
                    ? "fresh"
                    : "stale"
            );
            fflush(stdout);
            next_status_ns = now_ns + UINT64_C(250000000);
        }
        witness_sleep_microseconds(10000U);
    }
    if (witness_stop_requested)
    {
        return -1;
    }

    printf("\n[+] RV/Mag accuracy=%u/%u, reports=fresh\n",
           snapshot.rotation_accuracy, snapshot.mag_accuracy);
    printf(
        "[*] 차량을 testbed heading %.1f° 방향에 정확히 맞춘 뒤 Enter "
        "(중단 q): ",
        WITNESS_INITIAL_HEADING_DEG
    );
    fflush(stdout);
    if (witness_wait_for_enter_or_quit() != 0)
    {
        return -1;
    }

    previous_orientation_sequence = snapshot.orientation_sequence;
    while (!witness_stop_requested)
    {
        uint64_t now_ns = witness_monotonic_ns();
        witness_heading_result_t result;

        if (bno086_get_snapshot(&worker->imu, &snapshot) <= 0
            || snapshot.orientation_sequence == previous_orientation_sequence
            || !witness_snapshot_is_fresh(&snapshot, now_ns)
            || !witness_snapshot_accuracy_ready(&snapshot))
        {
            witness_sleep_microseconds(2000U);
            continue;
        }

        result = witness_heading_update(
            heading_state,
            &snapshot,
            now_ns,
            vehicle_state
        );
        if (result == WITNESS_HEADING_REFERENCE_CAPTURED)
        {
            *captured_snapshot = snapshot;
            return 0;
        }
    }

    return -1;
}

static void witness_configure_send_timeout(int file_descriptor)
{
    struct timeval timeout;

    timeout.tv_sec = WITNESS_SEND_TIMEOUT_MS / 1000L;
    timeout.tv_usec =
        (WITNESS_SEND_TIMEOUT_MS % 1000L) * 1000L;

    /* filter 정지 시 send가 무기한 막혀 BNO freshness 감시가 멈추지 않게 한다. */
    (void)setsockopt(
        file_descriptor,
        SOL_SOCKET,
        SO_SNDTIMEO,
        &timeout,
        sizeof(timeout)
    );
}

/*
 * detection_time은 heading을 만든 RV sample의 CLOCK_MONOTONIC ns다.
 * 현재 witness_filter는 payload 필드를 판정에 사용하지 않고 IPC 수신
 * 시각으로 freshness를 관리하므로, 이 값은 샘플 추적용 정보다.
 * header timestamp에도 같은 값을 넣어 payload와 IPC envelope의 sample 경계를
 * 일치시킨다. sequence는 성공한 송신에서만 증가한다.
 * x_mm/y_mm는 미터나 센티미터가 아니라 millimeter이고 heading은 degree다.
 * 단위가 다른 accident payload와 혼동하지 않도록 vehicle_state_t 계약을 그대로
 * 사용한다.
 */
static int witness_send_state(
    int file_descriptor,
    const vehicle_state_t *vehicle_state,
    uint32_t *next_sequence
)
{
    ipc_header_t header;
    int result;

    if (file_descriptor < 0
        || vehicle_state == NULL
        || next_sequence == NULL
        || *next_sequence == 0U
        || vehicle_state->detection_time <= 0
        || !isfinite(vehicle_state->x_mm)
        || !isfinite(vehicle_state->y_mm)
        || !isfinite(vehicle_state->heading_deg)
        || vehicle_state->heading_deg < 0.0f
        || vehicle_state->heading_deg >= 360.0f)
    {
        return IPC_ERROR_SEND;
    }

    memset(&header, 0, sizeof(header));
    header.type = IPC_MSG_SENSOR_DATA;
    header.flags = IPC_FLAG_NONE;
    header.sequence = *next_sequence;
    header.timestamp = (uint64_t)vehicle_state->detection_time;

    result = ipc_send(
        file_descriptor,
        &header,
        vehicle_state,
        sizeof(*vehicle_state)
    );
    if (result == IPC_SUCCESS)
    {
        *next_sequence = *next_sequence == UINT32_MAX
                       ? 1U
                       : *next_sequence + 1U;
    }

    return result;
}

int main(void)
{
    witness_imu_worker_t imu_worker;
    witness_heading_state_t heading_state;
    bno086_snapshot_t snapshot;
    vehicle_state_t vehicle_state;
    uint32_t next_sequence = 1U;
    uint64_t next_publish_ns = 0U;
    uint64_t next_connect_ns = 0U;
    int ipc_file_descriptor = -1;
    bool reference_was_valid = false;
    int result = EXIT_FAILURE;

    signal(SIGINT, witness_handle_signal);
    signal(SIGTERM, witness_handle_signal);
    signal(SIGPIPE, SIG_IGN);

    puts("============================================================");
    puts(" BNO086 Witness Heading Sensor");
    puts("============================================================");
    if (!witness_testbed_config_is_valid())
    {
        fputs("[-] witness testbed X/Y/heading/sign 설정이 유효하지 않습니다.\n",
              stderr);
        return EXIT_FAILURE;
    }
    printf(
        "fixed_position=(%.1f, %.1f) mm  initial_heading=%.1f deg\n",
        WITNESS_FIXED_X_MM,
        WITNESS_FIXED_Y_MM,
        WITNESS_INITIAL_HEADING_DEG
    );

    if (witness_imu_worker_start(&imu_worker) != 0)
    {
        fprintf(
            stderr,
            "[-] BNO086 시작 실패: %s\n",
            imu_worker.start_error[0] != '\0'
                ? imu_worker.start_error
                : "unknown error"
        );
        if (imu_worker.thread_created)
        {
            witness_imu_worker_stop(&imu_worker);
        }
        return EXIT_FAILURE;
    }

    printf(
        "[+] BNO086 ready: RV interval=%u us, Mag interval=%u us\n",
        imu_worker.rotation_interval_us,
        imu_worker.mag_interval_us
    );
    puts("[*] 동적 보정 중입니다. 센서를 방해 자성체에서 멀리 두고");
    puts("    천천히 여러 축으로 움직여 RV/Mag accuracy를 2 이상으로 올리세요.");

    witness_heading_state_initialize(&heading_state);
    memset(&vehicle_state, 0, sizeof(vehicle_state));

    /*
     * Main은 reference 유효성 검사, filter 재연결, 20 Hz publish만 담당한다.
     * BNO generation/stale loss가 나면 즉시 publish를 멈추며 마지막 heading을
     * 계속 보내지 않는다. filter가 없어도 BNO Thread는 독립적으로 계속 drain한다.
     */
    while (!witness_stop_requested)
    {
        uint64_t now_ns = witness_monotonic_ns();
        witness_heading_result_t heading_result = WITNESS_HEADING_UNAVAILABLE;

        if (!heading_state.reference_valid)
        {
            if (witness_wait_for_reference(
                    &imu_worker,
                    &heading_state,
                    &vehicle_state,
                    &snapshot
                ) != 0)
            {
                break;
            }
            printf(
                "[+] heading reference captured: RV/Mag=%u/%u, "
                "generation=%llu\n",
                snapshot.rotation_accuracy,
                snapshot.mag_accuracy,
                (unsigned long long)snapshot.bno_generation
            );
            reference_was_valid = true;
            next_publish_ns = 0U;
            continue;
        }

        if (bno086_get_snapshot(&imu_worker.imu, &snapshot) > 0)
        {
            heading_result = witness_heading_update(
                &heading_state,
                &snapshot,
                now_ns,
                &vehicle_state
            );
        }
        else
        {
            heading_state.reference_valid = false;
        }

        if (heading_result == WITNESS_HEADING_UNAVAILABLE
            && reference_was_valid)
        {
            puts(
                "[!] BNO startup/generation/freshness loss: "
                "IPC publish를 중지하고 사용자 재기준을 대기합니다."
            );
        }
        reference_was_valid = heading_state.reference_valid;

        /* filter가 늦게 시작해도 0.5초 간격으로 재연결하며 센서 기준은 유지한다. */
        if (ipc_file_descriptor < 0 && now_ns >= next_connect_ns)
        {
            ipc_file_descriptor = ipc_client_connect(
                WITNESS_FILTER_SOCKET_PATH
            );
            if (ipc_file_descriptor >= 0)
            {
                witness_configure_send_timeout(ipc_file_descriptor);
                puts("[+] witness_filter IPC connected");
            }
            else
            {
                next_connect_ns = now_ns + WITNESS_RECONNECT_INTERVAL_NS;
            }
        }

        /* reference와 연결이 모두 유효할 때만 가장 최근 한 상태를 20 Hz로 보낸다. */
        if (heading_state.reference_valid
            && ipc_file_descriptor >= 0
            && now_ns >= next_publish_ns)
        {
            if (witness_send_state(
                    ipc_file_descriptor,
                    &vehicle_state,
                    &next_sequence
                ) != IPC_SUCCESS)
            {
                fprintf(stderr, "[!] witness_filter IPC send 실패, 재연결 대기\n");
                ipc_close(ipc_file_descriptor);
                ipc_file_descriptor = -1;
                next_connect_ns = now_ns + WITNESS_RECONNECT_INTERVAL_NS;
            }
            next_publish_ns = now_ns + WITNESS_PUBLISH_INTERVAL_NS;
        }

        witness_sleep_microseconds(5000U);
    }

    result = EXIT_SUCCESS;
    if (ipc_file_descriptor >= 0)
    {
        ipc_close(ipc_file_descriptor);
    }
    witness_imu_worker_stop(&imu_worker);

    puts("[*] Witness sensor stopped");
    return result;
}
