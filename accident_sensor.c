#define _POSIX_C_SOURCE 200809L

/*
 * ==================================================
 * 사고차량용 PAA5100JE + BNO086 센서 프로세스
 * ==================================================
 *
 * 기존 read_flow.py의 production 경로를 C11과 pthread로 이식한 실행 파일이다.
 *
 * Thread 역할:
 *
 *     BNO086 I/O Thread
 *          - BNO086을 유일하게 소유
 *          - startup, feature 설정, SH-2 packet drain
 *
 *     Main / Position Thread
 *          - PAA5100JE를 유일하게 소유
 *          - BNO snapshot을 복사
 *          - 상대 heading과 body -> world 위치 적분
 *          - 사용자 calibration 명령 처리
 *
 *     Collision Detector Thread
 *          - BNO086 event ring의 독립 cursor로 모든 가속도 sample을 관측
 *          - 실측으로 확정한 복수 지표가 시간 창 안에서 일치할 때 1회 판정
 *
 *     IPC Sender Thread
 *          - 판정 경로와 분리된 bounded queue에서 사고 이벤트를 재전송
 *          - /tmp/p2p.sock 장애가 sensor 처리 시간을 막지 않게 한다.
 *
 * 이 실행 파일은 사고차량 전용이다. 관찰차량처럼 witness_filter에 주기 상태를
 * 보내지 않으며, PAA 위치와 충돌 판정이 모두 준비된 뒤 사고 rising edge만
 * P2P 프로세스로 보낸다. PAA나 BNO 중 하나가 없을 때 위치를 추측하는 fallback도
 * 두지 않는다. 장치 소유권을 Thread별로 한 곳에 고정해 같은 I2C/SPI stream을
 * 둘 이상의 실행 흐름이 동시에 소비하지 않게 한다.
 *
 * 쉽게 말하면 이 프로세스에는 서로 다른 두 일이 있다.
 *
 * 1. PAA의 바닥 이동량과 BNO의 방향을 합쳐 사고차량의 현재 위치를 갱신한다.
 * 2. BNO 가속도 흐름에서 충돌 후보를 찾고, 그 순간의 위치를 붙여 P2P에 알린다.
 *
 * 두 계산은 같은 BNO sample을 필요로 하지만 서로 sample을 빼앗아서는 안 된다.
 * 위치는 최신 snapshot을 복사하고, 충돌은 독립 event cursor로 모든 가속도
 * sample을 읽는다. IPC가 느려져도 센서 읽기가 밀리지 않도록 송신도 분리한다.
 */

#include "bno086.h"
#include "paa5100je.h"
#include "position_fusion.h"
#include "ipc.h"
#include "protocol.h"
#include "p2p_protocol.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
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

#define SENSOR_CONFIG_FILE                 "config.json"
#define SENSOR_DEFAULT_HEIGHT_CM           3.5
#define SENSOR_DEFAULT_SCALER              400.8
#define SENSOR_IMU_READY_TIMEOUT_MS        2500U
#define SENSOR_POST_ENTER_TIMEOUT_MS       5000U
#define SENSOR_HEADING_CHECK_TARGET_DEG     90.0
#define SENSOR_HEADING_CHECK_TOLERANCE_DEG  15.0
#define SENSOR_BNO_UPDATE_INTERVAL_US      2000U
#define SENSOR_MAG_INTERVAL_US             20000U
#define SENSOR_ACCEL_INTERVAL_US           10000U
#define SENSOR_STATUS_LINE_SIZE            256U
#define SENSOR_STATUS_TTY_INTERVAL_NS       UINT64_C(100000000)
#define SENSOR_STATUS_LOG_INTERVAL_NS       UINT64_C(1000000000)
#define SENSOR_P2P_SOCKET_PATH              "/tmp/p2p.sock"
#define SENSOR_IPC_QUEUE_CAPACITY           8U
#define SENSOR_DV_QUEUE_CAPACITY            256U
#define SENSOR_ITS_EPOCH_UNIX_MS            UINT64_C(1072915200000)
#define SENSOR_ITS_LEAP_SECONDS_MS          UINT64_C(5000)

/*
 * TODO(testbed): 실제 배치가 확정되면 사고차량의 알려진 시작 좌표와
 * 시작 방향을 여기에 기록한다. ACCIDENT_FUSION_TO_TESTBED_DEG는 현재
 * 융합 좌표축을 testbed +X/+Y 축으로 옮기는 표준 CCW 회전각이고,
 * ACCIDENT_START_HEADING_DEG는 filter의 0°=+Y, 시계방향 증가 규약이다.
 * BNO 상대 yaw 부호는 ACCIDENT_RELATIVE_HEADING_SIGN으로 분리했다.
 * 이 값들과 장착축은 PAA 실기기 ±90° 이동·회전 시험에서 확정해야 한다.
 * 예를 들어 차량을 testbed (1000, 500) mm에 놓고 +Y를 보게 시작한다면 시작
 * 좌표와 heading을 그 배치값으로 넣는다. 융합 내부 원점 (0,0)은 그대로 두고
 * 외부에 공개할 때만 이 고정 offset/회전을 적용한다.
 */
#define ACCIDENT_START_X_MM                 0.0
#define ACCIDENT_START_Y_MM                 0.0
#define ACCIDENT_START_HEADING_DEG          0.0
#define ACCIDENT_FUSION_TO_TESTBED_DEG      0.0
#define ACCIDENT_RELATIVE_HEADING_SIGN       1.0
#define SENSOR_DEGREES_TO_RADIANS           0.017453292519943295

#define COLLISION_METRIC_RAW_ACCEL          (UINT32_C(1) << 0)
#define COLLISION_METRIC_LINEAR_ACCEL       (UINT32_C(1) << 1)
#define COLLISION_METRIC_RAW_JERK           (UINT32_C(1) << 2)
#define COLLISION_METRIC_LINEAR_JERK        (UINT32_C(1) << 3)
#define COLLISION_METRIC_LINEAR_DELTA_V     (UINT32_C(1) << 4)
#define COLLISION_METRIC_ALL                ((UINT32_C(1) << 5) - 1U)

typedef struct
{
    double height_cm;
    double scaler;
} sensor_config_t;

/* BNO 장치의 startup과 지속 packet drain을 전담하고 적용 interval을 공개한다. */
typedef struct
{
    bno086_t imu;

    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t start_condition;

    bool thread_created;
    bool start_finished;
    bool ready;
    bool stop_requested;

    uint32_t raw_accel_interval_us;
    uint32_t linear_accel_interval_us;
    uint32_t gyro_interval_us;

    char start_error[BNO086_ERROR_TEXT_SIZE];
} imu_worker_t;

typedef enum
{
    COLLISION_DV_WINDOW_20_MS = 20,
    COLLISION_DV_WINDOW_50_MS = 50,
    COLLISION_DV_WINDOW_100_MS = 100,
    COLLISION_DV_WINDOW_200_MS = 200
} collision_dv_window_t;

typedef struct
{
    /* 어떤 지표를 쓸지와 RC-car 실측 후 확정할 모든 시간/크기 임계값이다. */
    bool enabled;
    uint32_t metric_mask;
    unsigned int minimum_evidence_count;
    collision_dv_window_t delta_v_window;
    unsigned int coincidence_window_ms;
    unsigned int cooldown_ms;
    unsigned int quiet_rearm_ms;
    bool require_position;

    double raw_accel_threshold_mps2;
    double linear_accel_threshold_mps2;
    double raw_jerk_threshold_mps3;
    double linear_jerk_threshold_mps3;
    double linear_delta_v_threshold_mps;
} collision_config_t;

/*
 * TODO(empirical): test_collision_capture.c의 실제 RC-car 결과를 검토한 뒤
 * mask/threshold/count/window를 채운다. enabled=false와 0 threshold가 기본이므로
 * 실측값을 넣기 전에는 우발적으로 사고 IPC가 발생하지 않는다.
 * 시험 중 정상 급가속·급제동·방지턱과 실제 충돌 기록을 사람이 채택/폐기한 뒤,
 * 서로 가장 잘 구분하는 지표와 임계값만 여기로 옮긴다. enabled를 먼저 켜거나
 * 0 임계값을 임의의 작은 값으로 바꾸지 않는다.
 */
static const collision_config_t collision_config = {
    .enabled = false,
    .metric_mask = 0U,
    .minimum_evidence_count = 0U,
    .delta_v_window = COLLISION_DV_WINDOW_100_MS,
    .coincidence_window_ms = 100U,
    .cooldown_ms = 2000U,
    .quiet_rearm_ms = 1000U,
    .require_position = true,
    .raw_accel_threshold_mps2 = 0.0,
    .linear_accel_threshold_mps2 = 0.0,
    .raw_jerk_threshold_mps3 = 0.0,
    .linear_jerk_threshold_mps3 = 0.0,
    .linear_delta_v_threshold_mps = 0.0
};

typedef struct
{
    double x;
    double y;
    double z;
} collision_vector3_t;

typedef struct
{
    /* 한 report stream의 직전 sample로 jerk의 연속성을 검증한다. */
    bool valid;
    uint8_t sequence;
    uint64_t generation;
    uint64_t monotonic_ns;
    collision_vector3_t value;
} collision_stream_state_t;

typedef struct
{
    /* Linear acceleration을 사다리꼴 적분한 고정 시간창 delta-V 벡터다. */
    collision_vector3_t values[SENSOR_DV_QUEUE_CAPACITY];
    size_t head;
    size_t count;
    size_t limit;
    collision_vector3_t sum;
} collision_delta_v_window_t;

typedef struct
{
    /* 검출 latch, 최근 evidence와 raw/linear stream을 한 lock 아래 보관한다. */
    collision_config_t config;
    bool armed;
    bool latched;
    uint64_t latched_at_ns;
    uint64_t last_evidence_ns;
    uint64_t last_event_ns;
    bool generation_valid;
    uint64_t generation;
    uint64_t evidence_ns[5];
    double evidence_values[5];
    collision_stream_state_t raw;
    collision_stream_state_t linear;
    collision_delta_v_window_t delta_v;
    double raw_interval_s;
    double linear_interval_s;
} collision_detector_state_t;

typedef struct
{
    /* Main 위치 Thread가 쓰고 충돌 Thread가 사고 순간 한 번 복사하는 상태다. */
    pthread_mutex_t mutex;
    bool initialized;
    bool valid;
    bool heading_valid;
    double x_cm;
    double y_cm;
    double heading_deg;
} position_publication_t;

typedef struct
{
    accident_event_payload_t payload;
    uint64_t monotonic_ns;
} collision_ipc_item_t;

typedef struct
{
    /* IPC 장애가 센서 sample 처리를 막지 않도록 사고 payload를 bounded queue에 둔다. */
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t condition;
    collision_ipc_item_t items[SENSOR_IPC_QUEUE_CAPACITY];
    size_t head;
    size_t count;
    bool initialized;
    bool thread_created;
    bool stop_requested;
    uint32_t next_sequence;
    uint64_t dropped;
    uint64_t sent;
} collision_ipc_worker_t;

typedef struct
{
    /* BNO event ring의 독립 cursor로 가속도 sample을 순서대로 소비한다. */
    pthread_t thread;
    pthread_mutex_t mutex;
    imu_worker_t *imu_worker;
    position_publication_t *position;
    collision_ipc_worker_t *ipc_worker;
    collision_detector_state_t detector;
    bool initialized;
    bool thread_created;
    bool stop_requested;
    bool arm_requested;
    bool startup_ready_seen;
    bool start_finished;
    bool ready;
    pthread_cond_t start_condition;
} collision_worker_t;

/*
 * run_measurement()의 'c' 명령 경계를 hardware 없이도 그대로 시험하기
 * 위한 작은 I/O seam이다. production에서는 아래 callback이 실제 BNO
 * snapshot과 PAA burst를 사용하고, 회귀시험에서만 결정론적 값을 넣는다.
 */
typedef int (*measurement_read_imu_fn)(
    void *context,
    position_imu_sample_t *sample
);

typedef int (*measurement_flush_flow_fn)(
    void *context,
    int16_t *delta_x_ticks,
    int16_t *delta_y_ticks
);

typedef double (*measurement_time_fn)(void *context);

typedef struct
{
    measurement_read_imu_fn read_imu;
    void *imu_context;

    measurement_flush_flow_fn flush_flow;
    void *flow_context;

    measurement_time_fn get_time_s;
    void *time_context;
} measurement_runtime_ops_t;

typedef enum
{
    MEASUREMENT_CALIBRATION_UNAVAILABLE = 0,
    MEASUREMENT_CALIBRATION_STARTED,
    MEASUREMENT_CALIBRATION_LOCKED,
    MEASUREMENT_CALIBRATION_NEEDS_MORE
} measurement_calibration_action_t;

typedef struct
{
    measurement_calibration_action_t action;
    bool fresh_heading_used;
    bool flow_flushed;
} measurement_calibration_result_t;

static volatile sig_atomic_t process_stop_requested = 0;
static char sensor_config_path[PATH_MAX] = SENSOR_CONFIG_FILE;

/* ========================================================================== */
/* 시각, signal, 문자열 보조 함수                                              */
/* ========================================================================== */

static void handle_signal(int signal_number)
{
    (void)signal_number;
    process_stop_requested = 1;
}

static uint64_t monotonic_time_ns(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
    {
        return 0U;
    }

    return
        (uint64_t)now.tv_sec * 1000000000ULL
        + (uint64_t)now.tv_nsec;
}

static double monotonic_time_seconds(void)
{
    return (double)monotonic_time_ns() / 1000000000.0;
}

static void sleep_microseconds(unsigned int microseconds)
{
    struct timespec duration;

    duration.tv_sec = (time_t)(microseconds / 1000000U);
    duration.tv_nsec = (long)(microseconds % 1000000U) * 1000L;

    while (nanosleep(&duration, &duration) != 0 && errno == EINTR)
    {
        if (process_stop_requested)
        {
            break;
        }
    }
}

static void trim_line(char *text)
{
    size_t length;
    size_t start = 0U;

    if (text == NULL)
    {
        return;
    }

    length = strlen(text);
    while (length > 0U && isspace((unsigned char)text[length - 1U]))
    {
        text[--length] = '\0';
    }
    while (text[start] != '\0' && isspace((unsigned char)text[start]))
    {
        start++;
    }
    if (start > 0U)
    {
        memmove(text, text + start, strlen(text + start) + 1U);
    }
}

/* 실행 위치와 무관하게 실행 파일 옆의 config.json을 사용한다. */
static void initialize_config_path(void)
{
    char executable_path[PATH_MAX];
    ssize_t length = readlink(
        "/proc/self/exe",
        executable_path,
        sizeof(executable_path) - 1U
    );

    if (length > 0 && (size_t)length < sizeof(executable_path))
    {
        char *separator;
        size_t directory_length;
        size_t file_name_length = strlen(SENSOR_CONFIG_FILE);

        executable_path[length] = '\0';
        separator = strrchr(executable_path, '/');
        if (separator == NULL)
        {
            return;
        }

        directory_length = (size_t)(separator - executable_path) + 1U;
        if (directory_length + file_name_length + 1U
            > sizeof(sensor_config_path))
        {
            return;
        }

        memcpy(sensor_config_path, executable_path, directory_length);
        memcpy(
            sensor_config_path + directory_length,
            SENSOR_CONFIG_FILE,
            file_name_length + 1U
        );
    }
}

static int stdin_line_ready(int timeout_ms)
{
    struct pollfd descriptor;
    int result;

    descriptor.fd = STDIN_FILENO;
    descriptor.events = POLLIN;
    descriptor.revents = 0;

    do
    {
        result = poll(&descriptor, 1U, timeout_ms);
    }
    while (result < 0 && errno == EINTR && !process_stop_requested);

    return result > 0 && (descriptor.revents & POLLIN) != 0;
}

static int read_line_prompt(
    const char *prompt,
    char *buffer,
    size_t buffer_size
)
{
    if (prompt != NULL)
    {
        fputs(prompt, stdout);
        fflush(stdout);
    }

    if (fgets(buffer, (int)buffer_size, stdin) == NULL)
    {
        return -1;
    }

    trim_line(buffer);
    return 0;
}

static int64_t timestamp_its_ms(void)
{
    struct timespec now;
    uint64_t unix_ms;

    if (clock_gettime(CLOCK_REALTIME, &now) != 0)
    {
        return 0;
    }

    unix_ms = (uint64_t)now.tv_sec * 1000ULL
            + (uint64_t)now.tv_nsec / 1000000ULL;
    if (unix_ms < SENSOR_ITS_EPOCH_UNIX_MS)
    {
        return 0;
    }
    return (int64_t)(
        unix_ms - SENSOR_ITS_EPOCH_UNIX_MS + SENSOR_ITS_LEAP_SECONDS_MS
    );
}

/* ========================================================================== */
/* 충돌 지표 계산과 rising-edge 판정                                           */
/* ========================================================================== */

/*
 * raw acceleration은 중력을 포함하고 linear acceleration은 SH-2가 중력을
 * 제거한 값이다. jerk는 같은 stream의 연속 두 sample 차 / 실제 설정 interval,
 * delta-V는 linear acceleration의 사다리꼴 적분 벡터 크기다. 서로 다른 물리량을
 * 임계값 하나로 섞지 않고 metric별 evidence로 유지한다.
 *
 * - acceleration: 지금 순간에 얼마나 큰 힘이 관측됐는지 본다.
 * - jerk: 가속도가 얼마나 갑자기 변했는지 본다.
 * - delta-V: 짧은 시간 동안 속도가 얼마나 변했는지 본다.
 *
 * RC카의 평상시 진동 하나만으로 사고가 되지 않도록, 선택한 지표 여러 개가
 * 가까운 시간 안에 함께 임계값을 넘는지를 확인한다.
 */

typedef struct
{
    double raw_accel_mps2;
    double linear_accel_mps2;
    double raw_jerk_mps3;
    double linear_jerk_mps3;
    double linear_delta_v_mps;
    bool raw_jerk_valid;
    bool linear_jerk_valid;
    bool linear_delta_v_valid;
} collision_observation_t;

static collision_vector3_t collision_vector_add(
    collision_vector3_t left,
    collision_vector3_t right
)
{
    collision_vector3_t result = {
        left.x + right.x,
        left.y + right.y,
        left.z + right.z
    };
    return result;
}

static collision_vector3_t collision_vector_subtract(
    collision_vector3_t left,
    collision_vector3_t right
)
{
    collision_vector3_t result = {
        left.x - right.x,
        left.y - right.y,
        left.z - right.z
    };
    return result;
}

static collision_vector3_t collision_vector_scale(
    collision_vector3_t value,
    double scale
)
{
    collision_vector3_t result = {
        value.x * scale,
        value.y * scale,
        value.z * scale
    };
    return result;
}

static double collision_vector_magnitude(collision_vector3_t value)
{
    return sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

static void collision_reset_evidence(collision_detector_state_t *state)
{
    memset(state->evidence_ns, 0, sizeof(state->evidence_ns));
    memset(state->evidence_values, 0, sizeof(state->evidence_values));
    state->last_evidence_ns = 0U;
}

static void collision_reset_delta_v(collision_detector_state_t *state)
{
    state->delta_v.head = 0U;
    state->delta_v.count = 0U;
    state->delta_v.sum = (collision_vector3_t){0.0, 0.0, 0.0};
}

static unsigned int collision_delta_v_duration_ms(
    collision_dv_window_t window
)
{
    switch (window)
    {
    case COLLISION_DV_WINDOW_20_MS:
        return 20U;
    case COLLISION_DV_WINDOW_50_MS:
        return 50U;
    case COLLISION_DV_WINDOW_200_MS:
        return 200U;
    case COLLISION_DV_WINDOW_100_MS:
    default:
        return 100U;
    }
}

static bool collision_config_is_valid(const collision_config_t *config)
{
    uint32_t mask;
    unsigned int selected = 0U;
    size_t index;

    if (config == NULL || !config->enabled
        || config->coincidence_window_ms == 0U
        || config->cooldown_ms == 0U
        || config->quiet_rearm_ms == 0U)
    {
        return false;
    }
    if (config->delta_v_window != COLLISION_DV_WINDOW_20_MS
        && config->delta_v_window != COLLISION_DV_WINDOW_50_MS
        && config->delta_v_window != COLLISION_DV_WINDOW_100_MS
        && config->delta_v_window != COLLISION_DV_WINDOW_200_MS)
    {
        return false;
    }
    mask = config->metric_mask & COLLISION_METRIC_ALL;
    for (index = 0U; index < 5U; index++)
    {
        if ((mask & (UINT32_C(1) << index)) != 0U)
        {
            selected++;
        }
    }
    if (mask == 0U || mask != config->metric_mask
        || config->minimum_evidence_count == 0U
        || config->minimum_evidence_count > selected)
    {
        return false;
    }
    if (((mask & COLLISION_METRIC_RAW_ACCEL) != 0U
         && (!isfinite(config->raw_accel_threshold_mps2)
             || config->raw_accel_threshold_mps2 <= 0.0))
        || ((mask & COLLISION_METRIC_LINEAR_ACCEL) != 0U
            && (!isfinite(config->linear_accel_threshold_mps2)
                || config->linear_accel_threshold_mps2 <= 0.0))
        || ((mask & COLLISION_METRIC_RAW_JERK) != 0U
            && (!isfinite(config->raw_jerk_threshold_mps3)
                || config->raw_jerk_threshold_mps3 <= 0.0))
        || ((mask & COLLISION_METRIC_LINEAR_JERK) != 0U
            && (!isfinite(config->linear_jerk_threshold_mps3)
                || config->linear_jerk_threshold_mps3 <= 0.0))
        || ((mask & COLLISION_METRIC_LINEAR_DELTA_V) != 0U
            && (!isfinite(config->linear_delta_v_threshold_mps)
                || config->linear_delta_v_threshold_mps <= 0.0)))
    {
        return false;
    }
    return true;
}

static void collision_detector_initialize(
    collision_detector_state_t *state,
    const collision_config_t *config,
    uint32_t raw_interval_us,
    uint32_t linear_interval_us
)
{
    double samples;

    memset(state, 0, sizeof(*state));
    state->config = *config;
    state->raw_interval_s = (double)raw_interval_us / 1000000.0;
    state->linear_interval_s = (double)linear_interval_us / 1000000.0;
    if (state->raw_interval_s <= 0.0)
    {
        state->raw_interval_s = 0.01;
    }
    if (state->linear_interval_s <= 0.0)
    {
        state->linear_interval_s = 0.01;
    }

    samples = ceil(
        (double)collision_delta_v_duration_ms(config->delta_v_window)
        / 1000.0 / state->linear_interval_s
    );
    if (samples < 1.0)
    {
        samples = 1.0;
    }
    if (samples > (double)SENSOR_DV_QUEUE_CAPACITY)
    {
        samples = (double)SENSOR_DV_QUEUE_CAPACITY;
    }
    state->delta_v.limit = (size_t)samples;
}

static void collision_detector_set_armed(
    collision_detector_state_t *state,
    bool armed
)
{
    /* arm/disarm은 이전 sample, evidence, latch를 모두 끊는 측정 구간 경계다. */
    state->armed = armed && collision_config_is_valid(&state->config);
    state->latched = false;
    state->latched_at_ns = 0U;
    state->raw.valid = false;
    state->linear.valid = false;
    state->last_event_ns = 0U;
    state->generation_valid = false;
    state->generation = 0U;
    collision_reset_delta_v(state);
    collision_reset_evidence(state);
}

static bool collision_stream_is_contiguous(
    collision_stream_state_t *stream,
    const bno086_event_t *event
)
{
    uint8_t expected;

    /* 직전 번호 다음 report가 아니면 두 값을 빼서 jerk를 만들 근거가 없다. */
    if (!stream->valid || stream->generation != event->bno_generation)
    {
        return false;
    }
    expected = (uint8_t)((unsigned int)stream->sequence + 1U);
    return event->report_sequence == expected
        && event->monotonic_ns >= stream->monotonic_ns;
}

static void collision_stream_update(
    collision_stream_state_t *stream,
    const bno086_event_t *event,
    collision_vector3_t value
)
{
    stream->valid = true;
    stream->sequence = event->report_sequence;
    stream->generation = event->bno_generation;
    stream->monotonic_ns = event->monotonic_ns;
    stream->value = value;
}

static double collision_delta_v_push(
    collision_detector_state_t *state,
    collision_vector3_t contribution,
    bool *valid
)
{
    size_t tail;

    /* 가장 오래된 기여 벡터를 빼고 새 기여를 더해 O(1) sliding sum을 유지한다. */
    if (state->delta_v.count == state->delta_v.limit)
    {
        collision_vector3_t removed =
            state->delta_v.values[state->delta_v.head];
        state->delta_v.sum = collision_vector_subtract(
            state->delta_v.sum,
            removed
        );
        state->delta_v.head =
            (state->delta_v.head + 1U) % SENSOR_DV_QUEUE_CAPACITY;
        state->delta_v.count--;
    }

    tail = (state->delta_v.head + state->delta_v.count)
         % SENSOR_DV_QUEUE_CAPACITY;
    state->delta_v.values[tail] = contribution;
    state->delta_v.sum = collision_vector_add(
        state->delta_v.sum,
        contribution
    );
    state->delta_v.count++;
    *valid = state->delta_v.count >= state->delta_v.limit;
    return *valid ? collision_vector_magnitude(state->delta_v.sum) : 0.0;
}

static void collision_record_evidence(
    collision_detector_state_t *state,
    uint32_t metric,
    size_t evidence_index,
    double value,
    double threshold,
    bool value_valid,
    uint64_t now_ns,
    bool *any_evidence
)
{
    if ((state->config.metric_mask & metric) != 0U
        && value_valid && value >= threshold)
    {
        state->evidence_ns[evidence_index] = now_ns;
        state->evidence_values[evidence_index] = value;
        state->last_evidence_ns = now_ns;
        *any_evidence = true;
    }
}

/*
 * 한 sensor report를 처리하는 순수 판정 경로다. false이면 이벤트 없음,
 * true이면 latch의 rising edge 한 번이다. sequence/generation 단절은 jerk,
 * delta-V와 이전 evidence를 모두 끊어 서로 다른 구간을 결합하지 않는다.
 * 선택된 지표 중 minimum_evidence_count개가 coincidence window 안에 있을 때만
 * 충돌 후보가 된다. 한 충돌 파형의 여러 peak가 중복 전송되지 않도록 latch하고,
 * cooldown과 quiet 시간이 모두 지난 뒤에만 다음 사고를 받을 수 있다.
 * 예를 들어 minimum=2이면 acceleration peak 하나만으로는 부족하고, 같은
 * coincidence 구간에 jerk 또는 delta-V 같은 두 번째 근거가 있어야 true다.
 */
static bool collision_detector_process(
    collision_detector_state_t *state,
    const bno086_event_t *event,
    collision_observation_t *observation
)
{
    collision_stream_state_t *stream;
    collision_vector3_t value;
    collision_vector3_t difference;
    bool contiguous;
    bool any_evidence = false;
    bool boundary_discontinuity = false;
    uint64_t coincidence_ns;
    unsigned int evidence_count = 0U;
    size_t index;

    memset(observation, 0, sizeof(*observation));
    if (!state->armed || !state->config.enabled
        || (event->type != BNO086_EVENT_ACCELEROMETER
            && event->type != BNO086_EVENT_LINEAR_ACCELERATION))
    {
        return false;
    }

    value = (collision_vector3_t){event->x, event->y, event->z};
    stream = event->type == BNO086_EVENT_ACCELEROMETER
           ? &state->raw : &state->linear;

    /* reset 전후 report를 연결하면 정상 startup 차이가 거대한 jerk처럼 보일 수 있다. */
    if (state->generation_valid
        && event->bno_generation != state->generation)
    {
        state->raw.valid = false;
        state->linear.valid = false;
        collision_reset_delta_v(state);
        collision_reset_evidence(state);
        state->last_evidence_ns = event->monotonic_ns;
        boundary_discontinuity = true;
    }
    state->generation_valid = true;
    state->generation = event->bno_generation;

    if (state->last_event_ns != 0U
        && event->monotonic_ns < state->last_event_ns)
    {
        state->raw.valid = false;
        state->linear.valid = false;
        collision_reset_delta_v(state);
        collision_reset_evidence(state);
        state->last_evidence_ns = event->monotonic_ns;
        boundary_discontinuity = true;
    }
    state->last_event_ns = event->monotonic_ns;
    if (boundary_discontinuity)
    {
        collision_stream_update(stream, event, value);
        return false;
    }

    contiguous = collision_stream_is_contiguous(stream, event);
    if (stream->valid && !contiguous)
    {
        collision_reset_evidence(state);
        state->last_evidence_ns = event->monotonic_ns;
        collision_reset_delta_v(state);
        collision_stream_update(stream, event, value);
        return false;
    }

    if (event->type == BNO086_EVENT_ACCELEROMETER)
    {
        observation->raw_accel_mps2 = collision_vector_magnitude(value);
        if (contiguous)
        {
            difference = collision_vector_subtract(value, stream->value);
            observation->raw_jerk_mps3 =
                collision_vector_magnitude(difference) / state->raw_interval_s;
            observation->raw_jerk_valid = true;
        }
    }
    else
    {
        observation->linear_accel_mps2 = collision_vector_magnitude(value);
        if (contiguous)
        {
            collision_vector3_t average = collision_vector_scale(
                collision_vector_add(value, stream->value),
                0.5
            );
            collision_vector3_t contribution = collision_vector_scale(
                average,
                state->linear_interval_s
            );

            difference = collision_vector_subtract(value, stream->value);
            observation->linear_jerk_mps3 =
                collision_vector_magnitude(difference)
                / state->linear_interval_s;
            observation->linear_jerk_valid = true;
            observation->linear_delta_v_mps = collision_delta_v_push(
                state,
                contribution,
                &observation->linear_delta_v_valid
            );
        }
    }
    collision_stream_update(stream, event, value);

    collision_record_evidence(
        state, COLLISION_METRIC_RAW_ACCEL, 0U,
        observation->raw_accel_mps2,
        state->config.raw_accel_threshold_mps2,
        event->type == BNO086_EVENT_ACCELEROMETER,
        event->monotonic_ns, &any_evidence
    );
    collision_record_evidence(
        state, COLLISION_METRIC_LINEAR_ACCEL, 1U,
        observation->linear_accel_mps2,
        state->config.linear_accel_threshold_mps2,
        event->type == BNO086_EVENT_LINEAR_ACCELERATION,
        event->monotonic_ns, &any_evidence
    );
    collision_record_evidence(
        state, COLLISION_METRIC_RAW_JERK, 2U,
        observation->raw_jerk_mps3,
        state->config.raw_jerk_threshold_mps3,
        observation->raw_jerk_valid,
        event->monotonic_ns, &any_evidence
    );
    collision_record_evidence(
        state, COLLISION_METRIC_LINEAR_JERK, 3U,
        observation->linear_jerk_mps3,
        state->config.linear_jerk_threshold_mps3,
        observation->linear_jerk_valid,
        event->monotonic_ns, &any_evidence
    );
    collision_record_evidence(
        state, COLLISION_METRIC_LINEAR_DELTA_V, 4U,
        observation->linear_delta_v_mps,
        state->config.linear_delta_v_threshold_mps,
        observation->linear_delta_v_valid,
        event->monotonic_ns, &any_evidence
    );

    if (state->latched)
    {
        uint64_t cooldown_ns =
            (uint64_t)state->config.cooldown_ms * 1000000ULL;
        uint64_t quiet_ns =
            (uint64_t)state->config.quiet_rearm_ms * 1000000ULL;
        bool cooldown_done = event->monotonic_ns >= state->latched_at_ns
            && event->monotonic_ns - state->latched_at_ns >= cooldown_ns;
        bool quiet_done = !any_evidence && state->last_evidence_ns != 0U
            && event->monotonic_ns >= state->last_evidence_ns
            && event->monotonic_ns - state->last_evidence_ns >= quiet_ns;

        /* 시간만 지났어도 충격 진동이 계속되면 rearm하지 않는다. */
        if (cooldown_done && quiet_done)
        {
            state->latched = false;
            collision_reset_evidence(state);
        }
        return false;
    }

    coincidence_ns =
        (uint64_t)state->config.coincidence_window_ms * 1000000ULL;
    for (index = 0U; index < 5U; index++)
    {
        uint32_t metric = UINT32_C(1) << index;
        if ((state->config.metric_mask & metric) != 0U
            && state->evidence_ns[index] != 0U
            && event->monotonic_ns >= state->evidence_ns[index]
            && event->monotonic_ns - state->evidence_ns[index]
               <= coincidence_ns)
        {
            evidence_count++;
        }
    }

    if (evidence_count >= state->config.minimum_evidence_count)
    {
        state->latched = true;
        state->latched_at_ns = event->monotonic_ns;
        return true;
    }
    return false;
}

/* ========================================================================== */
/* 위치 publication과 P2P IPC payload                                          */
/* ========================================================================== */

/*
 * 융합 좌표는 시작 자세 기준 cm이고 P2P payload는 testbed 기준 mm다. Main이
 * 좌표/heading을 mutex 아래 함께 공개하면 충돌 Thread가 판정 순간의 일관된
 * 사본을 얻어 IPC 계약으로 변환한다. 유효하지 않은 GNSS 계열 필드는 ETSI
 * unavailable sentinel을 쓰며, 알려진 local_x/local_y만 실제 위치로 채운다.
 * 즉 latitude/longitude sentinel은 좌표가 0이라는 뜻이 아니라 "이 차량은
 * GNSS 좌표를 제공하지 않는다"는 protocol 표현이다.
 */

static int position_publication_initialize(position_publication_t *publication)
{
    memset(publication, 0, sizeof(*publication));
    if (pthread_mutex_init(&publication->mutex, NULL) != 0)
    {
        return -1;
    }
    publication->initialized = true;
    return 0;
}

static void position_publication_deinitialize(
    position_publication_t *publication
)
{
    if (publication->initialized)
    {
        pthread_mutex_destroy(&publication->mutex);
        publication->initialized = false;
    }
}

static void position_publication_update(
    position_publication_t *publication,
    double x_cm,
    double y_cm,
    bool heading_valid,
    double heading_deg
)
{
    pthread_mutex_lock(&publication->mutex);
    publication->x_cm = x_cm;
    publication->y_cm = y_cm;
    publication->heading_deg = heading_deg;
    publication->valid = isfinite(x_cm) && isfinite(y_cm);
    publication->heading_valid = heading_valid && isfinite(heading_deg);
    pthread_mutex_unlock(&publication->mutex);
}

static bool accident_testbed_config_is_valid(void)
{
    return isfinite(ACCIDENT_START_X_MM)
        && isfinite(ACCIDENT_START_Y_MM)
        && fabs(ACCIDENT_START_X_MM) <= 1000000.0
        && fabs(ACCIDENT_START_Y_MM) <= 1000000.0
        && isfinite(ACCIDENT_START_HEADING_DEG)
        && isfinite(ACCIDENT_FUSION_TO_TESTBED_DEG)
        && (ACCIDENT_RELATIVE_HEADING_SIGN == 1.0
            || ACCIDENT_RELATIVE_HEADING_SIGN == -1.0);
}

static void accident_relative_to_testbed_cm(
    double relative_x_cm,
    double relative_y_cm,
    double *testbed_x_cm,
    double *testbed_y_cm
)
{
    double frame_angle_rad =
        ACCIDENT_FUSION_TO_TESTBED_DEG * SENSOR_DEGREES_TO_RADIANS;
    double cosine = cos(frame_angle_rad);
    double sine = sin(frame_angle_rad);

    *testbed_x_cm = ACCIDENT_START_X_MM / 10.0
        + relative_x_cm * cosine - relative_y_cm * sine;
    *testbed_y_cm = ACCIDENT_START_Y_MM / 10.0
        + relative_x_cm * sine + relative_y_cm * cosine;
}

static double accident_testbed_heading_deg(double relative_heading_deg)
{
    double heading = fmod(
        ACCIDENT_START_HEADING_DEG
        + ACCIDENT_RELATIVE_HEADING_SIGN * relative_heading_deg,
        360.0
    );

    if (heading < 0.0)
    {
        heading += 360.0;
    }
    return heading;
}

static void accident_position_publication_update(
    position_publication_t *publication,
    double relative_x_cm,
    double relative_y_cm,
    bool heading_valid,
    double relative_heading_deg
)
{
    double testbed_x_cm;
    double testbed_y_cm;

    accident_relative_to_testbed_cm(
        relative_x_cm,
        relative_y_cm,
        &testbed_x_cm,
        &testbed_y_cm
    );

    position_publication_update(
        publication,
        testbed_x_cm,
        testbed_y_cm,
        heading_valid,
        accident_testbed_heading_deg(relative_heading_deg)
    );
}

static void position_publication_invalidate(position_publication_t *publication)
{
    pthread_mutex_lock(&publication->mutex);
    publication->valid = false;
    publication->heading_valid = false;
    pthread_mutex_unlock(&publication->mutex);
}

static void position_publication_snapshot(
    position_publication_t *publication,
    position_publication_t *snapshot
)
{
    pthread_mutex_lock(&publication->mutex);
    snapshot->valid = publication->valid;
    snapshot->heading_valid = publication->heading_valid;
    snapshot->x_cm = publication->x_cm;
    snapshot->y_cm = publication->y_cm;
    snapshot->heading_deg = publication->heading_deg;
    pthread_mutex_unlock(&publication->mutex);
}

static bool collision_position_to_mm(double centimeters, int32_t *millimeters)
{
    double value = centimeters * 10.0;

    /* ETSI LocalCoordinates 계약 범위: -1,000,000 .. +1,000,000 mm. */
    if (!isfinite(value) || value < -1000000.0 || value > 1000000.0)
    {
        return false;
    }
    *millimeters = (int32_t)llround(value);
    return true;
}

static uint16_t collision_heading_value(
    bool heading_valid,
    double heading_deg
)
{
    double normalized;
    long value;

    /* ETSI heading은 0.1도 단위이고 3601은 사용할 수 없음을 뜻한다. */
    if (!heading_valid || !isfinite(heading_deg))
    {
        return 3601U;
    }
    normalized = fmod(heading_deg, 360.0);
    if (normalized < 0.0)
    {
        normalized += 360.0;
    }
    value = lround(normalized * 10.0);
    if (value >= 3600L)
    {
        value = 0L;
    }
    return (uint16_t)value;
}

static bool collision_fill_payload(
    const collision_config_t *config,
    const position_publication_t *position,
    int64_t detection_time,
    accident_event_payload_t *payload
)
{
    bool position_in_range;
    int32_t x_mm = 0;
    int32_t y_mm = 0;

    if (config == NULL || position == NULL || payload == NULL
        || detection_time <= 0)
    {
        return false;
    }

    position_in_range = position->valid
        && collision_position_to_mm(position->x_cm, &x_mm)
        && collision_position_to_mm(position->y_cm, &y_mm);
    if (config->require_position && !position_in_range)
    {
        return false;
    }

    memset(payload, 0, sizeof(*payload));
    /*
     * 이 프로세스는 센서 사실만 제공한다. station/action ID와 referenceTime은
     * 네트워크 역할을 소유한 broadcast_sender가 최종 메시지를 만들며 채운다.
     */
    payload->station_id = 0U;
    payload->sequence_number = 0U;
    payload->detection_time = detection_time;
    payload->reference_time = 0;

    payload->latitude = 900000001;
    payload->longitude = 1800000001;
    payload->speed_value = 16383U;
    payload->heading_value = collision_heading_value(
        position->heading_valid,
        position->heading_deg
    );
    payload->delta_latitude = 131072;
    payload->delta_longitude = 131072;
    payload->delta_altitude = 12800;
    payload->local_x_mm = position_in_range ? x_mm : 0;
    payload->local_y_mm = position_in_range ? y_mm : 0;
    return true;
}

static void collision_ipc_wait_retry(collision_ipc_worker_t *worker)
{
    struct timespec deadline;

    (void)clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_nsec += 500000000L;
    if (deadline.tv_nsec >= 1000000000L)
    {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000L;
    }
    pthread_mutex_lock(&worker->mutex);
    if (!worker->stop_requested)
    {
        (void)pthread_cond_timedwait(
            &worker->condition,
            &worker->mutex,
            &deadline
        );
    }
    pthread_mutex_unlock(&worker->mutex);
}

static void *collision_ipc_thread_main(void *argument)
{
    collision_ipc_worker_t *worker = argument;
    int fd = -1;

    for (;;)
    {
        collision_ipc_item_t item;
        uint32_t sequence;
        ipc_header_t header;

        pthread_mutex_lock(&worker->mutex);
        while (worker->count == 0U && !worker->stop_requested)
        {
            pthread_cond_wait(&worker->condition, &worker->mutex);
        }
        if (worker->stop_requested)
        {
            pthread_mutex_unlock(&worker->mutex);
            break;
        }
        item = worker->items[worker->head];
        sequence = worker->next_sequence;
        pthread_mutex_unlock(&worker->mutex);

        /* 연결 실패 시 queue head를 제거하지 않아 같은 사고를 재연결 후 보낸다. */
        if (fd < 0)
        {
            struct timeval send_timeout = {0, 200000};

            fd = ipc_client_connect(SENSOR_P2P_SOCKET_PATH);
            if (fd < 0)
            {
                collision_ipc_wait_retry(worker);
                continue;
            }
            (void)setsockopt(
                fd,
                SOL_SOCKET,
                SO_SNDTIMEO,
                &send_timeout,
                sizeof(send_timeout)
            );
        }

        memset(&header, 0, sizeof(header));
        header.type = IPC_MSG_ACCIDENT_EVENT;
        header.flags = IPC_FLAG_NONE;
        header.sequence = sequence;
        header.timestamp = item.monotonic_ns;

        if (ipc_send(
                fd,
                &header,
                &item.payload,
                sizeof(item.payload)
            ) != IPC_SUCCESS)
        {
            ipc_close(fd);
            fd = -1;
            collision_ipc_wait_retry(worker);
            continue;
        }

        pthread_mutex_lock(&worker->mutex);
        worker->head = (worker->head + 1U) % SENSOR_IPC_QUEUE_CAPACITY;
        worker->count--;
        worker->sent++;
        worker->next_sequence = sequence >= UINT16_MAX ? 1U : sequence + 1U;
        pthread_mutex_unlock(&worker->mutex);
    }

    if (fd >= 0)
    {
        ipc_close(fd);
    }
    return NULL;
}

static int collision_ipc_worker_start(collision_ipc_worker_t *worker)
{
    int result;

    memset(worker, 0, sizeof(*worker));
    if (pthread_mutex_init(&worker->mutex, NULL) != 0)
    {
        return -1;
    }
    if (pthread_cond_init(&worker->condition, NULL) != 0)
    {
        pthread_mutex_destroy(&worker->mutex);
        return -1;
    }
    worker->initialized = true;
    worker->next_sequence = 1U;
    result = pthread_create(
        &worker->thread,
        NULL,
        collision_ipc_thread_main,
        worker
    );
    if (result != 0)
    {
        pthread_cond_destroy(&worker->condition);
        pthread_mutex_destroy(&worker->mutex);
        worker->initialized = false;
        return -1;
    }
    worker->thread_created = true;
    return 0;
}

static bool collision_ipc_enqueue(
    collision_ipc_worker_t *worker,
    const collision_ipc_item_t *item
)
{
    size_t tail;
    bool queued = false;

    pthread_mutex_lock(&worker->mutex);
    /* 센서 Thread는 기다리지 않는다. 포화 시 drop을 명시적으로 계수한다. */
    if (!worker->stop_requested && worker->count < SENSOR_IPC_QUEUE_CAPACITY)
    {
        tail = (worker->head + worker->count) % SENSOR_IPC_QUEUE_CAPACITY;
        worker->items[tail] = *item;
        worker->count++;
        queued = true;
        pthread_cond_signal(&worker->condition);
    }
    else
    {
        worker->dropped++;
    }
    pthread_mutex_unlock(&worker->mutex);
    return queued;
}

static void collision_ipc_worker_stop(collision_ipc_worker_t *worker)
{
    if (!worker->initialized)
    {
        return;
    }
    pthread_mutex_lock(&worker->mutex);
    worker->stop_requested = true;
    pthread_cond_broadcast(&worker->condition);
    pthread_mutex_unlock(&worker->mutex);
    if (worker->thread_created)
    {
        pthread_join(worker->thread, NULL);
    }
    pthread_cond_destroy(&worker->condition);
    pthread_mutex_destroy(&worker->mutex);
    worker->initialized = false;
    worker->thread_created = false;
}

/* ========================================================================== */
/* BNO event ring 소비 사고 판정 Thread                                        */
/* ========================================================================== */

/*
 * cursor는 Thread 시작 이후 event부터 읽어 startup/calibration 전 sample을
 * 사고 지표에 섞지 않는다. startup_ready loss나 ring overwrite는 sample
 * 연속성이 깨진 것이므로 detector를 재초기화한다. 위치 publication은 오직
 * rising edge가 난 순간에만 복사하며 IPC 송신 자체는 전용 Thread에 넘긴다.
 * cursor.dropped가 늘었다면 중간 sample이 사라진 것이므로 직전 sample과의
 * jerk/delta-V 계산을 이어가지 않고 detector 상태를 깨끗하게 다시 시작한다.
 */

static bool collision_worker_should_stop(collision_worker_t *worker)
{
    bool stop;

    pthread_mutex_lock(&worker->mutex);
    stop = worker->stop_requested;
    pthread_mutex_unlock(&worker->mutex);
    return stop;
}

static void collision_worker_set_armed(
    collision_worker_t *worker,
    bool armed
)
{
    if (worker == NULL || !worker->initialized)
    {
        return;
    }
    /* 위치 측정이 안정된 구간에서만 사고 판정을 허용하고 보정/stale에는 끈다. */
    pthread_mutex_lock(&worker->mutex);
    worker->arm_requested = armed;
    collision_detector_set_armed(
        &worker->detector,
        armed && worker->startup_ready_seen
    );
    pthread_mutex_unlock(&worker->mutex);
}

static bool collision_worker_apply_startup_locked(
    collision_worker_t *worker,
    bool startup_ready
)
{
    /* SH-2 reset 동안은 무조건 disarm하고 새 startup 경계에서만 요청 상태로 복구한다. */
    if (!startup_ready)
    {
        collision_detector_set_armed(&worker->detector, false);
        worker->startup_ready_seen = false;
        return false;
    }
    if (!worker->startup_ready_seen)
    {
        worker->startup_ready_seen = true;
        collision_detector_set_armed(
            &worker->detector,
            worker->arm_requested
        );
    }
    return worker->detector.armed;
}

static void *collision_worker_thread_main(void *argument)
{
    collision_worker_t *worker = argument;
    bno086_event_cursor_t cursor;
    uint64_t previous_dropped = 0U;

    memset(&cursor, 0, sizeof(cursor));
    pthread_mutex_lock(&worker->mutex);
    worker->ready = bno086_event_cursor_init(
            &worker->imu_worker->imu,
            &cursor,
            false
        ) > 0;
    worker->start_finished = true;
    pthread_cond_broadcast(&worker->start_condition);
    pthread_mutex_unlock(&worker->mutex);
    if (!worker->ready)
    {
        return NULL;
    }

    while (!collision_worker_should_stop(worker))
    {
        bno086_event_t event;
        collision_observation_t observation;
        double detection_evidence[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
        bno086_snapshot_t snapshot;
        int next_result;
        bool detected;

        next_result = bno086_event_next(
                &worker->imu_worker->imu,
                &cursor,
                &event,
                100
            );
        memset(&snapshot, 0, sizeof(snapshot));
        (void)bno086_get_snapshot(&worker->imu_worker->imu, &snapshot);

        pthread_mutex_lock(&worker->mutex);
        if (!collision_worker_apply_startup_locked(
                worker,
                snapshot.startup_ready
            )
            || next_result <= 0)
        {
            pthread_mutex_unlock(&worker->mutex);
            continue;
        }
        if (cursor.dropped != previous_dropped)
        {
            bool was_armed = worker->detector.armed;
            collision_detector_set_armed(&worker->detector, was_armed);
            previous_dropped = cursor.dropped;
        }
        detected = collision_detector_process(
            &worker->detector,
            &event,
            &observation
        );
        if (detected)
        {
            memcpy(
                detection_evidence,
                worker->detector.evidence_values,
                sizeof(detection_evidence)
            );
        }
        pthread_mutex_unlock(&worker->mutex);

        if (detected)
        {
            position_publication_t position_snapshot;
            collision_ipc_item_t item;

            memset(&position_snapshot, 0, sizeof(position_snapshot));
            position_publication_snapshot(
                worker->position,
                &position_snapshot
            );
            if (collision_fill_payload(
                    &worker->detector.config,
                    &position_snapshot,
                    timestamp_its_ms(),
                    &item.payload
                ))
            {
                item.monotonic_ns = event.monotonic_ns;
                if (!collision_ipc_enqueue(worker->ipc_worker, &item))
                {
                    fprintf(stderr, "\n[!] 사고 IPC queue가 가득 차 이벤트를 폐기했습니다.\n");
                }
                else
                {
                    fprintf(
                        stderr,
                        "\n[!] 충돌 후보를 P2P IPC queue에 넣었습니다. "
                        "evidence rawA=%.3f linA=%.3f rawJ=%.3f "
                        "linJ=%.3f dV=%.3f\n",
                        detection_evidence[0],
                        detection_evidence[1],
                        detection_evidence[2],
                        detection_evidence[3],
                        detection_evidence[4]
                    );
                }
            }
            else
            {
                fprintf(stderr, "\n[!] 충돌 후보가 발생했으나 유효 위치가 없어 전송하지 않았습니다.\n");
            }
        }
    }
    return NULL;
}

static int collision_worker_start(
    collision_worker_t *worker,
    imu_worker_t *imu_worker,
    position_publication_t *position,
    collision_ipc_worker_t *ipc_worker
)
{
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
    worker->initialized = true;
    worker->imu_worker = imu_worker;
    worker->position = position;
    worker->ipc_worker = ipc_worker;
    collision_detector_initialize(
        &worker->detector,
        &collision_config,
        imu_worker->raw_accel_interval_us,
        imu_worker->linear_accel_interval_us
    );
    result = pthread_create(
        &worker->thread,
        NULL,
        collision_worker_thread_main,
        worker
    );
    if (result != 0)
    {
        pthread_cond_destroy(&worker->start_condition);
        pthread_mutex_destroy(&worker->mutex);
        worker->initialized = false;
        return -1;
    }
    worker->thread_created = true;

    pthread_mutex_lock(&worker->mutex);
    while (!worker->start_finished)
    {
        pthread_cond_wait(&worker->start_condition, &worker->mutex);
    }
    result = worker->ready ? 0 : -1;
    pthread_mutex_unlock(&worker->mutex);
    if (result != 0)
    {
        pthread_join(worker->thread, NULL);
        pthread_cond_destroy(&worker->start_condition);
        pthread_mutex_destroy(&worker->mutex);
        worker->initialized = false;
        worker->thread_created = false;
        return -1;
    }
    return 0;
}

static void collision_worker_stop(collision_worker_t *worker)
{
    if (!worker->initialized)
    {
        return;
    }
    collision_worker_set_armed(worker, false);
    pthread_mutex_lock(&worker->mutex);
    worker->stop_requested = true;
    pthread_mutex_unlock(&worker->mutex);
    if (worker->thread_created)
    {
        pthread_join(worker->thread, NULL);
    }
    pthread_cond_destroy(&worker->start_condition);
    pthread_mutex_destroy(&worker->mutex);
    worker->initialized = false;
    worker->thread_created = false;
}

/* ========================================================================== */
/* config.json의 두 숫자만 읽고 쓰는 최소 설정 처리                           */
/* ========================================================================== */

static int parse_json_number(
    const char *document,
    const char *key,
    double *value
)
{
    const char *position;
    char *end;
    double parsed;

    position = strstr(document, key);
    if (position == NULL)
    {
        return -1;
    }
    position = strchr(position, ':');
    if (position == NULL)
    {
        return -1;
    }

    errno = 0;
    parsed = strtod(position + 1, &end);
    if (end == position + 1 || errno != 0 || !isfinite(parsed))
    {
        return -1;
    }

    *value = parsed;
    return 0;
}

static void load_sensor_config(sensor_config_t *config)
{
    FILE *file;
    char document[1024];
    size_t bytes;
    double value;

    config->height_cm = SENSOR_DEFAULT_HEIGHT_CM;
    config->scaler = SENSOR_DEFAULT_SCALER;

    file = fopen(sensor_config_path, "rb");
    if (file == NULL)
    {
        return;
    }

    bytes = fread(document, 1U, sizeof(document) - 1U, file);
    fclose(file);
    document[bytes] = '\0';

    if (parse_json_number(document, "\"height_cm\"", &value) == 0
        && value > 0.0)
    {
        config->height_cm = value;
    }
    if (parse_json_number(document, "\"scaler\"", &value) == 0
        && value > 0.0)
    {
        config->scaler = value;
    }
}

static int save_sensor_config(const sensor_config_t *config)
{
    FILE *file = fopen(sensor_config_path, "wb");
    int write_failed;
    int close_failed;

    if (file == NULL)
    {
        fprintf(stderr, "[!] 설정 저장 실패: %s\n", strerror(errno));
        return -1;
    }

    write_failed = fprintf(
            file,
            "{\n    \"height_cm\": %.12g,\n    \"scaler\": %.12g\n}\n",
            config->height_cm,
            config->scaler
        ) < 0;
    close_failed = fclose(file) != 0;

    if (write_failed || close_failed)
    {
        fprintf(stderr, "[!] 설정 저장 실패: %s\n", strerror(errno));
        return -1;
    }

    printf(
        "[+] 보정 설정이 저장되었습니다: 높이=%.3f cm, Scaler=%.3f\n",
        config->height_cm,
        config->scaler
    );
    return 0;
}

/* ========================================================================== */
/* BNO snapshot을 순수 융합 자료형으로 변환                                    */
/* ========================================================================== */

static void convert_imu_snapshot(
    const bno086_snapshot_t *source,
    position_imu_sample_t *destination
)
{
    memset(destination, 0, sizeof(*destination));

    destination->startup_ready = source->startup_ready;
    destination->orientation_sequence = source->orientation_sequence;
    destination->gyro_sequence = source->gyro_sequence;
    destination->mag_sequence = source->mag_sequence;
    destination->orientation_time_s =
        (double)source->last_orientation_update_ns / 1000000000.0;
    destination->gyro_time_s =
        (double)source->last_gyro_update_ns / 1000000000.0;
    destination->mag_time_s =
        (double)source->last_mag_update_ns / 1000000000.0;

    destination->quaternion.real = source->quat_real;
    destination->quaternion.i = source->quat_i;
    destination->quaternion.j = source->quat_j;
    destination->quaternion.k = source->quat_k;

    destination->gyro_x_rad_s = source->gyro_x_rad_s;
    destination->gyro_y_rad_s = source->gyro_y_rad_s;
    destination->gyro_z_rad_s = source->gyro_z_rad_s;
    destination->rotation_accuracy = source->rotation_accuracy;
    destination->mag_accuracy = source->mag_accuracy;
}

static int get_position_imu_sample(
    const imu_worker_t *worker,
    position_imu_sample_t *sample
)
{
    bno086_snapshot_t snapshot;

    if (bno086_get_snapshot(&worker->imu, &snapshot) <= 0)
    {
        return -1;
    }

    convert_imu_snapshot(&snapshot, sample);
    return 0;
}

static int measurement_read_worker_imu(
    void *context,
    position_imu_sample_t *sample
)
{
    return get_position_imu_sample((const imu_worker_t *)context, sample);
}

static int measurement_flush_paa(
    void *context,
    int16_t *delta_x_ticks,
    int16_t *delta_y_ticks
)
{
    return paa5100je_read_motion_count(
        (paa5100je_t *)context,
        delta_x_ticks,
        delta_y_ticks
    );
}

static double measurement_monotonic_time(void *context)
{
    (void)context;
    return monotonic_time_seconds();
}

/* ========================================================================== */
/* BNO086 단일 소유 I/O Thread                                                 */
/* ========================================================================== */

/*
 * BNO API 호출을 이 Thread에 집중시켜 SHTP transaction 순서를 보존한다.
 * Main과 collision Thread는 driver의 mutex-protected snapshot/event cursor만
 * 읽는다. start_condition은 모든 feature의 FC와 실제 input report가 확인된
 * 이후에만 ready를 공개해 부분 초기화 상태로 측정을 시작하지 않게 한다.
 * 이 구조 덕분에 충돌 Thread가 accelerometer를 읽는 동안 위치 Thread가 BNO
 * register를 따로 읽어 packet 순서를 깨뜨리는 상황이 생기지 않는다.
 */

static bool imu_worker_stop_requested(imu_worker_t *worker)
{
    bool stop;

    pthread_mutex_lock(&worker->mutex);
    stop = worker->stop_requested;
    pthread_mutex_unlock(&worker->mutex);
    return stop;
}

static void imu_worker_publish_start(
    imu_worker_t *worker,
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

static int setup_one_feature(
    bno086_t *imu,
    const char *name,
    uint8_t sensor_id,
    uint32_t interval_us,
    uint32_t *applied_interval_us,
    unsigned int report_timeout_ms,
    char *error,
    size_t error_size
)
{
    if (bno086_enable_feature(imu, sensor_id, interval_us, 750U) <= 0)
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

    if (applied_interval_us != NULL
        && (bno086_get_feature_interval(
                imu,
                sensor_id,
                applied_interval_us
            ) <= 0
            || *applied_interval_us == 0U))
    {
        snprintf(error, error_size, "%s 실제 FC interval 없음", name);
        return -1;
    }

    return 0;
}

static void *imu_thread_main(void *argument)
{
    imu_worker_t *worker = argument;
    char error[BNO086_ERROR_TEXT_SIZE] = "";
    char summary[BNO086_ERROR_TEXT_SIZE] = "";

    if (bno086_begin(&worker->imu) <= 0)
    {
        bno086_status_summary(&worker->imu, summary, sizeof(summary));
        snprintf(error, sizeof(error), "BNO086 begin 실패: %.120s", summary);
        imu_worker_publish_start(worker, false, error);
        bno086_close(&worker->imu);
        return NULL;
    }

    /*
     * Python production과 동일하게 FD -> FE/FC -> 실제 Channel 3 report를
     * 센서별로 하나씩 확인한 뒤 다음 feature를 설정한다. RV/Gyro/Mag는 위치,
     * raw/linear acceleration은 충돌 지표에 쓰며 동적 보정 accuracy는 이후
     * 사용자 시작 gate에서 별도로 확인한다.
     */
    if (setup_one_feature(
            &worker->imu,
            "Rotation Vector",
            BNO086_SENSOR_ROTATION_VECTOR,
            10000U,
            NULL,
            750U,
            error,
            sizeof(error)
        ) != 0
        || setup_one_feature(
            &worker->imu,
            "Gyroscope",
            BNO086_SENSOR_GYROSCOPE_CALIBRATED,
            10000U,
            &worker->gyro_interval_us,
            750U,
            error,
            sizeof(error)
        ) != 0
        || setup_one_feature(
            &worker->imu,
            "Magnetometer",
            BNO086_SENSOR_MAGNETOMETER_CALIBRATED,
            SENSOR_MAG_INTERVAL_US,
            NULL,
            1000U,
            error,
            sizeof(error)
        ) != 0
        || setup_one_feature(
            &worker->imu,
            "Accelerometer",
            BNO086_SENSOR_ACCELEROMETER,
            SENSOR_ACCEL_INTERVAL_US,
            &worker->raw_accel_interval_us,
            1000U,
            error,
            sizeof(error)
        ) != 0
        || setup_one_feature(
            &worker->imu,
            "Linear Acceleration",
            BNO086_SENSOR_LINEAR_ACCELERATION,
            SENSOR_ACCEL_INTERVAL_US,
            &worker->linear_accel_interval_us,
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
        imu_worker_publish_start(worker, false, error);
        bno086_close(&worker->imu);
        return NULL;
    }

    imu_worker_publish_start(worker, true, NULL);

    while (!imu_worker_stop_requested(worker))
    {
        (void)bno086_update(&worker->imu);
        sleep_microseconds(SENSOR_BNO_UPDATE_INTERVAL_US);
    }

    bno086_close(&worker->imu);
    return NULL;
}

static int imu_worker_start(imu_worker_t *worker)
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

    result = pthread_create(&worker->thread, NULL, imu_thread_main, worker);
    if (result != 0)
    {
        fprintf(stderr, "[!] BNO086 Thread 생성 실패: %s\n", strerror(result));
        bno086_deinit(&worker->imu);
        pthread_cond_destroy(&worker->start_condition);
        pthread_mutex_destroy(&worker->mutex);
        return -1;
    }
    worker->thread_created = true;

    pthread_mutex_lock(&worker->mutex);
    while (!worker->start_finished)
    {
        pthread_cond_wait(&worker->start_condition, &worker->mutex);
    }
    result = worker->ready ? 0 : -1;
    pthread_mutex_unlock(&worker->mutex);

    return result;
}

static void imu_worker_stop(imu_worker_t *worker)
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

/* ========================================================================== */
/* 융합 시작 전 report freshness와 accuracy gate                              */
/* ========================================================================== */

/*
 * 시작할 때는 RV/Gyro/Mag가 모두 fresh하고 RV/Mag accuracy가 2 이상이어야
 * reference를 잡는다. 운행 중 accuracy의 순간 하락은 진단만 남기며 flow를
 * 폐기하지 않는다. 반면 stale/startup loss는 시간 정렬 근거가 없어 적분과
 * 충돌 detector arm을 중단한다.
 * accuracy는 "보정 품질 등급", stale은 "새 report가 제시간에 오지 않음"이다.
 * 전자는 순간적으로 낮아져도 값이 즉시 사라진 것은 아니지만, 후자는 현재
 * 방향을 모른다는 뜻이므로 같은 조건으로 다루지 않는다.
 */

static bool wait_for_fusion_reports(
    const imu_worker_t *worker,
    unsigned int timeout_ms,
    position_imu_sample_t *sample
)
{
    uint64_t deadline =
        monotonic_time_ns() + (uint64_t)timeout_ms * 1000000ULL;

    while (!process_stop_requested && monotonic_time_ns() < deadline)
    {
        double now_s = monotonic_time_seconds();

        if (get_position_imu_sample(worker, sample) != 0
            || !sample->startup_ready)
        {
            return false;
        }
        if (position_fusion_reports_are_fresh(sample, now_s))
        {
            return true;
        }
        sleep_microseconds(10000U);
    }

    return false;
}

static bool wait_for_calibration_ready(
    const imu_worker_t *worker,
    position_imu_sample_t *sample
)
{
    uint64_t next_status_ns = 0U;

    puts("[*] BNO086 보정 대기: 센서를 3초간 정지시킨 뒤 Roll/Pitch/Yaw 축으로");
    puts("    약 180도 왕복 회전하십시오. RV/Mag가 모두 2 이상이어야 융합을 시작합니다.");
    puts("    사고차량 모드는 IMU 없이 시작하지 않습니다. 중단하려면 'q' 후 Enter를 누르세요.");

    while (!process_stop_requested)
    {
        uint64_t now_ns = monotonic_time_ns();
        double now_s = (double)now_ns / 1000000000.0;

        if (get_position_imu_sample(worker, sample) != 0
            || !sample->startup_ready)
        {
            puts("\n[!] 보정 중 BNO086 startup_ready가 해제되었습니다.");
            return false;
        }

        if (position_fusion_reports_are_fresh(sample, now_s)
            && position_fusion_accuracy_is_ready(sample))
        {
            printf(
                "\n[+] BNO086 보정 준비 완료: RV/Mag=%u/%u\n",
                sample->rotation_accuracy,
                sample->mag_accuracy
            );
            return true;
        }

        if (now_ns >= next_status_ns)
        {
            printf(
                "\r    RV/Mag=%u/%u reports=%s",
                sample->rotation_accuracy,
                sample->mag_accuracy,
                position_fusion_reports_are_fresh(sample, now_s)
                    ? "fresh"
                    : "stale"
            );
            fflush(stdout);
            next_status_ns = now_ns + 250000000ULL;
        }

        if (stdin_line_ready(20))
        {
            char command[32];
            if (fgets(command, sizeof(command), stdin) == NULL)
            {
                return false;
            }
            trim_line(command);
            if (strcmp(command, "q") == 0 || strcmp(command, "Q") == 0)
            {
                puts("\n[*] 사용자가 IMU 보정 대기를 중단했습니다.");
                return false;
            }
        }
        sleep_microseconds(5000U);
    }

    return false;
}

static bool wait_for_new_orientation(
    const imu_worker_t *worker,
    uint64_t previous_sequence,
    unsigned int timeout_ms,
    bool require_accuracy,
    position_imu_sample_t *sample
)
{
    uint64_t deadline =
        monotonic_time_ns() + (uint64_t)timeout_ms * 1000000ULL;

    while (!process_stop_requested && monotonic_time_ns() < deadline)
    {
        double now_s = monotonic_time_seconds();

        if (get_position_imu_sample(worker, sample) != 0
            || !sample->startup_ready)
        {
            return false;
        }

        if (sample->orientation_sequence > previous_sequence
            && position_fusion_reports_are_fresh(sample, now_s)
            && (!require_accuracy
                || position_fusion_accuracy_is_ready(sample)))
        {
            return true;
        }

        sleep_microseconds(2000U);
    }

    return false;
}

/*
 * 사용자가 Enter를 누른 시점보다 뒤에 생성된 RV를 한 번 더 기다린다.
 * 화면 안내를 읽고 차량을 움직이는 동안 BNO I/O Thread는 계속 report를
 * 소비하므로, Enter 직전 snapshot을 그대로 쓰면 실제 버튼 입력보다 앞선
 * 자세가 선택될 수 있다. sequence 경계를 잡고 그 다음 fresh report만
 * 반환하여 시작 기준과 90도 점검값 모두 같은 규칙으로 측정한다.
 */
static bool capture_orientation_after_enter(
    const imu_worker_t *worker,
    const char *prompt,
    bool require_accuracy,
    position_imu_sample_t *sample
)
{
    char line[32];
    position_imu_sample_t before;
    bool captured;

    if (read_line_prompt(prompt, line, sizeof(line)) != 0
        || get_position_imu_sample(worker, &before) != 0)
    {
        return false;
    }

    captured = wait_for_new_orientation(
        worker,
        before.orientation_sequence,
        SENSOR_POST_ENTER_TIMEOUT_MS,
        require_accuracy,
        sample
    );
    if (!captured)
    {
        double now_s = monotonic_time_seconds();
        bool snapshot_available =
            get_position_imu_sample(worker, sample) == 0;

        if (snapshot_available)
        {
            printf(
                "\n    [RV 획득 진단] startup=%s, sequence=%llu -> %llu, "
                "reports=%s, RV/Mag=%u/%u\n",
                sample->startup_ready ? "ready" : "lost",
                (unsigned long long)before.orientation_sequence,
                (unsigned long long)sample->orientation_sequence,
                position_fusion_reports_are_fresh(sample, now_s)
                    ? "fresh"
                    : "stale",
                sample->rotation_accuracy,
                sample->mag_accuracy
            );
            if (require_accuracy
                && sample->orientation_sequence > before.orientation_sequence
                && position_fusion_reports_are_fresh(sample, now_s)
                && !position_fusion_accuracy_is_ready(sample))
            {
                puts("    새 RV는 수신됐지만 RV/Mag accuracy 2 이상 조건을 만족하지 못했습니다.");
            }
        }
        else
        {
            puts("\n    [RV 획득 진단] BNO snapshot을 읽지 못했습니다.");
        }
    }

    return captured;
}

/*
 * RV/Mag accuracy만으로는 BNO quaternion의 축 방향과 프로그램의 곱셈
 * 순서가 차량 장착 상태에서 올바른지 확인할 수 없다. 특히 센서가 고정된
 * roll/pitch를 가진 채 장착되었을 때도, 차량 전체를 testbed 수직축으로
 * CCW 90도 돌리면 q_now * inverse(q_start)의 yaw는 +90도가 되어야 한다.
 *
 * 이 점검은 사람이 차량을 돌리므로 정확히 90.000도를 강제하지 않고
 * +/-15도 범위를 허용한다. 반대 부호는 범위에 들어오지 않으므로 장착축이나
 * heading 부호가 뒤집힌 경우도 시작 전에 발견한다. 90도 자세에서 Mag
 * accuracy가 순간적으로 1로 내려가더라도 fresh RV의 측정 heading과 품질값은
 * 표시한다. 그래야 accuracy gate 때문에 정작 확인하려던 장착축 결과가 가려지지
 * 않는다. RV/Mag 2 이상은 이 함수 직전의 보정 gate에서 이미 한 번 확인했다.
 * 그 이후 순간 하락은 운행 중 정책과 동일하게 진단만 남기고 fresh RV 획득을
 * 막지 않는다. 점검 후에는 반드시
 * 출발 방향으로 돌아오게 하고, 돌아온 뒤의 fresh quaternion을 최종 reference로
 * 넘긴다. 따라서 손으로 90도 시험한 회전이나 그동안 쌓인 heading은 실제
 * 위치 적분의 첫 sample에 포함되지 않는다. PAA latch는 기존 시작 flush가
 * 별도로 비우므로 이 함수에서는 SPI 장치를 읽지 않는다.
 */
static bool run_startup_heading_check(
    const imu_worker_t *worker,
    const position_imu_sample_t *start_reference,
    position_imu_sample_t *final_reference
)
{
    relative_heading_tracker_t tracker;
    position_imu_sample_t turned;
    position_imu_sample_t returned;
    double measured_heading_deg;
    double turn_error_deg;
    double closure_heading_deg;

    relative_heading_reset(&tracker, start_reference);

    puts("[*] Heading 실기기 점검을 시작합니다.");
    puts("    차량 위에서 내려다봤을 때 차량 전체를 반시계(CCW) 방향으로 약 90도 돌리세요.");
    puts("    사람이 돌리는 시험이므로 정확히 90도일 필요는 없으며 75~105도를 정상 범위로 봅니다.");
    if (!capture_orientation_after_enter(
            worker,
            "    회전이 끝나면 Enter를 누르세요: ",
            false,
            &turned
        ))
    {
        puts("[!] 90도 회전 후 fresh RV를 확인하지 못했습니다.");
        return false;
    }

    measured_heading_deg = relative_heading_raw_deg(&tracker, &turned);
    turn_error_deg = position_angle_delta_deg(
        measured_heading_deg,
        SENSOR_HEADING_CHECK_TARGET_DEG
    );
    printf(
        "    기대 heading=+%.1f deg, 측정=%+.2f deg, 오차=%+.2f deg "
        "(RV/Mag %u/%u)\n",
        SENSOR_HEADING_CHECK_TARGET_DEG,
        measured_heading_deg,
        turn_error_deg,
        turned.rotation_accuracy,
        turned.mag_accuracy
    );
    if (!position_fusion_accuracy_is_ready(&turned))
    {
        puts("[!] 90도 자세에서 accuracy가 2 미만입니다. 각도는 표시하지만 보정 품질은 낮은 상태입니다.");
        puts("    보정 gate는 이미 통과했으므로 순간 하락은 기록하되 Heading 점검을 계속합니다.");
    }
    if (fabs(turn_error_deg) > SENSOR_HEADING_CHECK_TOLERANCE_DEG)
    {
        printf(
            "[!] Heading 점검 실패: 허용 범위는 +%.1f +/- %.1f deg입니다.\n",
            SENSOR_HEADING_CHECK_TARGET_DEG,
            SENSOR_HEADING_CHECK_TOLERANCE_DEG
        );
        puts("    차량 회전 방향, BNO 장착축, quaternion 방향/곱셈 순서를 확인하세요.");
        return false;
    }

    if (!capture_orientation_after_enter(
            worker,
            "[*] 차량을 처음 출발 방향으로 되돌린 뒤 Enter를 누르세요: ",
            false,
            &returned
        ))
    {
        puts("[!] 출발 방향 복귀 후 fresh RV를 확인하지 못했습니다.");
        return false;
    }

    closure_heading_deg = relative_heading_raw_deg(&tracker, &returned);
    printf(
        "    복귀 heading=%+.2f deg (0도 기준 오차=%+.2f deg)\n",
        closure_heading_deg,
        closure_heading_deg
    );
    if (fabs(closure_heading_deg) > SENSOR_HEADING_CHECK_TOLERANCE_DEG)
    {
        printf(
            "[!] 출발 방향 복귀 확인 실패: |heading|이 %.1f deg보다 큽니다.\n",
            SENSOR_HEADING_CHECK_TOLERANCE_DEG
        );
        puts("    차량을 처음 표시한 방향에 맞춘 뒤 프로그램을 다시 시작하세요.");
        return false;
    }

    *final_reference = returned;
    printf(
        "[+] Heading 점검 PASS: CCW 90도와 원점 복귀를 확인했고, "
        "현재 자세를 새 heading 0도로 사용합니다. (RV/Mag %u/%u)\n",
        returned.rotation_accuracy,
        returned.mag_accuracy
    );
    return true;
}

/* ========================================================================== */
/* PAA5100JE 거리 scaler 보정                                                   */
/* ========================================================================== */

static int run_distance_calibration(
    paa5100je_t *flow,
    sensor_config_t *config
)
{
    char input[64];
    double target_distance_cm = 10.0;
    int64_t accumulated_x = 0;
    int64_t accumulated_y = 0;

    puts("\n----------------------------------------------------------------");
    puts(" [간편 캘리브레이션 모드]");
    printf(" - 현재 센서 높이 z: %.3f cm (PAA5100JE 권장: 1.5~3.5 cm)\n",
           config->height_cm);

    if (read_line_prompt(
            " - 테스트로 이동시킬 실제 거리(cm) 입력 [기본값: 10]: ",
            input,
            sizeof(input)
        ) == 0
        && input[0] != '\0')
    {
        char *end;
        double parsed = strtod(input, &end);
        if (end != input && parsed > 0.0 && isfinite(parsed))
        {
            target_distance_cm = parsed;
        }
    }

    printf("[*] 센서를 수평으로 유지하며 정확히 %.3f cm 직선 이동합니다.\n",
           target_distance_cm);
    if (read_line_prompt(">> 시작 위치에서 Enter: ", input, sizeof(input)) != 0)
    {
        return -1;
    }

    if (paa5100je_read_motion_count(
            flow,
            &(int16_t){0},
            &(int16_t){0}
        ) != 0)
    {
        fprintf(
            stderr,
            "[!] 거리 보정 시작 PAA5100JE flush 실패: %s\n",
            strerror(errno)
        );
        return -1;
    }
    printf(">> %.3f cm 이동한 뒤 Enter를 누르세요.\n", target_distance_cm);

    while (!process_stop_requested)
    {
        int16_t dx;
        int16_t dy;

        if (paa5100je_read_motion_count(flow, &dx, &dy) != 0)
        {
            fprintf(stderr, "\n[!] PAA5100JE burst 실패: %s\n", strerror(errno));
            return -1;
        }
        accumulated_x += dx;
        accumulated_y += dy;
        printf(
            "\r   누적 tick: X=%7lld Y=%7lld | SQUAL=%3u",
            (long long)accumulated_x,
            (long long)accumulated_y,
            flow->last_squal
        );
        fflush(stdout);

        if (stdin_line_ready(50))
        {
            if (fgets(input, sizeof(input), stdin) == NULL)
            {
                return -1;
            }
            break;
        }
    }

    if (process_stop_requested)
    {
        puts("\n[!] 거리 보정이 중단되어 scaler를 저장하지 않습니다.");
        return -1;
    }

    {
        double total_ticks = hypot(
            (double)accumulated_x,
            (double)accumulated_y
        );

        printf("\n   측정 완료: %.3f ticks, SQUAL=%u\n",
               total_ticks, flow->last_squal);
        if (total_ticks > 0.0)
        {
            config->scaler =
                total_ticks * config->height_cm / target_distance_cm;
            (void)paa5100je_set_scaler(flow, config->scaler);
            (void)save_sensor_config(config);
            printf("[+] 새 Scaler: %.6f\n", config->scaler);
        }
        else
        {
            puts("[!] tick이 없어 기존 Scaler를 유지합니다.");
        }
    }

    puts("----------------------------------------------------------------\n");
    return 0;
}

/* ========================================================================== */
/* 실시간 위치 측정                                                            */
/* ========================================================================== */

/*
 * Main loop의 한 iteration은 PAA burst read -> 같은 시각의 BNO snapshot 복사
 * -> 순수 fusion 함수 호출 -> testbed 좌표 publication 순이다. 이 순서를 한
 * production 경로로 유지해 화면 출력, 충돌 payload와 회귀시험이 서로 다른
 * 위치 계산을 사용하지 않게 한다.
 */

static const char *position_result_text(position_sample_result_t result)
{
    switch (result)
    {
    case POSITION_SAMPLE_LOW_SQUAL:
        return "LOW-SQUAL";
    case POSITION_SAMPLE_IMU_STALE:
        return "IMU-STALE";
    case POSITION_SAMPLE_CALIBRATION_DISCARDED:
        return "CAL";
    default:
        return "OK";
    }
}

/*
 * run_measurement()가 읽은 한 motion burst를 production 융합 함수로
 * 전달하는 유일한 경로다. 회귀시험도 이 함수를 호출하므로 명령 경계와
 * 첫 정상 sample의 결합을 실제 실행 경로와 같은 순서로 확인한다.
 */
static void measurement_process_motion(
    position_fusion_t *fusion,
    const paa5100je_motion_t *motion,
    const position_imu_sample_t *imu_sample,
    double now_s,
    position_step_t *step
)
{
    position_fusion_process(
        fusion,
        motion->delta_x_ticks,
        motion->delta_y_ticks,
        motion->squal,
        imu_sample,
        now_s,
        step
    );
}

static bool measurement_collision_rearm_allowed(
    bool fusion_enabled,
    const position_fusion_t *fusion
)
{
    return !fusion_enabled || (fusion != NULL && !fusion->yaw_compensator.learning);
}

/*
 * 실제 'c' 명령의 시작/종료 orchestration이다.
 *
 * 종료 시 loop 앞부분에서 사용하던 snapshot을 재사용하지 않고 callback을
 * 통해 현재 snapshot을 다시 읽는다. fresh report가 아니면 calibration을
 * 끝내지 않으며, fresh heading으로 last_flow_heading을 재기준화한 뒤에만
 * PAA latch를 한 번 더 비운다. 보정 중에도 main loop가 burst를 계속 읽으므로
 * delta가 센서에 누적되지 않고, 종료 flush는 경계 직전에 latch된 마지막 값까지
 * 첫 정상 주행 sample로 넘어가지 않게 한다.
 * 따라서 큰 CW/CCW 보정을 끝낸 뒤 첫 직선 이동은 보정 전 heading과의 중간각이
 * 아니라 보정 종료 시점의 현재 heading을 기준으로 변환된다.
 */
static int measurement_toggle_yaw_calibration(
    position_fusion_t *fusion,
    bool fusion_enabled,
    const measurement_runtime_ops_t *ops,
    measurement_calibration_result_t *result
)
{
    position_imu_sample_t current_sample;
    double now_s;
    int16_t flush_x;
    int16_t flush_y;

    if (fusion == NULL || ops == NULL || result == NULL
        || ops->read_imu == NULL || ops->flush_flow == NULL
        || ops->get_time_s == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    memset(result, 0, sizeof(*result));
    result->action = MEASUREMENT_CALIBRATION_UNAVAILABLE;

    if (!fusion_enabled)
    {
        return 0;
    }

    memset(&current_sample, 0, sizeof(current_sample));
    if (ops->read_imu(ops->imu_context, &current_sample) != 0)
    {
        return 0;
    }

    now_s = ops->get_time_s(ops->time_context);
    if (!position_fusion_reports_are_fresh(&current_sample, now_s))
    {
        return 0;
    }

    if (!fusion->yaw_compensator.learning)
    {
        if (position_fusion_start_yaw_calibration(
                fusion,
                &current_sample,
                now_s
            ))
        {
            result->action = MEASUREMENT_CALIBRATION_STARTED;
            result->fresh_heading_used = true;
        }
        return 0;
    }

    result->fresh_heading_used = true;
    if (position_fusion_finish_yaw_calibration(
            fusion,
            &current_sample,
            now_s
        ))
    {
        result->action = MEASUREMENT_CALIBRATION_LOCKED;
    }
    else
    {
        result->action = MEASUREMENT_CALIBRATION_NEEDS_MORE;
    }

    /* 값은 의도적으로 사용하지 않는다. 이 burst 전체가 종료 경계 flush다. */
    if (ops->flush_flow(ops->flow_context, &flush_x, &flush_y) != 0)
    {
        return -1;
    }
    result->flow_flushed = true;

    return 0;
}

static void print_measurement_summary(
    const position_fusion_t *fusion,
    const position_imu_sample_t *sample,
    double now_s
)
{
    double testbed_x_cm;
    double testbed_y_cm;

    accident_relative_to_testbed_cm(
        fusion->x_cm,
        fusion->y_cm,
        &testbed_x_cm,
        &testbed_y_cm
    );
    printf("\n\n================================================================\n");
    puts(" [최종 정지 좌표 요약]");
    printf(" - 시작점 기준 변위  : ( %+0.2f cm,  %+0.2f cm )\n",
           fusion->x_cm, fusion->y_cm);
    printf(" - testbed 좌표      : ( %+0.2f cm,  %+0.2f cm )\n",
           testbed_x_cm, testbed_y_cm);
    printf(" - 원점으로부터 거리: %.2f cm\n", hypot(fusion->x_cm, fusion->y_cm));
    printf(" - 총 원시 tick      : X=%lld, Y=%lld\n",
           (long long)fusion->raw_ticks_x,
           (long long)fusion->raw_ticks_y);
    printf(" - 최종 표면 품질    : %u\n", fusion->last_squal);

    if (fusion->fusion_enabled && sample != NULL)
    {
        relative_heading_tracker_t tracker = fusion->heading_tracker;
        double heading = relative_heading_at_deg(&tracker, sample, now_s);

        printf(" - 최종 상대 heading : %+.1f deg (RV/Mag %u/%u)\n",
               heading, sample->rotation_accuracy, sample->mag_accuracy);
        printf(" - testbed heading   : %.1f deg\n",
               accident_testbed_heading_deg(heading));
        printf(" - 회전 offset 학습  : %u samples, (%+.3f, %+.3f) cm/rad, %s\n",
               fusion->yaw_compensator.sample_count,
               fusion->yaw_compensator.x_per_rad_cm,
               fusion->yaw_compensator.y_per_rad_cm,
               fusion->yaw_compensator.learning
                   ? "learning"
                   : fusion->yaw_compensator.calibrated ? "locked" : "disabled");
        printf(" - 운행 중 accuracy 저하: %u events, %.3f s (적분 계속)\n",
               fusion->accuracy_low_events,
               position_fusion_accuracy_low_total(fusion, now_s));
    }
    puts("================================================================");
}

static int run_measurement(
    paa5100je_t *flow,
    const sensor_config_t *config,
    imu_worker_t *worker,
    position_publication_t *position_publication,
    collision_worker_t *collision_worker
)
{
    position_imu_sample_t sample;
    position_imu_sample_t reference;
    position_fusion_t fusion;
    bool collision_arm_requested = false;
    bool stdout_is_terminal = isatty(STDOUT_FILENO) != 0;
    uint64_t next_status_ns = 0U;
    int16_t flush_x;
    int16_t flush_y;
    measurement_runtime_ops_t runtime_ops = {
        .read_imu = measurement_read_worker_imu,
        .imu_context = worker,
        .flush_flow = measurement_flush_paa,
        .flow_context = flow,
        .get_time_s = measurement_monotonic_time,
        .time_context = NULL
    };

    memset(&sample, 0, sizeof(sample));
    memset(&reference, 0, sizeof(reference));

    puts("[*] BNO086 Rotation Vector / Gyro / Magnetometer report 대기 중...");
    if (!wait_for_fusion_reports(
            worker,
            SENSOR_IMU_READY_TIMEOUT_MS,
            &sample
        ))
    {
        puts("[!] IMU report가 fresh하지 않아 사고차량 센서 프로세스를 시작하지 않습니다.");
        position_publication_invalidate(position_publication);
        return -1;
    }
    if (!wait_for_calibration_ready(worker, &sample))
    {
        puts("[!] RV/Mag accuracy 조건을 만족하지 않아 사고차량 센서 프로세스를 시작하지 않습니다.");
        position_publication_invalidate(position_publication);
        return -1;
    }

    if (!capture_orientation_after_enter(
            worker,
            "[*] 출발 자세에서 Enter를 누르세요. 이 자세로 90도 점검을 시작합니다: ",
            false,
            &sample
        ))
    {
        puts("[!] Enter 이후 fresh RV를 확인하지 못해 시작하지 않습니다.");
        position_publication_invalidate(position_publication);
        return -1;
    }
    if (!position_fusion_accuracy_is_ready(&sample))
    {
        printf(
            "[!] 보정 gate 통과 후 accuracy가 RV/Mag=%u/%u로 내려갔습니다. "
            "fresh quaternion으로 Heading 점검은 계속합니다.\n",
            sample.rotation_accuracy,
            sample.mag_accuracy
        );
    }
    if (!run_startup_heading_check(worker, &sample, &reference))
    {
        puts("[!] 실제 차량 회전과 BNO heading이 일치하지 않아 위치 융합을 시작하지 않습니다.");
        position_publication_invalidate(position_publication);
        return -1;
    }

    position_fusion_init(
        &fusion,
        config->height_cm,
        config->scaler,
        &reference
    );
    accident_position_publication_update(
        position_publication,
        fusion.x_cm,
        fusion.y_cm,
        true,
        0.0
    );

    puts("================================================================");
    printf(" [사고차량 PAA5100JE + BNO086 융합/충돌 감시] 높이 %.3f cm, scaler %.3f\n",
           config->height_cm,
           config->scaler);
    printf(" - SQUAL %u 미만은 위치에 적분하지 않습니다.\n", POSITION_MIN_SQUAL);
    puts(" - r: 위치/가능하면 heading 기준 reset");
    puts(" - c: 회전 offset 보정 시작/고정");
    puts(" - q 또는 Ctrl-C: 종료");
    puts("================================================================");

    /* 기준 설정 중 latch된 PAA delta를 한 번 비운다. */
    if (paa5100je_read_motion_count(flow, &flush_x, &flush_y) != 0)
    {
        fprintf(stderr, "[!] PAA5100JE 초기 flush 실패: %s\n", strerror(errno));
        collision_worker_set_armed(collision_worker, false);
        position_publication_invalidate(position_publication);
        return -1;
    }

    /* 시작 경계 이전 IMU event는 cursor가 비웠고 PAA latch도 비운 뒤 arm한다. */
    collision_worker_set_armed(collision_worker, true);
    collision_arm_requested = true;

    while (!process_stop_requested)
    {
        paa5100je_motion_t motion;
        position_step_t step;
        uint64_t now_ns;
        double now_s;
        double testbed_x_cm;
        double testbed_y_cm;
        double testbed_heading_deg;

        if (paa5100je_read_motion(flow, &motion) != 0)
        {
            fprintf(stderr, "\n[!] PAA5100JE burst 실패: %s\n", strerror(errno));
            collision_worker_set_armed(collision_worker, false);
            position_publication_invalidate(position_publication);
            return -1;
        }

        now_ns = monotonic_time_ns();
        now_s = (double)now_ns / 1000000000.0;
        if (get_position_imu_sample(worker, &sample) != 0)
        {
            memset(&sample, 0, sizeof(sample));
        }

        measurement_process_motion(
            &fusion,
            &motion,
            &sample,
            now_s,
            &step
        );
        accident_relative_to_testbed_cm(
            fusion.x_cm,
            fusion.y_cm,
            &testbed_x_cm,
            &testbed_y_cm
        );
        testbed_heading_deg = accident_testbed_heading_deg(step.heading_deg);
        /* 위치 heading이 stale이면 잘못된 좌표를 사고 payload에 싣지 않도록 무효화한다. */
        if (!step.imu_fresh)
        {
            position_publication_invalidate(position_publication);
            if (collision_arm_requested)
            {
                collision_worker_set_armed(collision_worker, false);
                collision_arm_requested = false;
            }
        }
        else
        {
            accident_position_publication_update(
                position_publication,
                fusion.x_cm,
                fusion.y_cm,
                true,
                step.heading_deg
            );
            if (!collision_arm_requested
                && measurement_collision_rearm_allowed(
                    true,
                    &fusion
                ))
            {
                collision_worker_set_armed(collision_worker, true);
                collision_arm_requested = true;
            }
        }

        /*
         * SPI를 읽는 5 ms loop마다 긴 상태줄을 출력하면 터미널 폭을 넘어간
         * 줄이 계속 누적되고, 느린 터미널에서는 stdout back-pressure가 sensor
         * read까지 지연시킬 수 있다. 센서 처리 주기는 그대로 두고 화면만 10 Hz로
         * 제한한다. ANSI clear-line은 실제 터미널에서만 사용하며, 파일/pipe로
         * 보낼 때는 1 Hz newline log로 남겨 하나의 거대한 줄이 되지 않게 한다.
         */
        if (now_ns >= next_status_ns)
        {
            const char *offset_state = fusion.yaw_compensator.learning
                ? "CAL"
                : fusion.yaw_compensator.calibrated ? "LOCK" : "OFF";

            if (stdout_is_terminal)
            {
                printf(
                    "\r\033[2K[POS] X:%+7.2f Y:%+7.2f Q:%3u H:%+6.2f "
                    "A:%u/%u O:%s/%02u %s",
                    testbed_x_cm,
                    testbed_y_cm,
                    motion.squal,
                    testbed_heading_deg,
                    sample.rotation_accuracy,
                    sample.mag_accuracy,
                    offset_state,
                    fusion.yaw_compensator.sample_count,
                    position_result_text(step.result)
                );
                next_status_ns = now_ns + SENSOR_STATUS_TTY_INTERVAL_NS;
            }
            else
            {
                printf(
                    "[POS] X=%+.2f Y=%+.2f cm Q=%u H=%+.2f "
                    "A=%u/%u O=%s/%u %s\n",
                    testbed_x_cm,
                    testbed_y_cm,
                    motion.squal,
                    testbed_heading_deg,
                    sample.rotation_accuracy,
                    sample.mag_accuracy,
                    offset_state,
                    fusion.yaw_compensator.sample_count,
                    position_result_text(step.result)
                );
                next_status_ns = now_ns + SENSOR_STATUS_LOG_INTERVAL_NS;
            }
            fflush(stdout);
        }

        /*
         * Python select timeout과 같은 약 5 ms cadence를 유지한다. 무제한
         * busy-loop는 같은 PAA delta를 과도하게 읽고 heading 경계를 바꿀 수 있다.
         */
        if (stdin_line_ready(5))
        {
            char command[32];

            if (fgets(command, sizeof(command), stdin) == NULL)
            {
                break;
            }
            trim_line(command);

            if (strcmp(command, "q") == 0)
            {
                break;
            }
            if (strcmp(command, "r") == 0)
            {
                bool reset_heading = position_fusion_reset_origin(
                    &fusion,
                    &sample,
                    now_s
                );
                printf("\n[!] 위치 reset%s\n",
                       !reset_heading
                           ? " (IMU stale: heading 기준 유지)"
                           : "");
                if (!reset_heading)
                {
                    position_publication_invalidate(position_publication);
                }
                else
                {
                    accident_position_publication_update(
                        position_publication,
                        fusion.x_cm,
                        fusion.y_cm,
                        true,
                        0.0
                    );
                }
            }
            else if (strcmp(command, "c") == 0)
            {
                measurement_calibration_result_t calibration_result;

                collision_worker_set_armed(collision_worker, false);
                collision_arm_requested = false;

                if (measurement_toggle_yaw_calibration(
                        &fusion,
                        true,
                        &runtime_ops,
                        &calibration_result
                    ) != 0)
                {
                    fprintf(
                        stderr,
                        "\n[!] 보정 종료 PAA5100JE flush 실패: %s\n",
                        strerror(errno)
                    );
                    position_publication_invalidate(position_publication);
                    return -1;
                }
                else if (calibration_result.action
                         == MEASUREMENT_CALIBRATION_STARTED)
                {
                    puts("\n[!] 보정 시작: 위치를 고정하고 CW/CCW 양방향으로 회전하세요.");
                    puts("    보정 중 PAA burst는 계속 읽지만 위치에는 적분하지 않습니다.");
                }
                else if (calibration_result.action
                         == MEASUREMENT_CALIBRATION_LOCKED)
                {
                    printf(
                        "\n[+] 보정 고정: samples=%u, CW/CCW=%u/%u, "
                        "offset=(%+.3f,%+.3f) cm/rad\n",
                        fusion.yaw_compensator.sample_count,
                        fusion.yaw_compensator.positive_sample_count,
                        fusion.yaw_compensator.negative_sample_count,
                        fusion.yaw_compensator.x_per_rad_cm,
                        fusion.yaw_compensator.y_per_rad_cm
                    );
                }
                else if (calibration_result.action
                         == MEASUREMENT_CALIBRATION_NEEDS_MORE)
                {
                    printf(
                        "\n[!] 보정 표본 부족: total=%u, CW/CCW=%u/%u\n",
                        fusion.yaw_compensator.sample_count,
                        fusion.yaw_compensator.positive_sample_count,
                        fusion.yaw_compensator.negative_sample_count
                    );
                }
                else
                {
                    puts(
                        "\n[!] fresh BNO heading을 얻지 못했습니다. "
                        "보정 상태를 유지하고 'c'를 다시 누르세요."
                    );
                }

                /*
                 * unavailable이면 이 자리에서 stale 위치로 재arm하지 않고 다음
                 * fresh 측정 loop가 복구한다. learning 중이면 fresh 종료와 PAA
                 * flush가 모두 끝나 LOCKED가 될 때까지 disarmed를 유지한다.
                 */
                if (calibration_result.fresh_heading_used
                    && measurement_collision_rearm_allowed(
                        true,
                        &fusion
                    ))
                {
                    collision_worker_set_armed(collision_worker, true);
                    collision_arm_requested = true;
                }
            }
        }
    }


    collision_worker_set_armed(collision_worker, false);

    if (stdout_is_terminal)
    {
        /* 마지막 실시간 상태줄을 지운 뒤 다중 행 요약을 정상 위치에서 시작한다. */
        fputs("\r\033[2K", stdout);
    }

    (void)get_position_imu_sample(worker, &sample);
    print_measurement_summary(
        &fusion,
        &sample,
        monotonic_time_seconds()
    );
    position_publication_invalidate(position_publication);
    return 0;
}

/* ========================================================================== */
/* Main                                                                        */
/* ========================================================================== */

/*
 * 초기화 순서는 PAA -> 공유 위치 -> IPC sender -> BNO worker -> collision
 * worker -> measurement다. cleanup은 실행 중인 consumer부터 중지하고 장치를
 * 닫아 Thread가 파기된 mutex나 fd를 접근하지 않도록 역방향으로 수행한다.
 * 어느 중간 단계에서 실패해도 이미 성공한 단계만 cleanup하도록 bool flag를
 * 따로 두며, 초기화되지 않은 Thread를 join하거나 mutex를 destroy하지 않는다.
 */

int main(void)
{
    sensor_config_t config;
    paa5100je_t flow;
    imu_worker_t imu_worker;
    position_publication_t position_publication;
    collision_worker_t collision_worker;
    collision_ipc_worker_t collision_ipc_worker;
    bool imu_worker_initialized = false;
    bool position_publication_initialized = false;
    bool collision_worker_initialized = false;
    bool collision_ipc_worker_initialized = false;
    char choice[32];
    int result = EXIT_FAILURE;

    initialize_config_path();

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    signal(SIGPIPE, SIG_IGN);

    puts("================================================================");
    puts(" 사고차량 PAA5100JE + BNO086 위치/충돌 센서 프로세스");
    puts("================================================================");

    if (!accident_testbed_config_is_valid())
    {
        fputs("[-] 사고차량 testbed 시작 좌표/방향 설정이 유효하지 않습니다.\n",
              stderr);
        return EXIT_FAILURE;
    }

    load_sensor_config(&config);
    memset(&flow, 0, sizeof(flow));

    if (paa5100je_open(
            &flow,
            0U,
            1U,
            PAA5100JE_DEFAULT_SPEED_HZ,
            config.height_cm,
            config.scaler
        ) != 0
        || paa5100je_begin(&flow) != 0)
    {
        fprintf(
            stderr,
            "[-] PAA5100JE 초기화 실패: %s\n"
            "    Pin 19(MOSI), 21(MISO), 23(SCLK), 26(CE1)를 확인하세요.\n",
            strerror(errno)
        );
        paa5100je_close(&flow);
        return EXIT_FAILURE;
    }

    printf("[+] PAA5100JE 인식: 높이 %.3f cm, scaler %.3f\n",
           config.height_cm, config.scaler);

    if (position_publication_initialize(&position_publication) != 0)
    {
        fprintf(stderr, "[-] 위치 publication mutex 초기화 실패\n");
        goto cleanup;
    }
    position_publication_initialized = true;
    puts("----------------------------------------------------------------");
    printf(
        " [Enter/1] 저장값 그대로 사용하고 시작 "
        "(높이 %.3f cm, scaler %.3f, 재보정 없음)\n",
        config.height_cm,
        config.scaler
    );
    puts(" [2] 알려진 거리로 PAA5100JE scaler 보정 후 시작");
    puts(" [3] 센서 높이 변경 후 시작");
    puts("----------------------------------------------------------------");

    if (read_line_prompt(
            "선택 [Enter=저장값 사용, 2=거리 보정, 3=높이 변경]: ",
            choice,
            sizeof(choice)
        ) != 0)
    {
        goto cleanup;
    }

    if (strcmp(choice, "2") == 0)
    {
        if (run_distance_calibration(&flow, &config) != 0)
        {
            goto cleanup;
        }
    }
    else if (strcmp(choice, "3") == 0)
    {
        char input[64];

        if (read_line_prompt("새 높이(cm): ", input, sizeof(input)) == 0)
        {
            char *end;
            double height = strtod(input, &end);

            if (end != input && height > 0.0 && isfinite(height))
            {
                config.height_cm = height;
                (void)paa5100je_set_height_cm(&flow, height);
                (void)save_sensor_config(&config);
            }
            else
            {
                puts("[!] 잘못된 높이입니다. 기존 값을 유지합니다.");
            }
        }

        if (read_line_prompt("새 높이에서 거리 보정도 수행합니까? [y/N]: ",
                             input, sizeof(input)) == 0
            && (strcmp(input, "y") == 0 || strcmp(input, "Y") == 0))
        {
            if (run_distance_calibration(&flow, &config) != 0)
            {
                goto cleanup;
            }
        }
    }
    else if (choice[0] != '\0' && strcmp(choice, "1") != 0)
    {
        puts("[!] 잘못된 선택입니다.");
        goto cleanup;
    }

    printf(
        "[*] testbed 시작값: X=%.1f mm, Y=%.1f mm, heading=%.1f deg\n",
        ACCIDENT_START_X_MM,
        ACCIDENT_START_Y_MM,
        ACCIDENT_START_HEADING_DEG
    );
    puts("[*] BNO086 I/O Thread 시작: I2C-1, address 0x4B, INT GPIO18");
    imu_worker_initialized = true;
    if (imu_worker_start(&imu_worker) != 0)
    {
        printf("[!] BNO086 설정 실패: %s\n", imu_worker.start_error);
        puts("    사고차량 프로세스는 PAA 단독으로 강등하지 않고 종료합니다.");
        goto cleanup;
    }
    puts("[+] BNO086: RV/Gyro/Raw Accel/Linear Accel 100Hz, Mag 50Hz");

    if (collision_ipc_worker_start(&collision_ipc_worker) != 0)
    {
        puts("[!] P2P IPC Thread 생성 실패: 사고차량 프로세스를 시작하지 않습니다.");
        goto cleanup;
    }
    collision_ipc_worker_initialized = true;

    if (collision_worker_start(
            &collision_worker,
            &imu_worker,
            &position_publication,
            &collision_ipc_worker
        ) != 0)
    {
        puts("[!] 충돌 판정 Thread 생성 실패: 사고차량 프로세스를 시작하지 않습니다.");
        goto cleanup;
    }
    collision_worker_initialized = true;
    printf(
        "[+] 충돌 판정 Thread 준비: config=%s, raw/linear FC=%u/%u us\n",
        collision_config.enabled ? "ENABLED" : "DISABLED",
        imu_worker.raw_accel_interval_us,
        imu_worker.linear_accel_interval_us
    );
    if (!collision_config.enabled)
    {
        puts("[!] RC-car 실측 임계값 미확정: 충돌 IPC는 기본 비활성입니다.");
    }

    result = run_measurement(
        &flow,
        &config,
        &imu_worker,
        &position_publication,
        &collision_worker
    ) == 0 ? EXIT_SUCCESS : EXIT_FAILURE;

cleanup:
    if (collision_worker_initialized)
    {
        collision_worker_stop(&collision_worker);
    }
    if (collision_ipc_worker_initialized)
    {
        collision_ipc_worker_stop(&collision_ipc_worker);
    }
    if (imu_worker_initialized)
    {
        imu_worker_stop(&imu_worker);
    }
    if (position_publication_initialized)
    {
        position_publication_deinitialize(&position_publication);
    }
    paa5100je_close(&flow);
    return result;
}
