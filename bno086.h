#ifndef BNO086_H
#define BNO086_H

/*
 * BNO086 SH-2/SHTP 드라이버 공개 인터페이스
 *
 * 이 드라이버는 BNO086 하드웨어를 한 개의 I/O 스레드가 소유하는 구조를
 * 전제로 한다. begin(), feature 설정, update()는 반드시 그 소유 스레드에서
 * 순서대로 호출한다. 위치 추적 스레드와 사고 감지 스레드는 하드웨어를 직접
 * 접근하지 않고 snapshot 또는 SPMC event ring을 통해 같은 샘플을 읽는다.
 *
 * 전체 데이터 흐름:
 *
 *     소유 I/O Thread
 *       bno086_begin()
 *           -> reset/startup/Product ID 확인
 *       bno086_enable_feature()
 *           -> FD 전송 -> FE/FC 실제 적용 주기 확인
 *       bno086_update()
 *           -> SHTP packet 해석
 *           -> 최신 snapshot 갱신
 *           -> sample별 event ring 기록
 *
 *     위치 Thread                  충돌 판정 Thread
 *       bno086_get_snapshot()        독립 event cursor
 *                                   bno086_event_next()
 *
 * snapshot은 "현재 최신 상태 한 벌"이 필요할 때 사용하고, event ring은
 * 중간 sample을 빠뜨리지 않고 jerk/delta-V 같은 시간 연속량을 계산할 때
 * 사용한다. 두 경로를 구분해야 느린 소비자가 I/O Thread를 막지 않는다.
 *
 * 처음 읽는 사람을 위한 용어 정리:
 *
 * - SHTP: BNO086과 byte packet을 주고받는 전송 형식이다.
 * - SH-2: 센서 내부에서 자세 계산과 calibration을 수행하는 firmware다.
 * - feature: Rotation Vector, Gyroscope처럼 켜고 주기를 지정할 센서 출력이다.
 * - report: 켜진 feature가 실제 측정값을 담아 보내는 한 개의 sample이다.
 * - snapshot: 여러 종류의 가장 최근 report를 사용하기 쉽게 모아 둔 사본이다.
 *
 * 예를 들어 100 Hz Rotation Vector를 요청했다면 interval_us는 10000이다.
 * enable 성공은 단순히 요청을 전송했다는 뜻이 아니라 SH-2가 non-zero 주기를
 * 돌려줬다는 뜻이며, wait_for_sensor_report까지 성공해야 실제 값이 흐른다.
 *
 * 반환값 규칙:
 *
 *     1 : 성공 또는 새 데이터 있음
 *     0 : timeout/데이터 없음/동작 실패
 *    -1 : 잘못된 인수 또는 초기화되지 않은 객체
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SH-2 sensor report ID. 숫자는 packet에서 어떤 물리량인지 구별하는 표식이다. */
#define BNO086_SENSOR_ACCELEROMETER            0x01u
#define BNO086_SENSOR_GYROSCOPE_CALIBRATED     0x02u
#define BNO086_SENSOR_MAGNETOMETER_CALIBRATED  0x03u
#define BNO086_SENSOR_LINEAR_ACCELERATION      0x04u
#define BNO086_SENSOR_ROTATION_VECTOR          0x05u
#define BNO086_SENSOR_GAME_ROTATION_VECTOR     0x08u

/*
 * Python 기준 구현의 최대 SHTP packet과 동일하다. ring 2048개는 짧은 consumer
 * 지연을 흡수하는 공간이며 영구 기록 공간은 아니다. 소비가 계속 늦으면 덮어쓴다.
 */
#define BNO086_MAX_SHTP_PACKET       284u
#define BNO086_EVENT_RING_CAPACITY  2048u
#define BNO086_CHANNEL_COUNT            6u
#define BNO086_ERROR_TEXT_SIZE        160u

typedef struct
{
    /* Linux I2C 장치와 7-bit 주소. 기본값: /dev/i2c-1, 0x4B. */
    const char *i2c_device;
    uint8_t i2c_address;

    /*
     * libgpiod character-device ABI에 전달할 GPIO chip과 line offset.
     * int_line은 active-low H_INTN 입력, reset_line은 active-low RST 출력이다.
     * 기본값: /dev/gpiochip0, BCM/RP1 line 18(INT), 23(RST).
     */
    const char *gpiochip_device;
    int int_line;
    int reset_line;

} bno086_config_t;

/*
 * 공개 객체에는 구현 포인터만 둔다.
 *
 * 호출자는 객체를 반드시 0으로 초기화한 뒤 bno086_init()을 호출한다.
 * 내부 mutex/condition/ring과 Linux file descriptor는 구현부에 숨겨서 이후
 * sensor 프로세스로 옮길 때 공개 헤더가 불필요하게 커지지 않도록 한다.
 */
typedef struct
{
    void *impl;

} bno086_t;

typedef enum
{
    BNO086_EVENT_NONE = 0,
    BNO086_EVENT_ACCELEROMETER,
    BNO086_EVENT_GYROSCOPE,
    BNO086_EVENT_MAGNETOMETER,
    BNO086_EVENT_LINEAR_ACCELERATION,
    BNO086_EVENT_ORIENTATION

} bno086_event_type_t;

/*
 * ring에 저장되는 하나의 SH-2 sensor sample.
 *
 * orientation:
 *     x/y/z/w == quaternion i/j/k/real
 *
 * accelerometer, linear acceleration:
 *     x/y/z == m/s^2
 *
 * gyroscope:
 *     x/y/z == rad/s
 *
 * magnetometer:
 *     x/y/z == uT
 *
 * ring_sequence는 드라이버 전체 event 순서이고 report_sequence는 SH-2가
 * report 종류별로 제공하는 8-bit 순서다. bno_generation은 reset/startup
 * 경계를 나타내므로, 미분이나 적분 소비자는 generation이 바뀐 두 sample을
 * 절대로 연결해서는 안 된다. monotonic_ns는 Linux CLOCK_MONOTONIC 기준이다.
 * 쉽게 말해 ring_sequence는 "우리 프로그램이 받은 전체 순번",
 * report_sequence는 "센서가 같은 종류 report에 붙인 순번"이다.
 */
typedef struct
{
    uint64_t ring_sequence;
    uint64_t monotonic_ns;
    uint64_t bno_generation;

    bno086_event_type_t type;
    uint8_t report_id;
    uint8_t report_sequence;
    uint8_t accuracy;

    double x;
    double y;
    double z;
    double w;

} bno086_event_t;

/*
 * 각 소비자가 독립적으로 보유하는 cursor.
 * next_sequence가 ring에서 다음에 읽을 전역 sequence를 나타낸다.
 * 소비자가 ring보다 늦어지면 dropped가 증가하고 현재 가장 오래된 sample부터
 * 다시 읽는다. 생산자는 느린 소비자를 기다리지 않는다.
 */
typedef struct
{
    uint64_t next_sequence;
    uint64_t dropped;
    bool initialized;

} bno086_event_cursor_t;

typedef struct
{
    /* 마지막으로 해석한 자세. Euler 각은 표시/진단용, quaternion은 계산용. */
    double roll_deg;
    double pitch_deg;
    double yaw_deg;

    double quat_i;
    double quat_j;
    double quat_k;
    double quat_real;

    double gyro_x_rad_s;
    double gyro_y_rad_s;
    double gyro_z_rad_s;

    double mag_x_ut;
    double mag_y_ut;
    double mag_z_ut;

    double accel_x_mps2;
    double accel_y_mps2;
    double accel_z_mps2;

    double linear_accel_x_mps2;
    double linear_accel_y_mps2;
    double linear_accel_z_mps2;

    /* SH-2 accuracy는 0~3 등급이다. 0은 미보정, 값이 높을수록 신뢰도가 높다. */
    uint8_t rotation_accuracy;
    uint8_t gyro_accuracy;
    uint8_t mag_accuracy;
    uint8_t accel_accuracy;
    uint8_t linear_accel_accuracy;

    /*
     * report별 마지막 수신 시각과 누적 수신 개수(CLOCK_MONOTONIC).
     * sequence가 늘지 않거나 timestamp가 너무 오래됐으면 값 자체가 정상이어도
     * "새 측정값이 계속 들어온다"고 볼 수 없으므로 stale 판단에 사용한다.
     */
    uint64_t last_update_ns;
    uint64_t last_orientation_update_ns;
    uint64_t last_gyro_update_ns;
    uint64_t last_mag_update_ns;
    uint64_t last_accel_update_ns;
    uint64_t last_linear_accel_update_ns;

    uint64_t orientation_sequence;
    uint64_t gyro_sequence;
    uint64_t mag_sequence;
    uint64_t accel_sequence;
    uint64_t linear_accel_sequence;
    /*
     * reset/새 startup 경계마다 증가하여 서로 다른 sensor session을 구분한다.
     * 이 값이 달라졌다면 reset 전 마지막 sample과 reset 후 첫 sample을 연결해
     * jerk나 heading 변화량을 계산하면 안 된다.
     */
    uint64_t bno_generation;

    int last_command_id;
    int last_command_status;

    /* startup handshake 관측 상태. feature 설정/융합 시작 전 반드시 확인한다. */
    bool startup_ready;
    bool boot_advertisement_seen;
    bool boot_reset_seen;
    bool boot_initialize_seen;
    int last_reset_cause;

    /*
     * FE/FC로 실제 확인한 적용 주기. 요청값과 다를 수 있다. 예를 들어 장치가
     * 지원 가능한 주기로 조정해도 non-zero이면 활성화 자체는 성공이다.
     */
    bool rotation_interval_valid;
    bool gyro_interval_valid;
    bool mag_interval_valid;
    bool accel_interval_valid;
    bool linear_accel_interval_valid;

    uint32_t rotation_interval_us;
    uint32_t gyro_interval_us;
    uint32_t mag_interval_us;
    uint32_t accel_interval_us;
    uint32_t linear_accel_interval_us;

    char boot_failure_reason[BNO086_ERROR_TEXT_SIZE];
    char startup_warning[BNO086_ERROR_TEXT_SIZE];

} bno086_snapshot_t;

/*
 * 전송 계층 진단값.
 *
 * syscall_bus_error_count:
 *     read/write/ioctl OSError 대응, short transfer, H_INTN timeout
 *
 * shtp_validation_error_count:
 *     continuation/channel/sequence/length 검증 실패
 *
 * phase2_null_*:
 *     정확히 00 00 00 00인 Phase-2 transient와 1회 recovery 결과
 *
 * 즉 io_error_count 하나만 보고 "배선 오류가 N번 났다"고 판단하면 안 된다.
 * bus와 validation을 나누고, null_count가 있어도 retry_success가 같고
 * retry_fail/validation이 0이면 알려진 transient가 정상 복구된 것이다.
 */
typedef struct
{
    /* 하위 호환 총계. 상세 원인은 아래 세 분리 counter를 우선 사용한다. */
    uint64_t io_error_count;
    uint64_t syscall_bus_error_count;
    uint64_t shtp_validation_error_count;
    uint64_t other_transport_error_count;

    uint64_t phase2_null_count;
    uint64_t phase2_null_retry_success;
    uint64_t phase2_null_retry_fail;

    /* reset과 channel별 packet/event-ring 포화 상태를 장기 안정성에 사용한다. */
    uint64_t reset_count;
    uint64_t packet_counts[BNO086_CHANNEL_COUNT];
    uint64_t event_overwrite_count;

    int last_errno;
    char last_io_error[BNO086_ERROR_TEXT_SIZE];
    char last_syscall_error[BNO086_ERROR_TEXT_SIZE];
    char last_validation_error[BNO086_ERROR_TEXT_SIZE];

    uint8_t last_shtp_errors[32];
    size_t last_shtp_error_count;

} bno086_diagnostics_t;

/* 기본 Linux 장치명, I2C 주소, GPIO line을 채운다. 필요하면 init 전에 덮어쓴다. */
void bno086_default_config(bno086_config_t *config);

/*
 * opaque 내부 상태/mutex/ring을 만들고 없앤다. init은 메모리 준비이고 open은
 * 실제 Linux 장치 열기이므로 서로 다르다. deinit 전 close가 권장된다.
 */
int bno086_init(bno086_t *imu, const bno086_config_t *config);
void bno086_deinit(bno086_t *imu);

/* 장치 FD와 GPIO line만 열고 닫는다. SH-2 startup은 begin()의 역할이다. */
int bno086_open(bno086_t *imu);
void bno086_close(bno086_t *imu);
int bno086_is_open(const bno086_t *imu);

int bno086_reset_hardware(bno086_t *imu);
int bno086_is_int_asserted(bno086_t *imu);

/*
 * open + hardware reset + startup handshake + Product-ID 확인.
 * 성공해도 sensor report는 아직 활성화되지 않으므로 필요한 feature를
 * 하나씩 enable하고 실제 input report까지 확인해야 한다.
 */
int bno086_begin(bno086_t *imu);

/*
 * pending packet을 한 호출에서 최대 8개 처리하고 갱신된 sensor report 수를
 * 반환한다. 상한은 고율 packet 상황에서도 stop 요청과 다른 작업을 주기적으로
 * 확인할 수 있게 하기 위한 공정성 제한이다.
 * 이 함수는 background service가 아니므로 소유 Thread가 반복 호출해야 새
 * snapshot과 event가 생긴다.
 */
int bno086_update(bno086_t *imu);

/*
 * FD(Set Feature) 후 FE(Get Feature)를 반복하여 같은 sensor ID의 FC에서
 * 실제 적용 interval을 확인한다. enable 요청 중 interval=0 FC는 전이/stale
 * 응답으로 무시하며, non-zero FC가 올 때까지 하나의 전체 deadline 안에서
 * 재조회한다. 요청 interval과 정확히 같지 않아도 SH-2가 적용한 non-zero
 * 값을 성공으로 인정한다.
 * interval_us==0은 향후 disable 의미이며 이때는 FC의 실제 값도 0이어야 한다.
 */
int bno086_enable_feature(
    bno086_t *imu,
    uint8_t sensor_id,
    uint32_t interval_us,
    unsigned int timeout_ms
);

int bno086_enable_rotation_vector(bno086_t *imu, uint32_t interval_us);
int bno086_enable_game_rotation_vector(bno086_t *imu, uint32_t interval_us);
int bno086_enable_gyroscope(bno086_t *imu, uint32_t interval_us);
int bno086_enable_magnetometer(bno086_t *imu, uint32_t interval_us);
int bno086_enable_accelerometer(bno086_t *imu, uint32_t interval_us);
int bno086_enable_linear_acceleration(bno086_t *imu, uint32_t interval_us);

/*
 * enable 이후 실제 Channel 3/4 sensor report가 들어오는지 2단계로 확인한다.
 * 설정표만 바뀌고 데이터가 오지 않는 고장을 시작 단계에서 걸러내기 위한 함수다.
 */
int bno086_wait_for_sensor_report(
    bno086_t *imu,
    uint8_t sensor_id,
    unsigned int timeout_ms
);

/* SH-2 calibration/tare command를 보내고 command response status까지 확인한다. */
int bno086_start_dynamic_calibration(bno086_t *imu);
int bno086_tare_all_axes(bno086_t *imu);

/* mutex 아래에서 최신 상태/진단을 통째로 복사하므로 호출 후 사본은 독립적이다. */
int bno086_get_snapshot(const bno086_t *imu, bno086_snapshot_t *snapshot);
int bno086_get_diagnostics(
    const bno086_t *imu,
    bno086_diagnostics_t *diagnostics
);

/* 마지막으로 관측한 유효 FC interval을 반환한다. 확인된 FC가 없으면 실패한다. */
int bno086_get_feature_interval(
    const bno086_t *imu,
    uint8_t sensor_id,
    uint32_t *interval_us
);

/*
 * include_buffered == true이면 현재 ring에 남아 있는 가장 오래된 event부터,
 * false이면 초기화 이후 새로 들어오는 event부터 읽는다.
 */
int bno086_event_cursor_init(
    const bno086_t *imu,
    bno086_event_cursor_t *cursor,
    bool include_buffered
);

/*
 * cursor 하나만 전진시키며 event 내용을 복사한다.
 * timeout_ms < 0: 무한 대기, == 0: non-blocking, > 0: 제한 대기.
 * ring overwrite를 따라잡으면 cursor.dropped가 증가하므로 소비자는 미분/적분
 * 상태를 초기화하고 반환된 sample부터 새 구간을 시작해야 한다.
 */
int bno086_event_next(
    const bno086_t *imu,
    bno086_event_cursor_t *cursor,
    bno086_event_t *event,
    int timeout_ms
);

/* 사람이 읽을 수 있는 startup/transport 상태 한 줄을 만든다. */
int bno086_status_summary(
    const bno086_t *imu,
    char *buffer,
    size_t buffer_size
);

/*
 * Production build에는 노출되지 않는 deterministic transport test hook.
 * scratch/test_bno086_c.c를 -DBNO086_TESTING으로 빌드할 때만 사용한다.
 */
#ifdef BNO086_TESTING

#define BNO086_TEST_MAX_WRITES 64u

typedef struct
{
    const uint8_t *data;
    size_t size;
    int error_number;
    size_t required_write_count;

} bno086_test_read_step_t;

typedef struct
{
    uint16_t total_length;
    uint8_t channel;
    uint8_t sequence;
    size_t cargo_length;
    uint8_t cargo[BNO086_MAX_SHTP_PACKET - 4u];

} bno086_test_packet_t;

void bno086_test_transport_reset(void);
void bno086_test_set_read_script(
    const bno086_test_read_step_t *steps,
    size_t step_count
);
size_t bno086_test_read_call_count(void);
size_t bno086_test_write_count(void);
size_t bno086_test_copy_write(
    size_t index,
    uint8_t *buffer,
    size_t buffer_size
);

int bno086_test_make_virtual_open(bno086_t *imu);
int bno086_test_set_startup_ready(bno086_t *imu, bool ready);
int bno086_test_read_packet(
    bno086_t *imu,
    bno086_test_packet_t *packet
);
int bno086_test_feed_packet(
    bno086_t *imu,
    uint8_t channel,
    uint8_t sequence,
    const uint8_t *cargo,
    size_t cargo_length
);

#endif

#ifdef __cplusplus
}
#endif

#endif
