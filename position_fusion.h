#ifndef POSITION_FUSION_H
#define POSITION_FUSION_H

#include <stdbool.h>
#include <stdint.h>

/*
 * PAA5100JE와 BNO086을 결합하는 위치 계산의 순수 C 인터페이스다.
 *
 * 이 파일은 Linux 장치나 pthread를 직접 다루지 않는다. 실제 센서값과
 * monotonic 시각을 입력받아 같은 결과를 돌려주므로, PAA5100JE가 없는
 * 개발 환경에서도 Python 기준 구현과 동일한 입력으로 회귀시험할 수 있다.
 *
 * 좌표 규약:
 *
 * - PAA dx/dy는 센서가 장착된 차량 body frame의 이동량이다.
 * - reference quaternion을 잡은 순간의 차량 방향을 world heading 0도로 둔다.
 * - body 이동은 sample 전후 heading의 midpoint로 회전해 world x/y에 적분한다.
 * - 위치 단위는 cm, heading은 degree, gyro는 rad/s, 시간은 monotonic second다.
 *
 * stale과 accuracy는 의도적으로 다르다. stale RV/gyro는 방향을 신뢰할 수 없어
 * 해당 flow 적분을 중단한다. accuracy 2 미만은 진단 누계만 남기고 순간 하락만으로
 * 주행 sample을 버리거나 heading reference를 다시 잡지는 않는다.
 *
 * 쉽게 풀어쓰면 PAA는 "차량 기준으로 어느 방향으로 움직였는가"를 알고,
 * BNO는 "차량이 어느 방향을 보고 있는가"를 안다. 차량이 오른쪽으로 90도
 * 돌아선 뒤 body 기준 전진했다면, 같은 PAA 전진값도 world 좌표에서는 옆 방향
 * 이동이 된다. 이 파일은 바로 그 회전과 누적을 담당한다.
 *
 * 한 burst의 처리 예:
 *
 *     raw tick -> 아주 작은 흔들림 제거 -> SQUAL 확인 -> heading 확인
 *              -> tick을 cm로 환산 -> 회전 때문에 생긴 가짜 flow 감산
 *              -> body 좌표를 world 좌표로 회전 -> x/y에 누적
 */

#define POSITION_MIN_SQUAL                 25U
#define POSITION_FLOW_DEADBAND_TICKS       1
#define POSITION_IMU_STALE_SECONDS         0.20
#define POSITION_MAG_STALE_SECONDS         0.50
#define POSITION_MAX_YAW_RATE_DEG_S        540.0
#define POSITION_MAX_YAW_EXTRAPOLATION_S   0.030
#define POSITION_MIN_FUSION_ACCURACY       2U

/*
 * 위 값들은 정책 경계다. SQUAL/시간/각속도 제한은 이상 sample이 좌표를 크게
 * 튀게 하는 것을 막는다. 실제 차량·바닥에서 적절한지는 PAA 연결 후 확인한다.
 */

typedef struct
{
    double real;
    double i;
    double j;
    double k;
} position_quaternion_t;

/*
 * BNO086 드라이버 snapshot을 융합 계층에 전달할 때 사용하는 최소 상태다.
 * timestamp는 CLOCK_MONOTONIC 기준 초 단위 값이다.
 * sequence는 값이 갱신됐는지를, time은 그 값이 지금도 충분히 최근인지를
 * 각각 판단하므로 둘 중 하나만으로 대체하지 않는다.
 */
typedef struct
{
    bool startup_ready;

    uint64_t orientation_sequence;
    uint64_t gyro_sequence;
    uint64_t mag_sequence;

    double orientation_time_s;
    double gyro_time_s;
    double mag_time_s;

    position_quaternion_t quaternion;

    double gyro_x_rad_s;
    double gyro_y_rad_s;
    double gyro_z_rad_s;

    uint8_t rotation_accuracy;
    uint8_t mag_accuracy;
} position_imu_sample_t;

/*
 * 시작 quaternion과 최신 report를 보관한다. report 사이의 짧은 시간은 직전
 * 두 heading으로 구한 제한된 yaw rate를 사용해 flow 시각까지 보간한다.
 */
typedef struct
{
    position_quaternion_t reference;
    uint64_t last_orientation_sequence;
    double last_report_time_s;
    double last_report_heading_deg;
    double yaw_rate_deg_s;
} relative_heading_tracker_t;

/*
 * 센서와 차량 회전 중심 사이 offset 때문에 순수 yaw에서도 보이는 body-frame
 * flow를 cm/rad 계수로 학습한다. CW/CCW 양쪽 sample이 있어야 calibrated가 된다.
 * 센서가 회전 중심에서 떨어져 있으면 제자리 회전에서도 작은 원호를 그리기
 * 때문에 PAA에는 이동처럼 보인다. 이 구조체는 그 반복 성분을 기억한다.
 */
typedef struct
{
    double x_per_rad_cm;
    double y_per_rad_cm;

    uint32_t sample_count;
    uint32_t positive_sample_count;
    uint32_t negative_sample_count;

    bool learning;
    bool calibrated;
} yaw_flow_compensator_t;

typedef enum
{
    /* 좌표 반영 성공. */
    POSITION_SAMPLE_APPLIED = 0,
    /* 바닥 영상 품질 부족으로 좌표 반영 안 함. */
    POSITION_SAMPLE_LOW_SQUAL,
    /* BNO heading/gyro가 오래되어 좌표 반영 안 함. */
    POSITION_SAMPLE_IMU_STALE,
    /* 보정용으로 읽었지만 의도적으로 좌표 반영 안 함. */
    POSITION_SAMPLE_CALIBRATION_DISCARDED
} position_sample_result_t;

typedef struct
{
    /* 해당 burst가 적용/폐기된 이유와 계산 중간값을 호출자 진단에 돌려준다. */
    position_sample_result_t result;

    int16_t raw_dx_ticks;
    int16_t raw_dy_ticks;
    int16_t filtered_dx_ticks;
    int16_t filtered_dy_ticks;
    uint8_t squal;

    double heading_deg;
    double heading_mid_deg;
    double delta_heading_deg;
    double dx_world_cm;
    double dy_world_cm;

    bool imu_fresh;
    bool accuracy_ready;
} position_step_t;

/*
 * 위치 적분기의 지속 상태다. raw tick 누계는 품질과 무관한 진단값이고 x/y는
 * 허용된 sample만 누적한다. imu_was_stale은 복구 직후 이전 heading gap을
 * midpoint 회전에 사용하지 않게 하는 경계 flag다.
 */
typedef struct
{
    double height_cm;
    double scaler;

    double x_cm;
    double y_cm;
    int64_t raw_ticks_x;
    int64_t raw_ticks_y;
    uint8_t last_squal;

    bool fusion_enabled;
    bool imu_was_stale;

    bool accuracy_was_low;
    uint32_t accuracy_low_events;
    double accuracy_low_started_s;
    double accuracy_low_total_s;

    double last_flow_heading_deg;
    relative_heading_tracker_t heading_tracker;
    yaw_flow_compensator_t yaw_compensator;
} position_fusion_t;

double position_wrap_angle_deg(double angle_deg);
double position_angle_delta_deg(double new_angle_deg, double old_angle_deg);

/* Angle 함수는 [-180, 180) 규약을 사용해 179 -> -179를 +2도로 계산한다. */

position_quaternion_t position_quaternion_normalize(
    position_quaternion_t quaternion
);
position_quaternion_t position_quaternion_multiply(
    position_quaternion_t left,
    position_quaternion_t right
);
position_quaternion_t position_quaternion_conjugate(
    position_quaternion_t quaternion
);
double position_yaw_from_quaternion_deg(position_quaternion_t quaternion);

void relative_heading_reset(
    relative_heading_tracker_t *tracker,
    const position_imu_sample_t *sample
);
double relative_heading_raw_deg(
    const relative_heading_tracker_t *tracker,
    const position_imu_sample_t *sample
);
double relative_heading_at_deg(
    relative_heading_tracker_t *tracker,
    const position_imu_sample_t *sample,
    double now_s
);

/* raw는 최신 quaternion의 상대 yaw이고, at은 report age만큼 제한 보간한 값이다. */

void yaw_flow_compensator_init(yaw_flow_compensator_t *compensator);
void yaw_flow_compensator_start(yaw_flow_compensator_t *compensator);
bool yaw_flow_compensator_finish(yaw_flow_compensator_t *compensator);
void yaw_flow_compensator_apply(
    yaw_flow_compensator_t *compensator,
    double dx_cm,
    double dy_cm,
    double delta_yaw_rad,
    double gyro_magnitude_rad_s,
    double *corrected_dx_cm,
    double *corrected_dy_cm
);

/* learning 중 apply 결과는 항상 0 displacement이며 calibration flow는 적분되지 않는다. */

bool position_fusion_reports_are_fresh(
    const position_imu_sample_t *sample,
    double now_s
);
bool position_fusion_accuracy_is_ready(const position_imu_sample_t *sample);

/* reports_are_fresh는 startup reference용 RV/Gyro/Mag 조건, accuracy는 별도 품질 조건이다. */

void position_fusion_init(
    position_fusion_t *fusion,
    double height_cm,
    double scaler,
    const position_imu_sample_t *reference_sample
);
void position_fusion_set_geometry(
    position_fusion_t *fusion,
    double height_cm,
    double scaler
);
void position_fusion_process(
    position_fusion_t *fusion,
    int16_t raw_dx_ticks,
    int16_t raw_dy_ticks,
    uint8_t squal,
    const position_imu_sample_t *imu_sample,
    double now_s,
    position_step_t *step
);

/*
 * process는 raw tick 진단 누계 -> deadband/SQUAL -> IMU freshness -> heading
 * midpoint -> lever-arm 보정 -> body/world 회전 -> 위치 적분 순으로 한 burst를
 * 처리하고 모든 판단 결과를 position_step_t에 남긴다.
 * 반환값 대신 step.result를 쓰는 이유는 "함수 오류"가 아니라 "정상적으로
 * 읽었지만 품질 정책상 적분하지 않은 sample"도 구분해 보여주기 위해서다.
 */

/*
 * 'r' 명령과 동일하다. IMU가 fresh이면 현재 quaternion도 새 상대 기준으로
 * 지정하고, 그렇지 않으면 좌표와 원시 tick만 초기화한다.
 * stale일 때 heading까지 억지로 0으로 만들면 다음 fresh sample에서 큰 가짜
 * 회전이 생길 수 있으므로 좌표 reset 성공과 heading reset 성공을 구분한다.
 */
bool position_fusion_reset_origin(
    position_fusion_t *fusion,
    const position_imu_sample_t *imu_sample,
    double now_s
);

/*
 * 'c' 보정 경계를 나타낸다. 시작과 종료 모두 현재 fresh heading으로
 * last_flow_heading을 맞춰 보정 중 전체 회전이 다음 정상 flow에 섞이지 않게 한다.
 * 보정 중에도 매 burst를 읽고 apply하여 PAA 누적 delta가 종료 뒤로 넘어가지 않으며,
 * 그 flow의 corrected displacement는 0이라 좌표에는 들어가지 않는다.
 */
bool position_fusion_start_yaw_calibration(
    position_fusion_t *fusion,
    const position_imu_sample_t *imu_sample,
    double now_s
);
bool position_fusion_finish_yaw_calibration(
    position_fusion_t *fusion,
    const position_imu_sample_t *imu_sample,
    double now_s
);

double position_fusion_accuracy_low_total(
    const position_fusion_t *fusion,
    double now_s
);

#endif
