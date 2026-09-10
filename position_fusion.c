#define _POSIX_C_SOURCE 200809L

#include "position_fusion.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define YAW_MIN_TURN_RAD              (0.35 * M_PI / 180.0)
#define YAW_MIN_GYRO_RAD_S            0.35
#define YAW_MAX_LEVER_ARM_CM          20.0
#define YAW_MAX_TRANSLATION_MARGIN_CM 0.25
#define YAW_MIN_SAMPLES               8U
#define YAW_MIN_DIRECTION_SAMPLES     2U
#define YAW_LEARNING_ALPHA            0.20

/*
 * 이 모듈은 장치 I/O나 전역 상태가 없는 결정적 계산 계층이다. production
 * sensor process와 synthetic regression이 같은 함수를 직접 호출하므로 수식을
 * 시험용으로 복제해 서로 달라지는 일을 막는다.
 * 계산 함수가 파일이나 장치를 직접 읽지 않기 때문에 같은 입력에는 항상 같은
 * 결과가 나온다. 센서 문제와 수학 문제를 나누어 확인하기 위한 구조다.
 */

/* ========================================================================== */
/* 각도와 quaternion 공통 계산                                                */
/* ========================================================================== */

double position_wrap_angle_deg(double angle_deg)
{
    /* 예: 181도는 같은 방향인 -179도로 표현해 경계에서 값이 갑자기 커지지 않는다. */
    double wrapped = fmod(angle_deg + 180.0, 360.0);

    if (wrapped < 0.0)
    {
        wrapped += 360.0;
    }

    return wrapped - 180.0;
}

double position_angle_delta_deg(double new_angle_deg, double old_angle_deg)
{
    /* 360도 경계를 가로질러도 가장 짧은 signed 회전량만 반환한다. */
    return position_wrap_angle_deg(new_angle_deg - old_angle_deg);
}

position_quaternion_t position_quaternion_normalize(
    position_quaternion_t quaternion
)
{
    double norm = sqrt(
        quaternion.real * quaternion.real
        + quaternion.i * quaternion.i
        + quaternion.j * quaternion.j
        + quaternion.k * quaternion.k
    );

    /* 거의 0인 잘못된 quaternion은 0으로 나누지 않고 회전 없는 identity로 둔다. */
    if (norm <= 1.0e-9)
    {
        position_quaternion_t identity = {1.0, 0.0, 0.0, 0.0};
        return identity;
    }

    quaternion.real /= norm;
    quaternion.i /= norm;
    quaternion.j /= norm;
    quaternion.k /= norm;
    return quaternion;
}

position_quaternion_t position_quaternion_multiply(
    position_quaternion_t left,
    position_quaternion_t right
)
{
    position_quaternion_t result;

    result.real =
        left.real * right.real
        - left.i * right.i
        - left.j * right.j
        - left.k * right.k;
    result.i =
        left.real * right.i
        + left.i * right.real
        + left.j * right.k
        - left.k * right.j;
    result.j =
        left.real * right.j
        - left.i * right.k
        + left.j * right.real
        + left.k * right.i;
    result.k =
        left.real * right.k
        + left.i * right.j
        - left.j * right.i
        + left.k * right.real;

    return result;
}

position_quaternion_t position_quaternion_conjugate(
    position_quaternion_t quaternion
)
{
    quaternion.i = -quaternion.i;
    quaternion.j = -quaternion.j;
    quaternion.k = -quaternion.k;
    return quaternion;
}

double position_yaw_from_quaternion_deg(position_quaternion_t quaternion)
{
    double sin_yaw = 2.0 * (
        quaternion.real * quaternion.k
        + quaternion.i * quaternion.j
    );
    double cos_yaw = 1.0 - 2.0 * (
        quaternion.j * quaternion.j
        + quaternion.k * quaternion.k
    );

    return atan2(sin_yaw, cos_yaw) * 180.0 / M_PI;
}

/* ========================================================================== */
/* 시작 quaternion 기준 상대 heading                                          */
/* ========================================================================== */

void relative_heading_reset(
    relative_heading_tracker_t *tracker,
    const position_imu_sample_t *sample
)
{
    if (tracker == NULL || sample == NULL)
    {
        return;
    }

    /* 이 순간의 실제 절대 방향이 무엇이든 이후 계산에서는 상대 heading 0도다. */
    tracker->reference = position_quaternion_normalize(sample->quaternion);
    tracker->last_orientation_sequence = sample->orientation_sequence;
    tracker->last_report_time_s = sample->orientation_time_s;
    tracker->last_report_heading_deg = 0.0;
    tracker->yaw_rate_deg_s = 0.0;
}

double relative_heading_raw_deg(
    const relative_heading_tracker_t *tracker,
    const position_imu_sample_t *sample
)
{
    position_quaternion_t current;
    position_quaternion_t relative;

    if (tracker == NULL || sample == NULL)
    {
        return 0.0;
    }

    current = position_quaternion_normalize(sample->quaternion);
    /* current * conjugate(reference)는 시작 absolute yaw offset을 제거한다. */
    relative = position_quaternion_multiply(
        current,
        position_quaternion_conjugate(tracker->reference)
    );

    return position_wrap_angle_deg(
        position_yaw_from_quaternion_deg(relative)
    );
}

double relative_heading_at_deg(
    relative_heading_tracker_t *tracker,
    const position_imu_sample_t *sample,
    double now_s
)
{
    double raw_heading;
    double age;

    if (tracker == NULL || sample == NULL)
    {
        return 0.0;
    }

    raw_heading = relative_heading_raw_deg(tracker, sample);

    if (sample->orientation_sequence != tracker->last_orientation_sequence)
    {
        double report_dt =
            sample->orientation_time_s - tracker->last_report_time_s;

        /* 새 report일 때만 rate를 갱신하고 비정상 dt/rate는 외삽하지 않는다. */
        if (report_dt >= 0.001 && report_dt <= POSITION_IMU_STALE_SECONDS)
        {
            double measured_rate = position_angle_delta_deg(
                raw_heading,
                tracker->last_report_heading_deg
            ) / report_dt;

            if (measured_rate > POSITION_MAX_YAW_RATE_DEG_S)
            {
                measured_rate = POSITION_MAX_YAW_RATE_DEG_S;
            }
            else if (measured_rate < -POSITION_MAX_YAW_RATE_DEG_S)
            {
                measured_rate = -POSITION_MAX_YAW_RATE_DEG_S;
            }

            tracker->yaw_rate_deg_s = measured_rate;
        }
        else
        {
            tracker->yaw_rate_deg_s = 0.0;
        }

        tracker->last_orientation_sequence = sample->orientation_sequence;
        tracker->last_report_time_s = sample->orientation_time_s;
        tracker->last_report_heading_deg = raw_heading;
    }

    /* Flow read와 마지막 RV report 사이의 작은 시간차만 제한적으로 보상한다. */
    age = now_s - sample->orientation_time_s;
    if (age < 0.0)
    {
        age = 0.0;
    }
    else if (age > POSITION_MAX_YAW_EXTRAPOLATION_S)
    {
        age = POSITION_MAX_YAW_EXTRAPOLATION_S;
    }

    return position_wrap_angle_deg(
        raw_heading + tracker->yaw_rate_deg_s * age
    );
}

/* ========================================================================== */
/* 명시적으로 실행하는 회전 중심 offset 학습                                  */
/* ========================================================================== */

void yaw_flow_compensator_init(yaw_flow_compensator_t *compensator)
{
    if (compensator == NULL)
    {
        return;
    }

    memset(compensator, 0, sizeof(*compensator));
}

void yaw_flow_compensator_start(yaw_flow_compensator_t *compensator)
{
    if (compensator == NULL)
    {
        return;
    }

    /* 이전 보정값과 표본을 모두 비우므로 사용자가 의도적으로 시작할 때만 호출한다. */
    memset(compensator, 0, sizeof(*compensator));
    compensator->learning = true;
}

bool yaw_flow_compensator_finish(yaw_flow_compensator_t *compensator)
{
    bool ready;

    if (compensator == NULL)
    {
        return false;
    }

    /* 한 방향 회전만으로 translation 성분을 잘못 학습하지 않도록 양방향을 요구한다. */
    ready =
        compensator->sample_count >= YAW_MIN_SAMPLES
        && compensator->positive_sample_count >= YAW_MIN_DIRECTION_SAMPLES
        && compensator->negative_sample_count >= YAW_MIN_DIRECTION_SAMPLES;

    if (ready)
    {
        compensator->learning = false;
        compensator->calibrated = true;
    }

    return ready;
}

void yaw_flow_compensator_apply(
    yaw_flow_compensator_t *compensator,
    double dx_cm,
    double dy_cm,
    double delta_yaw_rad,
    double gyro_magnitude_rad_s,
    double *corrected_dx_cm,
    double *corrected_dy_cm
)
{
    if (
        compensator == NULL
        || corrected_dx_cm == NULL
        || corrected_dy_cm == NULL
    )
    {
        return;
    }

    if (!compensator->learning)
    {
        if (!compensator->calibrated)
        {
            *corrected_dx_cm = dx_cm;
            *corrected_dy_cm = dy_cm;
            return;
        }

        /* 정상 주행에서는 학습된 cm/rad * 이번 yaw 변화량을 raw flow에서 뺀다. */
        *corrected_dx_cm =
            dx_cm - compensator->x_per_rad_cm * delta_yaw_rad;
        *corrected_dy_cm =
            dy_cm - compensator->y_per_rad_cm * delta_yaw_rad;
        return;
    }

    /*
     * 보정 모드에서 읽은 PAA delta는 학습 조건을 통과하지 못하더라도 위치에
     * 적분하지 않는다. 이 규칙이 calibration 종료 경계 회귀시험의 핵심이다.
     * 사용자는 제자리에서 CW/CCW로 돌리므로 이때 보이는 flow는 차량 이동이
     * 아니라 센서 장착 offset을 학습하기 위한 자료로만 취급한다.
     */
    *corrected_dx_cm = 0.0;
    *corrected_dy_cm = 0.0;

    if (
        fabs(delta_yaw_rad) < YAW_MIN_TURN_RAD
        || gyro_magnitude_rad_s < YAW_MIN_GYRO_RAD_S
    )
    {
        return;
    }

    {
        double max_turn_distance =
            YAW_MAX_LEVER_ARM_CM * fabs(delta_yaw_rad)
            + YAW_MAX_TRANSLATION_MARGIN_CM;
        double raw_distance = hypot(dx_cm, dy_cm);

        /* Lever arm 최대 반경으로 설명되지 않는 큰 값은 실제 translation으로 보고 학습 제외한다. */
        if (raw_distance <= max_turn_distance)
        {
            double candidate_x = dx_cm / delta_yaw_rad;
            double candidate_y = dy_cm / delta_yaw_rad;
            double candidate_distance = hypot(candidate_x, candidate_y);

            if (candidate_distance <= YAW_MAX_LEVER_ARM_CM)
            {
                if (compensator->sample_count == 0U)
                {
                    compensator->x_per_rad_cm = candidate_x;
                    compensator->y_per_rad_cm = candidate_y;
                }
                else
                {
                    compensator->x_per_rad_cm += YAW_LEARNING_ALPHA * (
                        candidate_x - compensator->x_per_rad_cm
                    );
                    compensator->y_per_rad_cm += YAW_LEARNING_ALPHA * (
                        candidate_y - compensator->y_per_rad_cm
                    );
                }

                compensator->sample_count++;
                if (delta_yaw_rad > 0.0)
                {
                    compensator->positive_sample_count++;
                }
                else
                {
                    compensator->negative_sample_count++;
                }
            }
        }
    }
}

/* ========================================================================== */
/* 리포트 준비 상태                                                            */
/* ========================================================================== */

bool position_fusion_reports_are_fresh(
    const position_imu_sample_t *sample,
    double now_s
)
{
    if (sample == NULL)
    {
        return false;
    }

    /* 초기 reference에는 RV, gyro, magnetometer 세 stream이 모두 필요하다. */
    return
        sample->startup_ready
        && sample->orientation_sequence > 0U
        && sample->gyro_sequence > 0U
        && sample->mag_sequence > 0U
        && now_s - sample->orientation_time_s < POSITION_IMU_STALE_SECONDS
        && now_s - sample->gyro_time_s < POSITION_IMU_STALE_SECONDS
        && now_s - sample->mag_time_s < POSITION_MAG_STALE_SECONDS;
}

bool position_fusion_accuracy_is_ready(const position_imu_sample_t *sample)
{
    return
        sample != NULL
        && sample->rotation_accuracy >= POSITION_MIN_FUSION_ACCURACY
        && sample->mag_accuracy >= POSITION_MIN_FUSION_ACCURACY;
}

/* ========================================================================== */
/* 한 PAA5100JE burst를 위치에 반영하는 production 계산                        */
/* ========================================================================== */

void position_fusion_init(
    position_fusion_t *fusion,
    double height_cm,
    double scaler,
    const position_imu_sample_t *reference_sample
)
{
    if (fusion == NULL)
    {
        return;
    }

    memset(fusion, 0, sizeof(*fusion));
    fusion->height_cm = height_cm;
    fusion->scaler = scaler;
    yaw_flow_compensator_init(&fusion->yaw_compensator);

    if (reference_sample != NULL)
    {
        fusion->fusion_enabled = true;
        relative_heading_reset(&fusion->heading_tracker, reference_sample);
    }
}

void position_fusion_set_geometry(
    position_fusion_t *fusion,
    double height_cm,
    double scaler
)
{
    if (fusion == NULL)
    {
        return;
    }

    fusion->height_cm = height_cm;
    fusion->scaler = scaler;
}

static bool orientation_is_fresh(
    const position_imu_sample_t *sample,
    double now_s
)
{
    return
        sample != NULL
        && sample->startup_ready
        && now_s - sample->orientation_time_s < POSITION_IMU_STALE_SECONDS;
}

static bool runtime_imu_is_active(
    const position_imu_sample_t *sample,
    double now_s
)
{
    /* 주행 계산은 heading과 각속도만 필요하므로 순간 Mag accuracy 저하와 분리한다. */
    return
        orientation_is_fresh(sample, now_s)
        && now_s - sample->gyro_time_s < POSITION_IMU_STALE_SECONDS;
}

static void update_accuracy_diagnostics(
    position_fusion_t *fusion,
    const position_imu_sample_t *sample,
    double now_s
)
{
    bool accuracy_low = !position_fusion_accuracy_is_ready(sample);

    /* 품질 하락 구간의 횟수/시간만 계측하며 위치 sample의 사용 여부는 바꾸지 않는다. */
    if (accuracy_low && !fusion->accuracy_was_low)
    {
        fusion->accuracy_low_events++;
        fusion->accuracy_low_started_s = now_s;
    }
    else if (!accuracy_low && fusion->accuracy_was_low)
    {
        fusion->accuracy_low_total_s +=
            now_s - fusion->accuracy_low_started_s;
        fusion->accuracy_low_started_s = 0.0;
    }

    fusion->accuracy_was_low = accuracy_low;
}

void position_fusion_process(
    position_fusion_t *fusion,
    int16_t raw_dx_ticks,
    int16_t raw_dy_ticks,
    uint8_t squal,
    const position_imu_sample_t *imu_sample,
    double now_s,
    position_step_t *step
)
{
    int16_t dx_ticks;
    int16_t dy_ticks;

    if (fusion == NULL || step == NULL)
    {
        return;
    }

    memset(step, 0, sizeof(*step));
    step->raw_dx_ticks = raw_dx_ticks;
    step->raw_dy_ticks = raw_dy_ticks;
    step->squal = squal;
    step->accuracy_ready = position_fusion_accuracy_is_ready(imu_sample);

    fusion->raw_ticks_x += raw_dx_ticks;
    fusion->raw_ticks_y += raw_dy_ticks;
    fusion->last_squal = squal;

    if (fusion->fusion_enabled)
    {
        update_accuracy_diagnostics(fusion, imu_sample, now_s);
    }

    /* raw 누계는 보존하고 위치 계산에만 작은 정지 noise deadband를 적용한다. */
    dx_ticks = abs((int)raw_dx_ticks) <= POSITION_FLOW_DEADBAND_TICKS
        ? 0
        : raw_dx_ticks;
    dy_ticks = abs((int)raw_dy_ticks) <= POSITION_FLOW_DEADBAND_TICKS
        ? 0
        : raw_dy_ticks;
    step->filtered_dx_ticks = dx_ticks;
    step->filtered_dy_ticks = dy_ticks;

    if (squal < POSITION_MIN_SQUAL)
    {
        step->result = POSITION_SAMPLE_LOW_SQUAL;

        /*
         * 낮은 SQUAL flow는 버리되 fresh heading은 계속 전진시킨다. 그렇지 않으면
         * 다음 정상 flow가 저품질 구간 전체 회전의 midpoint 영향을 받는다.
         */
        if (fusion->fusion_enabled)
        {
            if (orientation_is_fresh(imu_sample, now_s))
            {
                step->heading_deg = relative_heading_at_deg(
                    &fusion->heading_tracker,
                    imu_sample,
                    now_s
                );
                fusion->last_flow_heading_deg = step->heading_deg;
                fusion->imu_was_stale = false;
                step->imu_fresh = true;
            }
            else
            {
                fusion->imu_was_stale = true;
            }
        }

        return;
    }

    if (!fusion->fusion_enabled)
    {
        /* IMU를 쓰지 않는 명시적 모드는 body 축을 그대로 world 축으로 취급한다. */
        double scale = fusion->scaler != 0.0
            ? fusion->height_cm / fusion->scaler
            : 0.0;

        step->result = POSITION_SAMPLE_APPLIED;
        step->dx_world_cm = (double)dx_ticks * scale;
        step->dy_world_cm = (double)dy_ticks * scale;
        fusion->x_cm += step->dx_world_cm;
        fusion->y_cm += step->dy_world_cm;
        return;
    }

    if (!runtime_imu_is_active(imu_sample, now_s))
    {
        /* 방향을 모르는 이동을 임의 방향으로 넣는 것보다 이 sample을 버리는 편이 안전하다. */
        fusion->imu_was_stale = true;
        step->result = POSITION_SAMPLE_IMU_STALE;
        return;
    }

    step->imu_fresh = true;
    step->heading_deg = relative_heading_at_deg(
        &fusion->heading_tracker,
        imu_sample,
        now_s
    );

    if (fusion->imu_was_stale)
    {
        /* stale 동안 누적된 heading gap을 첫 복구 flow의 회전으로 오인하지 않는다. */
        fusion->last_flow_heading_deg = step->heading_deg;
        fusion->imu_was_stale = false;
    }

    step->delta_heading_deg = position_angle_delta_deg(
        step->heading_deg,
        fusion->last_flow_heading_deg
    );
    /* Burst가 sample 전후에 걸친 이동이라고 보고 두 heading의 midpoint를 사용한다. */
    step->heading_mid_deg = position_wrap_angle_deg(
        fusion->last_flow_heading_deg + step->delta_heading_deg * 0.5
    );
    fusion->last_flow_heading_deg = step->heading_deg;

    {
        double scale = fusion->scaler != 0.0
            ? fusion->height_cm / fusion->scaler
            : 0.0;
        double dx_body_cm = (double)dx_ticks * scale;
        double dy_body_cm = (double)dy_ticks * scale;
        double gyro_magnitude = hypot(
            hypot(imu_sample->gyro_x_rad_s, imu_sample->gyro_y_rad_s),
            imu_sample->gyro_z_rad_s
        );
        double yaw_rad;
        double cos_yaw;
        double sin_yaw;
        bool was_learning = fusion->yaw_compensator.learning;

        /* 보정기는 body-frame에서 가짜 회전 flow를 뺀 뒤 좌표 회전을 수행한다. */
        yaw_flow_compensator_apply(
            &fusion->yaw_compensator,
            dx_body_cm,
            dy_body_cm,
            step->delta_heading_deg * M_PI / 180.0,
            gyro_magnitude,
            &dx_body_cm,
            &dy_body_cm
        );

        yaw_rad = step->heading_mid_deg * M_PI / 180.0;
        cos_yaw = cos(yaw_rad);
        sin_yaw = sin(yaw_rad);
        /* 표준 2-D CCW rotation으로 body displacement를 world frame에 옮긴다. */
        step->dx_world_cm = dx_body_cm * cos_yaw - dy_body_cm * sin_yaw;
        step->dy_world_cm = dx_body_cm * sin_yaw + dy_body_cm * cos_yaw;

        /* x/y를 갱신하는 곳은 이 한 지점뿐이라 폐기 sample이 우회 적분되지 않는다. */
        fusion->x_cm += step->dx_world_cm;
        fusion->y_cm += step->dy_world_cm;
        step->result = was_learning
            ? POSITION_SAMPLE_CALIBRATION_DISCARDED
            : POSITION_SAMPLE_APPLIED;
    }
}

bool position_fusion_reset_origin(
    position_fusion_t *fusion,
    const position_imu_sample_t *imu_sample,
    double now_s
)
{
    if (fusion == NULL)
    {
        return false;
    }

    fusion->x_cm = 0.0;
    fusion->y_cm = 0.0;
    fusion->raw_ticks_x = 0;
    fusion->raw_ticks_y = 0;

    if (!fusion->fusion_enabled)
    {
        return true;
    }

    if (!orientation_is_fresh(imu_sample, now_s))
    {
        return false;
    }

    /* 좌표 원점과 heading 원점을 같은 fresh sample에서 함께 다시 잡는다. */
    relative_heading_reset(&fusion->heading_tracker, imu_sample);
    fusion->last_flow_heading_deg = 0.0;
    fusion->imu_was_stale = false;
    return true;
}

bool position_fusion_start_yaw_calibration(
    position_fusion_t *fusion,
    const position_imu_sample_t *imu_sample,
    double now_s
)
{
    bool fresh;

    if (fusion == NULL || !fusion->fusion_enabled)
    {
        return false;
    }

    /* learning을 먼저 표시해 이후 모든 PAA burst가 좌표 대신 학습 경로로 간다. */
    yaw_flow_compensator_start(&fusion->yaw_compensator);
    fresh = orientation_is_fresh(imu_sample, now_s);

    if (fresh)
    {
        /* 보정 시작 전 heading을 경계로 잡아 직전 정상 주행 회전을 끊는다. */
        fusion->last_flow_heading_deg = relative_heading_at_deg(
            &fusion->heading_tracker,
            imu_sample,
            now_s
        );
        fusion->imu_was_stale = false;
    }

    return fresh;
}

bool position_fusion_finish_yaw_calibration(
    position_fusion_t *fusion,
    const position_imu_sample_t *imu_sample,
    double now_s
)
{
    /* 표본이 부족하면 learning 상태를 유지하므로 더 회전한 뒤 다시 종료할 수 있다. */
    if (
        fusion == NULL
        || !fusion->fusion_enabled
        || !yaw_flow_compensator_finish(&fusion->yaw_compensator)
    )
    {
        return false;
    }

    if (orientation_is_fresh(imu_sample, now_s))
    {
        /* 보정 종료 시 fresh heading으로 다시 기준화해 보정 중 전체 회전을 끊는다. */
        fusion->last_flow_heading_deg = relative_heading_at_deg(
            &fusion->heading_tracker,
            imu_sample,
            now_s
        );
        fusion->imu_was_stale = false;
    }
    else
    {
        fusion->imu_was_stale = true;
    }

    return true;
}

double position_fusion_accuracy_low_total(
    const position_fusion_t *fusion,
    double now_s
)
{
    double total;

    if (fusion == NULL)
    {
        return 0.0;
    }

    total = fusion->accuracy_low_total_s;
    if (fusion->accuracy_was_low)
    {
        total += now_s - fusion->accuracy_low_started_s;
    }

    return total;
}
