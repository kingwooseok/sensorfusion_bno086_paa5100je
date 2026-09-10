#include "position_fusion.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef bool (*test_function_t)(void);

static bool close_enough(double actual, double expected, double tolerance)
{
    return fabs(actual - expected) <= tolerance;
}

static position_imu_sample_t make_sample(
    double absolute_yaw_deg,
    uint64_t sequence,
    double timestamp_s
)
{
    position_imu_sample_t sample;
    double half_yaw = absolute_yaw_deg * M_PI / 360.0;

    memset(&sample, 0, sizeof(sample));
    sample.startup_ready = true;
    sample.orientation_sequence = sequence;
    sample.gyro_sequence = sequence;
    sample.mag_sequence = sequence;
    sample.orientation_time_s = timestamp_s;
    sample.gyro_time_s = timestamp_s;
    sample.mag_time_s = timestamp_s;
    sample.quaternion.real = cos(half_yaw);
    sample.quaternion.k = sin(half_yaw);
    sample.rotation_accuracy = 3U;
    sample.mag_accuracy = 3U;
    return sample;
}

static position_quaternion_t make_axis_rotation(
    double axis_x,
    double axis_y,
    double axis_z,
    double angle_deg
)
{
    position_quaternion_t quaternion;
    double half_angle = angle_deg * M_PI / 360.0;
    double sine = sin(half_angle);

    quaternion.real = cos(half_angle);
    quaternion.i = axis_x * sine;
    quaternion.j = axis_y * sine;
    quaternion.k = axis_z * sine;
    return quaternion;
}

static position_imu_sample_t make_quaternion_sample(
    position_quaternion_t quaternion,
    uint64_t sequence,
    double timestamp_s
)
{
    position_imu_sample_t sample = make_sample(0.0, sequence, timestamp_s);

    sample.quaternion = quaternion;
    return sample;
}

static void make_stale(position_imu_sample_t *sample, double now_s)
{
    sample->orientation_time_s = now_s - 1.0;
    sample->gyro_time_s = now_s - 1.0;
    sample->mag_time_s = now_s - 1.0;
}

static void process(
    position_fusion_t *fusion,
    position_imu_sample_t *sample,
    double relative_yaw_deg,
    double initial_yaw_deg,
    uint64_t sequence,
    double now_s,
    int16_t dx,
    int16_t dy,
    uint8_t squal,
    bool fresh,
    bool startup_ready
)
{
    position_step_t step;

    *sample = make_sample(initial_yaw_deg + relative_yaw_deg, sequence, now_s);
    sample->startup_ready = startup_ready;
    if (!fresh)
    {
        make_stale(sample, now_s);
    }
    position_fusion_process(fusion, dx, dy, squal, sample, now_s, &step);
}

static bool test_angle_wrap(void)
{
    return close_enough(position_angle_delta_deg(-179.0, 179.0), 2.0, 1.0e-12);
}

static bool test_relative_quaternion_offsets(void)
{
    const double starts[] = {0.0, 37.0, 123.0, -170.0};
    size_t index;

    for (index = 0U; index < sizeof(starts) / sizeof(starts[0]); index++)
    {
        position_imu_sample_t reference = make_sample(starts[index], 1U, 1.0);
        position_imu_sample_t current = make_sample(starts[index] + 90.0, 2U, 2.0);
        relative_heading_tracker_t tracker;
        double heading;

        relative_heading_reset(&tracker, &reference);
        heading = relative_heading_at_deg(&tracker, &current, 2.0);
        if (!close_enough(heading, 90.0, 1.0e-9))
        {
            return false;
        }
    }

    return true;
}

static bool test_fixed_mount_tilt_cancellation(void)
{
    const struct
    {
        double roll_deg;
        double pitch_deg;
    } mounts[] = {
        {30.0, 0.0},
        {0.0, 30.0},
        {60.0, 35.0},
        {90.0, 0.0},
        {0.0, 90.0}
    };
    const position_quaternion_t initial_yaw =
        make_axis_rotation(0.0, 0.0, 1.0, 37.0);
    const position_quaternion_t vehicle_turn =
        make_axis_rotation(0.0, 0.0, 1.0, 90.0);
    size_t index;

    for (index = 0U; index < sizeof(mounts) / sizeof(mounts[0]); index++)
    {
        position_quaternion_t mount_roll = make_axis_rotation(
            1.0,
            0.0,
            0.0,
            mounts[index].roll_deg
        );
        position_quaternion_t mount_pitch = make_axis_rotation(
            0.0,
            1.0,
            0.0,
            mounts[index].pitch_deg
        );
        position_quaternion_t mount = position_quaternion_multiply(
            mount_pitch,
            mount_roll
        );
        position_quaternion_t reference_quaternion =
            position_quaternion_multiply(initial_yaw, mount);
        position_quaternion_t current_quaternion =
            position_quaternion_multiply(
                vehicle_turn,
                reference_quaternion
            );
        position_imu_sample_t reference = make_quaternion_sample(
            reference_quaternion,
            1U,
            1.0
        );
        position_imu_sample_t current = make_quaternion_sample(
            current_quaternion,
            2U,
            2.0
        );
        relative_heading_tracker_t tracker;
        double heading;

        relative_heading_reset(&tracker, &reference);
        heading = relative_heading_raw_deg(&tracker, &current);
        if (!close_enough(heading, 90.0, 1.0e-9))
        {
            return false;
        }
    }

    return true;
}

static bool test_square_closure(void)
{
    const double starts[] = {0.0, 37.0, 123.0, -170.0};
    size_t start_index;

    for (start_index = 0U;
         start_index < sizeof(starts) / sizeof(starts[0]);
         start_index++)
    {
        position_imu_sample_t reference = make_sample(starts[start_index], 1U, 1.0);
        position_imu_sample_t sample;
        position_fusion_t fusion;
        uint64_t sequence = 2U;
        double now_s = 1.01;
        int side;

        position_fusion_init(&fusion, 1.0, 1.0, &reference);

        for (side = 0; side < 4; side++)
        {
            double yaw = 90.0 * side;
            process(
                &fusion, &sample, yaw, starts[start_index], sequence++, now_s,
                100, 0, 255U, true, true
            );
            now_s += 0.01;
            if (side < 3)
            {
                process(
                    &fusion, &sample, yaw + 90.0, starts[start_index],
                    sequence++, now_s, 0, 0, 255U, true, true
                );
                now_s += 0.01;
            }
        }

        if (hypot(fusion.x_cm, fusion.y_cm) > 1.0e-6)
        {
            return false;
        }
    }

    return true;
}

static bool test_standalone_filters_scale(void)
{
    position_fusion_t fusion;
    position_step_t step;

    position_fusion_init(&fusion, 2.0, 4.0, NULL);
    position_fusion_process(&fusion, 10, 0, 24U, NULL, 1.0, &step);
    position_fusion_process(&fusion, 1, -1, 255U, NULL, 1.01, &step);
    position_fusion_process(&fusion, 4, -4, 25U, NULL, 1.02, &step);

    return
        close_enough(fusion.x_cm, 2.0, 1.0e-12)
        && close_enough(fusion.y_cm, -2.0, 1.0e-12)
        && fusion.raw_ticks_x == 15
        && fusion.raw_ticks_y == -5;
}

static bool test_origin_reset(void)
{
    position_fusion_t fusion;
    position_step_t step;

    position_fusion_init(&fusion, 1.0, 1.0, NULL);
    position_fusion_process(&fusion, 10, 5, 255U, NULL, 1.0, &step);
    if (!position_fusion_reset_origin(&fusion, NULL, 1.01))
    {
        return false;
    }
    position_fusion_process(&fusion, 4, -2, 255U, NULL, 1.02, &step);

    return
        close_enough(fusion.x_cm, 4.0, 1.0e-12)
        && close_enough(fusion.y_cm, -2.0, 1.0e-12)
        && fusion.raw_ticks_x == 4
        && fusion.raw_ticks_y == -2;
}

static bool test_absolute_yaw_invariant(void)
{
    const double starts[] = {0.0, 37.0, 123.0, -170.0};
    size_t index;

    for (index = 0U; index < sizeof(starts) / sizeof(starts[0]); index++)
    {
        position_imu_sample_t reference = make_sample(starts[index], 1U, 1.0);
        position_imu_sample_t sample;
        position_fusion_t fusion;

        position_fusion_init(&fusion, 1.0, 1.0, &reference);
        process(&fusion, &sample, 90.0, starts[index], 2U, 1.01,
                0, 0, 255U, true, true);
        process(&fusion, &sample, 90.0, starts[index], 3U, 1.02,
                10, 0, 255U, true, true);

        if (!close_enough(fusion.x_cm, 0.0, 1.0e-6)
            || !close_enough(fusion.y_cm, 10.0, 1.0e-6))
        {
            return false;
        }
    }

    return true;
}

static bool test_wrap_midpoint(void)
{
    position_imu_sample_t reference = make_sample(37.0, 1U, 1.0);
    position_imu_sample_t sample;
    position_fusion_t fusion;

    position_fusion_init(&fusion, 1.0, 1.0, &reference);
    process(&fusion, &sample, 179.0, 37.0, 2U, 1.01,
            0, 0, 255U, true, true);
    process(&fusion, &sample, 181.0, 37.0, 3U, 1.02,
            100, 0, 255U, true, true);

    return
        close_enough(fusion.x_cm, -100.0, 1.0e-6)
        && close_enough(fusion.y_cm, 0.0, 1.0e-6);
}

static bool test_low_squal_heading_sync(void)
{
    position_imu_sample_t reference = make_sample(0.0, 1U, 1.0);
    position_imu_sample_t sample;
    position_fusion_t fusion;

    position_fusion_init(&fusion, 1.0, 1.0, &reference);
    process(&fusion, &sample, 90.0, 0.0, 2U, 1.01,
            100, 0, 24U, true, true);
    process(&fusion, &sample, 90.0, 0.0, 3U, 1.02,
            10, 0, 255U, true, true);

    return
        close_enough(fusion.x_cm, 0.0, 1.0e-6)
        && close_enough(fusion.y_cm, 10.0, 1.0e-6);
}

static bool test_stale_discard_reanchor(void)
{
    position_imu_sample_t reference = make_sample(0.0, 1U, 1.0);
    position_imu_sample_t sample;
    position_fusion_t fusion;

    position_fusion_init(&fusion, 1.0, 1.0, &reference);
    process(&fusion, &sample, 0.0, 0.0, 2U, 1.01,
            10, 0, 255U, true, true);
    process(&fusion, &sample, 90.0, 0.0, 3U, 1.02,
            100, 0, 255U, false, true);
    process(&fusion, &sample, 90.0, 0.0, 4U, 1.03,
            10, 0, 255U, true, true);

    return
        close_enough(fusion.x_cm, 10.0, 1.0e-6)
        && close_enough(fusion.y_cm, 10.0, 1.0e-6);
}

static bool test_low_squal_stale_reanchor(void)
{
    position_imu_sample_t reference = make_sample(0.0, 1U, 1.0);
    position_imu_sample_t sample;
    position_fusion_t fusion;

    position_fusion_init(&fusion, 1.0, 1.0, &reference);
    process(&fusion, &sample, 0.0, 0.0, 2U, 1.01,
            10, 0, 255U, true, true);
    process(&fusion, &sample, 90.0, 0.0, 3U, 1.02,
            100, 0, 24U, false, true);
    process(&fusion, &sample, 90.0, 0.0, 4U, 1.03,
            10, 0, 255U, true, true);

    return
        close_enough(fusion.x_cm, 10.0, 1.0e-6)
        && close_enough(fusion.y_cm, 10.0, 1.0e-6);
}

static bool test_low_squal_recovery_boundary(void)
{
    position_imu_sample_t reference = make_sample(0.0, 1U, 1.0);
    position_imu_sample_t sample;
    position_fusion_t fusion;
    double expected_x = 10.0 + 10.0 * cos(95.0 * M_PI / 180.0);
    double expected_y = 10.0 * sin(95.0 * M_PI / 180.0);

    position_fusion_init(&fusion, 1.0, 1.0, &reference);
    process(&fusion, &sample, 0.0, 0.0, 2U, 1.01,
            10, 0, 255U, true, true);
    process(&fusion, &sample, 45.0, 0.0, 3U, 1.02,
            100, 0, 24U, false, true);
    process(&fusion, &sample, 90.0, 0.0, 4U, 1.03,
            100, 0, 24U, true, true);
    process(&fusion, &sample, 100.0, 0.0, 5U, 1.04,
            10, 0, 255U, true, true);

    return
        close_enough(fusion.x_cm, expected_x, 1.0e-6)
        && close_enough(fusion.y_cm, expected_y, 1.0e-6);
}

static bool test_startup_loss_reanchor(void)
{
    position_imu_sample_t reference = make_sample(0.0, 1U, 1.0);
    position_imu_sample_t sample;
    position_fusion_t fusion;

    position_fusion_init(&fusion, 1.0, 1.0, &reference);
    process(&fusion, &sample, 0.0, 0.0, 2U, 1.01,
            10, 0, 255U, true, true);
    process(&fusion, &sample, 90.0, 0.0, 3U, 1.02,
            100, 0, 255U, true, false);
    process(&fusion, &sample, 90.0, 0.0, 4U, 1.03,
            10, 0, 255U, true, true);

    return
        close_enough(fusion.x_cm, 10.0, 1.0e-6)
        && close_enough(fusion.y_cm, 10.0, 1.0e-6);
}

static bool test_accuracy_drop_continuity(void)
{
    position_imu_sample_t reference = make_sample(0.0, 1U, 1.0);
    position_imu_sample_t sample;
    position_fusion_t fusion;
    position_step_t step;
    double expected_x = 10.0 * cos(95.0 * M_PI / 180.0);
    double expected_y = 10.0 + 10.0 * sin(95.0 * M_PI / 180.0);

    position_fusion_init(&fusion, 1.0, 1.0, &reference);

    sample = make_sample(90.0, 2U, 1.01);
    sample.rotation_accuracy = 1U;
    sample.mag_accuracy = 1U;
    position_fusion_process(&fusion, 0, 0, 255U, &sample, 1.01, &step);

    sample = make_sample(90.0, 3U, 1.02);
    sample.rotation_accuracy = 1U;
    sample.mag_accuracy = 1U;
    position_fusion_process(&fusion, 10, 0, 255U, &sample, 1.02, &step);

    sample = make_sample(100.0, 4U, 1.03);
    position_fusion_process(&fusion, 10, 0, 255U, &sample, 1.03, &step);

    return
        close_enough(fusion.x_cm, expected_x, 1.0e-6)
        && close_enough(fusion.y_cm, expected_y, 1.0e-6)
        && fusion.accuracy_low_events == 1U;
}

static bool test_calibration_gate_and_reference(void)
{
    position_imu_sample_t sample = make_sample(0.0, 1U, 1.0);
    position_imu_sample_t post_enter;
    position_fusion_t fusion;
    position_step_t step;

    sample.rotation_accuracy = 0U;
    sample.mag_accuracy = 0U;
    if (position_fusion_accuracy_is_ready(&sample))
    {
        return false;
    }

    sample.rotation_accuracy = 2U;
    sample.mag_accuracy = 2U;
    if (!position_fusion_reports_are_fresh(&sample, 1.0)
        || !position_fusion_accuracy_is_ready(&sample))
    {
        return false;
    }

    post_enter = make_sample(90.0, 2U, 1.01);
    position_fusion_init(&fusion, 1.0, 1.0, &post_enter);
    post_enter = make_sample(90.0, 3U, 1.02);
    position_fusion_process(&fusion, 10, 0, 255U, &post_enter, 1.02, &step);

    return
        close_enough(fusion.x_cm, 10.0, 1.0e-6)
        && close_enough(fusion.y_cm, 0.0, 1.0e-6);
}

static bool train_compensator(yaw_flow_compensator_t *compensator)
{
    uint32_t index;

    yaw_flow_compensator_init(compensator);
    yaw_flow_compensator_start(compensator);
    for (index = 0U; index < 20U; index++)
    {
        double sign = index % 2U == 0U ? 1.0 : -1.0;
        double corrected_x;
        double corrected_y;

        yaw_flow_compensator_apply(
            compensator,
            sign * 2.0,
            0.0,
            sign * 0.2,
            1.0,
            &corrected_x,
            &corrected_y
        );
        if (corrected_x != 0.0 || corrected_y != 0.0)
        {
            return false;
        }
    }

    return yaw_flow_compensator_finish(compensator);
}

static bool test_auto_yaw_learning_freeze(void)
{
    yaw_flow_compensator_t normal;
    yaw_flow_compensator_t one_direction;
    yaw_flow_compensator_t frozen;
    double corrected_x;
    double corrected_y;
    uint32_t index;
    uint32_t frozen_count;
    double frozen_value;

    yaw_flow_compensator_init(&normal);
    yaw_flow_compensator_apply(
        &normal, 0.1, 0.0, M_PI / 180.0, 1.0,
        &corrected_x, &corrected_y
    );
    if (!close_enough(corrected_x, 0.1, 1.0e-12)
        || normal.sample_count != 0U)
    {
        return false;
    }

    yaw_flow_compensator_init(&one_direction);
    yaw_flow_compensator_start(&one_direction);
    for (index = 0U; index < 8U; index++)
    {
        yaw_flow_compensator_apply(
            &one_direction, 2.0, 0.0, 0.2, 1.0,
            &corrected_x, &corrected_y
        );
    }
    if (yaw_flow_compensator_finish(&one_direction))
    {
        return false;
    }

    if (!train_compensator(&frozen))
    {
        return false;
    }
    frozen_count = frozen.sample_count;
    frozen_value = frozen.x_per_rad_cm;
    yaw_flow_compensator_apply(
        &frozen, 1.0, 0.0, 0.1, 1.0,
        &corrected_x, &corrected_y
    );

    return
        close_enough(corrected_x, 0.0, 1.0e-6)
        && frozen.sample_count == frozen_count
        && frozen.x_per_rad_cm == frozen_value;
}

static bool test_lever_arm_production(void)
{
    position_imu_sample_t reference = make_sample(0.0, 1U, 1.0);
    position_imu_sample_t sample;
    position_fusion_t fusion;
    yaw_flow_compensator_t compensator;
    double yaw = 0.0;
    uint32_t index;

    if (!train_compensator(&compensator))
    {
        return false;
    }

    position_fusion_init(&fusion, 1.0, 1.0, &reference);
    fusion.yaw_compensator = compensator;

    for (index = 0U; index < 20U; index++)
    {
        double sign = index % 2U == 0U ? 1.0 : -1.0;
        yaw += sign * 0.2 * 180.0 / M_PI;
        sample = make_sample(yaw, index + 2U, 1.01 + index * 0.01);
        sample.gyro_z_rad_s = 1.0;
        {
            position_step_t step;
            position_fusion_process(
                &fusion,
                sign > 0.0 ? 2 : -2,
                0,
                255U,
                &sample,
                sample.orientation_time_s,
                &step
            );
        }
    }

    return hypot(fusion.x_cm, fusion.y_cm) < 1.0e-6;
}

static bool test_calibration_exit_boundary(void)
{
    const double deltas[] = {45.0, 45.0, -30.0, -30.0,
                             45.0, 45.0, -30.0, -30.0};
    position_imu_sample_t reference = make_sample(0.0, 1U, 1.0);
    position_imu_sample_t sample;
    position_fusion_t fusion;
    position_step_t step;
    double yaw = 0.0;
    size_t index;

    position_fusion_init(&fusion, 1.0, 1.0, &reference);
    sample = make_sample(0.0, 2U, 1.01);
    position_fusion_process(&fusion, 10, 0, 255U, &sample, 1.01, &step);

    if (!position_fusion_start_yaw_calibration(&fusion, &sample, 1.01))
    {
        return false;
    }

    for (index = 0U; index < sizeof(deltas) / sizeof(deltas[0]); index++)
    {
        yaw += deltas[index];
        sample = make_sample(yaw, index + 3U, 1.02 + (double)index * 0.01);
        sample.gyro_z_rad_s = 1.0;
        position_fusion_process(
            &fusion,
            deltas[index] > 0.0 ? 4 : -3,
            0,
            255U,
            &sample,
            sample.orientation_time_s,
            &step
        );
        if (step.result != POSITION_SAMPLE_CALIBRATION_DISCARDED)
        {
            return false;
        }
    }

    sample = make_sample(90.0, 20U, 1.20);
    sample.gyro_z_rad_s = 1.0;
    if (!position_fusion_finish_yaw_calibration(&fusion, &sample, 1.20))
    {
        return false;
    }

    sample = make_sample(90.0, 21U, 1.21);
    position_fusion_process(&fusion, 10, 0, 255U, &sample, 1.21, &step);

    return
        close_enough(fusion.x_cm, 10.0, 1.0e-6)
        && close_enough(fusion.y_cm, 10.0, 1.0e-6)
        && fusion.yaw_compensator.sample_count == 8U
        && fusion.raw_ticks_x == 24;
}

int main(void)
{
    const struct
    {
        const char *name;
        test_function_t function;
    } tests[] = {
        {"angle wrap", test_angle_wrap},
        {"relative quaternion offsets", test_relative_quaternion_offsets},
        {"fixed roll/pitch mount cancellation", test_fixed_mount_tilt_cancellation},
        {"square closure", test_square_closure},
        {"standalone SQUAL/deadband/scale", test_standalone_filters_scale},
        {"runtime origin reset", test_origin_reset},
        {"absolute yaw invariant", test_absolute_yaw_invariant},
        {"heading wrap midpoint", test_wrap_midpoint},
        {"low-SQUAL heading sync", test_low_squal_heading_sync},
        {"IMU stale discard/re-anchor", test_stale_discard_reanchor},
        {"low-SQUAL + stale re-anchor", test_low_squal_stale_reanchor},
        {"low-SQUAL recovery boundary", test_low_squal_recovery_boundary},
        {"startup loss re-anchor", test_startup_loss_reanchor},
        {"runtime accuracy continuity", test_accuracy_drop_continuity},
        {"calibration gate/post-Enter reference", test_calibration_gate_and_reference},
        {"AutoYaw learning/freeze", test_auto_yaw_learning_freeze},
        {"lever-arm production", test_lever_arm_production},
        {"lever calibration exit boundary", test_calibration_exit_boundary},
    };
    size_t index;
    size_t passed = 0U;

    printf("=== C POSITION FUSION TEST ===\n");
    for (index = 0U; index < sizeof(tests) / sizeof(tests[0]); index++)
    {
        bool ok = tests[index].function();
        printf("%-42s : %s\n", tests[index].name, ok ? "PASS" : "FAIL");
        if (ok)
        {
            passed++;
        }
    }

    printf("\npassed=%zu/%zu\n", passed, sizeof(tests) / sizeof(tests[0]));
    printf("FINAL RESULT=%s\n",
           passed == sizeof(tests) / sizeof(tests[0]) ? "PASS" : "FAIL");

    return passed == sizeof(tests) / sizeof(tests[0]) ? 0 : 1;
}
