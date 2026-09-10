#define _POSIX_C_SOURCE 200809L

#include "../bno086.h"

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static unsigned int passed_count;
static unsigned int failed_count;

static void check_result(int condition, const char *name)
{
    if (condition)
    {
        passed_count++;
        printf("[PASS] %s\n", name);
    }
    else
    {
        failed_count++;
        printf("[FAIL] %s\n", name);
    }
}

static int nearly_equal(double left, double right, double tolerance)
{
    return fabs(left - right) <= tolerance;
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void write_i16_le(uint8_t *data, int16_t value)
{
    write_u16_le(data, (uint16_t)value);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
    data[2] = (uint8_t)((value >> 16) & 0xFFu);
    data[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static void make_header(
    uint8_t header[4],
    uint16_t length,
    int continuation,
    uint8_t channel,
    uint8_t sequence
)
{
    uint16_t raw_length = continuation
                        ? (uint16_t)(length | 0x8000u)
                        : length;

    write_u16_le(header, raw_length);
    header[2] = channel;
    header[3] = sequence;
}

static size_t make_continuation(
    uint8_t *buffer,
    uint16_t total_length,
    uint8_t channel,
    uint8_t sequence,
    const uint8_t *cargo,
    size_t cargo_size
)
{
    make_header(buffer, total_length, 1, channel, sequence);
    if (cargo_size > 0u)
    {
        memcpy(buffer + 4u, cargo, cargo_size);
    }
    return cargo_size + 4u;
}

static int create_virtual_imu(bno086_t *imu)
{
    bno086_config_t config;

    memset(imu, 0, sizeof(*imu));
    bno086_default_config(&config);
    config.int_line = -1;
    config.reset_line = -1;

    return bno086_init(imu, &config) > 0
        && bno086_test_make_virtual_open(imu) > 0;
}

static void test_phase1_null_and_empty(void)
{
    bno086_t imu;
    bno086_test_packet_t packet;
    bno086_diagnostics_t diagnostics;
    uint8_t null_header[4] = {0u, 0u, 0u, 0u};
    uint8_t empty_header[4];
    bno086_test_read_step_t steps[1];
    int result;

    check_result(create_virtual_imu(&imu), "mock IMU create (null/empty)");

    bno086_test_transport_reset();
    steps[0] = (bno086_test_read_step_t){null_header, 4u, 0, 0u};
    bno086_test_set_read_script(steps, 1u);
    result = bno086_test_read_packet(&imu, &packet);
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    check_result(
        result == 0
            && diagnostics.io_error_count == 0u
            && bno086_test_read_call_count() == 1u,
        "Phase 1 null is ignored without error"
    );

    make_header(empty_header, 4u, 0, 2u, 7u);
    bno086_test_transport_reset();
    steps[0] = (bno086_test_read_step_t){empty_header, 4u, 0, 0u};
    bno086_test_set_read_script(steps, 1u);
    result = bno086_test_read_packet(&imu, &packet);
    check_result(
        result == 1
            && packet.total_length == 4u
            && packet.channel == 2u
            && packet.sequence == 7u
            && packet.cargo_length == 0u,
        "length 4 empty SHTP packet"
    );

    bno086_deinit(&imu);
}

static void test_phase2_null_retry(void)
{
    bno086_t imu;
    bno086_test_packet_t packet;
    bno086_diagnostics_t diagnostics;
    uint8_t phase1[4];
    uint8_t null_header[4] = {0u, 0u, 0u, 0u};
    uint8_t cargo[17];
    uint8_t continuation[21];
    bno086_test_read_step_t steps[3];
    size_t index;
    int result;

    check_result(create_virtual_imu(&imu), "mock IMU create (null retry)");
    for (index = 0u; index < sizeof(cargo); index++)
    {
        cargo[index] = (uint8_t)(index + 1u);
    }

    make_header(phase1, 21u, 0, 2u, 0x1Au);
    (void)make_continuation(
        continuation,
        21u,
        2u,
        0x1Bu,
        cargo,
        sizeof(cargo)
    );
    steps[0] = (bno086_test_read_step_t){phase1, 4u, 0, 0u};
    steps[1] = (bno086_test_read_step_t){null_header, 4u, 0, 0u};
    steps[2] = (bno086_test_read_step_t){continuation, 21u, 0, 0u};

    bno086_test_transport_reset();
    bno086_test_set_read_script(steps, 3u);
    result = bno086_test_read_packet(&imu, &packet);
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    check_result(
        result == 1
            && packet.sequence == 0x1Bu
            && packet.cargo_length == sizeof(cargo)
            && memcmp(packet.cargo, cargo, sizeof(cargo)) == 0,
        "exact Phase 2 null retries once and restores cargo"
    );
    check_result(
        diagnostics.phase2_null_count == 1u
            && diagnostics.phase2_null_retry_success == 1u
            && diagnostics.phase2_null_retry_fail == 0u
            && diagnostics.shtp_validation_error_count == 0u,
        "null retry success counters"
    );

    bno086_deinit(&imu);
}

static void test_phase2_null_retry_failures(void)
{
    enum { CASE_COUNT = 5 };
    const char *names[CASE_COUNT] = {
        "double null fails",
        "null retry non-continuation fails",
        "null retry wrong channel fails",
        "null retry wrong sequence fails",
        "null retry wrong length fails"
    };
    unsigned int case_index;

    for (case_index = 0u; case_index < CASE_COUNT; case_index++)
    {
        bno086_t imu;
        bno086_test_packet_t packet;
        bno086_diagnostics_t diagnostics;
        uint8_t phase1[4];
        uint8_t first_null[4] = {0u, 0u, 0u, 0u};
        uint8_t retry[13] = {0u};
        uint8_t cargo[9] = {0u};
        bno086_test_read_step_t steps[3];
        uint16_t retry_length = 13u;
        int continuation = 1;
        uint8_t retry_channel = 2u;
        uint8_t retry_sequence = 0x21u;
        size_t retry_size = 13u;
        int result;

        check_result(create_virtual_imu(&imu), "mock IMU create (retry failure)");
        make_header(phase1, 13u, 0, 2u, 0x20u);

        if (case_index == 0u)
        {
            retry_size = 4u;
        }
        else
        {
            if (case_index == 1u)
            {
                continuation = 0;
            }
            else if (case_index == 2u)
            {
                retry_channel = 3u;
            }
            else if (case_index == 3u)
            {
                retry_sequence = 0x22u;
            }
            else
            {
                retry_length = 12u;
            }

            make_header(
                retry,
                retry_length,
                continuation,
                retry_channel,
                retry_sequence
            );
            memcpy(retry + 4u, cargo, sizeof(cargo));
        }

        steps[0] = (bno086_test_read_step_t){phase1, 4u, 0, 0u};
        steps[1] = (bno086_test_read_step_t){first_null, 4u, 0, 0u};
        steps[2] = (bno086_test_read_step_t){retry, retry_size, 0, 0u};

        bno086_test_transport_reset();
        bno086_test_set_read_script(steps, 3u);
        result = bno086_test_read_packet(&imu, &packet);
        (void)bno086_get_diagnostics(&imu, &diagnostics);
        check_result(
            result == 0
                && bno086_test_read_call_count() == 3u
                && diagnostics.phase2_null_count == 1u
                && diagnostics.phase2_null_retry_success == 0u
                && diagnostics.phase2_null_retry_fail == 1u
                && diagnostics.shtp_validation_error_count == 1u,
            names[case_index]
        );

        bno086_deinit(&imu);
    }
}

static void test_non_null_validation_has_no_retry(void)
{
    bno086_t imu;
    bno086_test_packet_t packet;
    bno086_diagnostics_t diagnostics;
    uint8_t phase1[4];
    uint8_t phase2[13] = {0u};
    bno086_test_read_step_t steps[2];
    int result;

    check_result(create_virtual_imu(&imu), "mock IMU create (normal validation)");
    make_header(phase1, 13u, 0, 2u, 9u);
    make_header(phase2, 13u, 0, 2u, 10u);
    steps[0] = (bno086_test_read_step_t){phase1, 4u, 0, 0u};
    steps[1] = (bno086_test_read_step_t){phase2, 13u, 0, 0u};

    bno086_test_transport_reset();
    bno086_test_set_read_script(steps, 2u);
    result = bno086_test_read_packet(&imu, &packet);
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    check_result(
        result == 0
            && bno086_test_read_call_count() == 2u
            && diagnostics.phase2_null_count == 0u
            && diagnostics.shtp_validation_error_count == 1u,
        "ordinary continuation validation error is never retried"
    );

    bno086_deinit(&imu);
}

static void test_sequence_wrap_and_additional_chunk(void)
{
    bno086_t imu;
    bno086_test_packet_t packet;
    uint8_t phase1[4];
    uint8_t phase2[32];
    uint8_t continuation[12];
    uint8_t cargo[36];
    bno086_test_read_step_t steps[3];
    size_t index;
    int result;

    check_result(create_virtual_imu(&imu), "mock IMU create (continuations)");
    for (index = 0u; index < sizeof(cargo); index++)
    {
        cargo[index] = (uint8_t)(0x80u + index);
    }

    make_header(phase1, 40u, 0, 3u, 0xFFu);
    (void)make_continuation(phase2, 40u, 3u, 0x00u, cargo, 28u);
    (void)make_continuation(
        continuation,
        12u,
        3u,
        0x01u,
        cargo + 28u,
        8u
    );
    steps[0] = (bno086_test_read_step_t){phase1, 4u, 0, 0u};
    steps[1] = (bno086_test_read_step_t){phase2, 32u, 0, 0u};
    steps[2] = (bno086_test_read_step_t){continuation, 12u, 0, 0u};

    bno086_test_transport_reset();
    bno086_test_set_read_script(steps, 3u);
    result = bno086_test_read_packet(&imu, &packet);
    check_result(
        result == 1
            && packet.sequence == 0x00u
            && packet.cargo_length == sizeof(cargo)
            && memcmp(packet.cargo, cargo, sizeof(cargo)) == 0,
        "0xFF -> 0x00 sequence and additional continuation assembly"
    );

    bno086_deinit(&imu);
}

static void test_transport_error_categories(void)
{
    bno086_t imu;
    bno086_test_packet_t packet;
    bno086_diagnostics_t diagnostics;
    uint8_t short_header[2] = {8u, 0u};
    uint8_t invalid_header[4];
    bno086_test_read_step_t step;
    int result;

    check_result(create_virtual_imu(&imu), "mock IMU create (error classes)");

    bno086_test_transport_reset();
    step = (bno086_test_read_step_t){NULL, 0u, EREMOTEIO, 0u};
    bno086_test_set_read_script(&step, 1u);
    result = bno086_test_read_packet(&imu, &packet);
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    check_result(
        result == 0
            && diagnostics.syscall_bus_error_count == 1u
            && diagnostics.shtp_validation_error_count == 0u
            && diagnostics.last_errno == EREMOTEIO,
        "OSError/errno is a syscall-bus error"
    );

    bno086_test_transport_reset();
    step = (bno086_test_read_step_t){short_header, 2u, 0, 0u};
    bno086_test_set_read_script(&step, 1u);
    result = bno086_test_read_packet(&imu, &packet);
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    check_result(
        result == 0 && diagnostics.syscall_bus_error_count == 2u,
        "short read is a syscall-bus error"
    );

    make_header(invalid_header, 285u, 0, 2u, 0u);
    bno086_test_transport_reset();
    step = (bno086_test_read_step_t){invalid_header, 4u, 0, 0u};
    bno086_test_set_read_script(&step, 1u);
    result = bno086_test_read_packet(&imu, &packet);
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    check_result(
        result == 0 && diagnostics.shtp_validation_error_count == 1u,
        "invalid SHTP length is a protocol validation error"
    );

    bno086_deinit(&imu);
}

static void make_fc_cargo(uint8_t cargo[9], uint8_t sensor_id, uint32_t interval)
{
    memset(cargo, 0, 9u);
    cargo[0] = 0xFCu;
    cargo[1] = sensor_id;
    write_u32_le(cargo + 5u, interval);
}

static void make_fc_read_pair(
    uint8_t phase1[4],
    uint8_t phase2[13],
    uint8_t sequence,
    uint8_t sensor_id,
    uint32_t interval
)
{
    uint8_t cargo[9];

    make_fc_cargo(cargo, sensor_id, interval);
    make_header(phase1, 13u, 0, 2u, sequence);
    (void)make_continuation(
        phase2,
        13u,
        2u,
        (uint8_t)(sequence + 1u),
        cargo,
        sizeof(cargo)
    );
}

static void run_feature_transition_case(
    const char *name,
    uint32_t requested,
    uint32_t first_interval,
    uint32_t second_interval
)
{
    bno086_t imu;
    bno086_diagnostics_t diagnostics;
    uint8_t first_phase1[4];
    uint8_t first_phase2[13];
    uint8_t second_phase1[4];
    uint8_t second_phase2[13];
    bno086_test_read_step_t steps[4];
    uint8_t write_buffer[32];
    uint32_t applied = 0u;
    size_t first_write_size;
    int result;

    check_result(create_virtual_imu(&imu), "mock IMU create (feature transition)");
    (void)bno086_test_set_startup_ready(&imu, true);
    make_fc_read_pair(
        first_phase1,
        first_phase2,
        1u,
        BNO086_SENSOR_ROTATION_VECTOR,
        first_interval
    );
    make_fc_read_pair(
        second_phase1,
        second_phase2,
        3u,
        BNO086_SENSOR_ROTATION_VECTOR,
        second_interval
    );
    steps[0] = (bno086_test_read_step_t){first_phase1, 4u, 0, 2u};
    steps[1] = (bno086_test_read_step_t){first_phase2, 13u, 0, 2u};
    steps[2] = (bno086_test_read_step_t){second_phase1, 4u, 0, 3u};
    steps[3] = (bno086_test_read_step_t){second_phase2, 13u, 0, 3u};

    bno086_test_transport_reset();
    bno086_test_set_read_script(steps, 4u);
    result = bno086_enable_feature(
        &imu,
        BNO086_SENSOR_ROTATION_VECTOR,
        requested,
        100u
    );
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    first_write_size = bno086_test_copy_write(
        0u,
        write_buffer,
        sizeof(write_buffer)
    );

    check_result(
        result == 1
            && bno086_get_feature_interval(
                   &imu,
                   BNO086_SENSOR_ROTATION_VECTOR,
                   &applied) == 1
            && applied == second_interval
            && bno086_test_write_count() == 3u
            && diagnostics.io_error_count == 0u,
        name
    );
    check_result(
        first_write_size == 21u
            && write_buffer[4] == 0xFDu
            && write_buffer[5] == BNO086_SENSOR_ROTATION_VECTOR
            && write_buffer[9] == (uint8_t)(requested & 0xFFu)
            && write_buffer[10] == (uint8_t)((requested >> 8) & 0xFFu)
            && write_buffer[11] == (uint8_t)((requested >> 16) & 0xFFu)
            && write_buffer[12] == (uint8_t)((requested >> 24) & 0xFFu),
        "Set Feature is the exact 17-byte FD payload"
    );

    bno086_deinit(&imu);
}

static void test_feature_race(void)
{
    bno086_t imu;
    bno086_snapshot_t snapshot;
    uint8_t phase1[4];
    uint8_t phase2[5];
    uint8_t reset_cargo[1] = {0x01u};
    bno086_test_read_step_t steps[2];
    int result;

    run_feature_transition_case(
        "enable ignores FC=0 and accepts adjusted non-zero interval",
        10000u,
        0u,
        12500u
    );
    run_feature_transition_case(
        "disable ignores non-zero FC and accepts FC=0",
        0u,
        10000u,
        0u
    );

    check_result(create_virtual_imu(&imu), "mock IMU create (feature reset)");
    (void)bno086_test_set_startup_ready(&imu, true);
    make_header(phase1, 5u, 0, 1u, 8u);
    (void)make_continuation(phase2, 5u, 1u, 9u, reset_cargo, 1u);
    steps[0] = (bno086_test_read_step_t){phase1, 4u, 0, 2u};
    steps[1] = (bno086_test_read_step_t){phase2, 5u, 0, 2u};

    bno086_test_transport_reset();
    bno086_test_set_read_script(steps, 2u);
    result = bno086_enable_feature(
        &imu,
        BNO086_SENSOR_ROTATION_VECTOR,
        10000u,
        100u
    );
    (void)bno086_get_snapshot(&imu, &snapshot);
    check_result(
        result == 0 && !snapshot.startup_ready,
        "feature setup fails immediately on reset-complete"
    );
    bno086_deinit(&imu);
}

static size_t append_vector_record(
    uint8_t *cargo,
    size_t cursor,
    uint8_t report_id,
    uint8_t sequence,
    uint8_t accuracy,
    int16_t x,
    int16_t y,
    int16_t z
)
{
    cargo[cursor] = report_id;
    cargo[cursor + 1u] = sequence;
    cargo[cursor + 2u] = accuracy;
    cargo[cursor + 3u] = 0u;
    write_i16_le(cargo + cursor + 4u, x);
    write_i16_le(cargo + cursor + 6u, y);
    write_i16_le(cargo + cursor + 8u, z);
    return cursor + 10u;
}

static void test_sensor_decode_and_spmc(void)
{
    bno086_t imu;
    bno086_snapshot_t snapshot;
    bno086_event_cursor_t position_cursor;
    bno086_event_cursor_t accident_cursor;
    bno086_event_t position_event;
    bno086_event_t accident_event;
    uint8_t cargo[64] = {0u};
    size_t cursor = 0u;
    unsigned int event_index;
    int updated;
    int cursors_match = 1;

    check_result(create_virtual_imu(&imu), "mock IMU create (decode/ring)");
    (void)bno086_event_cursor_init(&imu, &position_cursor, false);
    (void)bno086_event_cursor_init(&imu, &accident_cursor, false);

    cargo[cursor++] = 0xFBu;
    cargo[cursor++] = 0u;
    cargo[cursor++] = 0u;
    cargo[cursor++] = 0u;
    cargo[cursor++] = 0u;
    cursor = append_vector_record(
        cargo, cursor, BNO086_SENSOR_ACCELEROMETER, 1u, 3u,
        256, -512, 128
    );
    cursor = append_vector_record(
        cargo, cursor, BNO086_SENSOR_LINEAR_ACCELERATION, 2u, 2u,
        64, -128, 256
    );
    cursor = append_vector_record(
        cargo, cursor, BNO086_SENSOR_GYROSCOPE_CALIBRATED, 3u, 2u,
        512, -256, 128
    );
    cursor = append_vector_record(
        cargo, cursor, BNO086_SENSOR_MAGNETOMETER_CALIBRATED, 4u, 1u,
        16, -32, 8
    );

    cargo[cursor] = BNO086_SENSOR_ROTATION_VECTOR;
    cargo[cursor + 1u] = 5u;
    cargo[cursor + 2u] = 2u;
    cargo[cursor + 3u] = 0u;
    write_i16_le(cargo + cursor + 4u, 0);
    write_i16_le(cargo + cursor + 6u, 0);
    write_i16_le(cargo + cursor + 8u, 11585);
    write_i16_le(cargo + cursor + 10u, 11585);
    write_i16_le(cargo + cursor + 12u, 0);
    cursor += 14u;

    updated = bno086_test_feed_packet(&imu, 3u, 0u, cargo, cursor);
    (void)bno086_get_snapshot(&imu, &snapshot);
    check_result(
        updated == 5
            && nearly_equal(snapshot.accel_x_mps2, 1.0, 1e-9)
            && nearly_equal(snapshot.accel_y_mps2, -2.0, 1e-9)
            && nearly_equal(snapshot.linear_accel_z_mps2, 1.0, 1e-9)
            && nearly_equal(snapshot.gyro_x_rad_s, 1.0, 1e-9)
            && nearly_equal(snapshot.mag_y_ut, -2.0, 1e-9)
            && nearly_equal(snapshot.yaw_deg, 90.0, 0.02),
        "Q8/Q9/Q4/quaternion report decode"
    );

    for (event_index = 0u; event_index < 5u; event_index++)
    {
        if (bno086_event_next(
                &imu,
                &position_cursor,
                &position_event,
                0) != 1
            || bno086_event_next(
                &imu,
                &accident_cursor,
                &accident_event,
                0) != 1
            || position_event.ring_sequence != accident_event.ring_sequence
            || position_event.report_id != accident_event.report_id
            || position_event.x != accident_event.x
            || position_event.y != accident_event.y
            || position_event.z != accident_event.z)
        {
            cursors_match = 0;
        }
    }
    check_result(
        cursors_match
            && position_cursor.dropped == 0u
            && accident_cursor.dropped == 0u,
        "two independent SPMC cursors receive every same sample"
    );

    bno086_deinit(&imu);
}

static void test_ring_overrun(void)
{
    bno086_t imu;
    bno086_event_cursor_t slow_cursor;
    bno086_event_t event;
    bno086_diagnostics_t diagnostics;
    uint8_t cargo[10] = {0u};
    unsigned int index;

    check_result(create_virtual_imu(&imu), "mock IMU create (ring overrun)");
    (void)bno086_event_cursor_init(&imu, &slow_cursor, false);

    cargo[0] = BNO086_SENSOR_LINEAR_ACCELERATION;
    for (index = 0u; index < BNO086_EVENT_RING_CAPACITY + 2u; index++)
    {
        cargo[1] = (uint8_t)index;
        write_i16_le(cargo + 4u, (int16_t)index);
        (void)bno086_test_feed_packet(&imu, 3u, 0u, cargo, sizeof(cargo));
    }

    (void)bno086_event_next(&imu, &slow_cursor, &event, 0);
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    check_result(
        slow_cursor.dropped == 2u
            && event.ring_sequence == 3u
            && diagnostics.event_overwrite_count == 2u,
        "slow consumer detects overrun without blocking producer"
    );

    bno086_deinit(&imu);
}

int main(void)
{
    printf("=== BNO086 C MOCK TEST ===\n");

    test_phase1_null_and_empty();
    test_phase2_null_retry();
    test_phase2_null_retry_failures();
    test_non_null_validation_has_no_retry();
    test_sequence_wrap_and_additional_chunk();
    test_transport_error_categories();
    test_feature_race();
    test_sensor_decode_and_spmc();
    test_ring_overrun();

    printf("\npassed=%u\n", passed_count);
    printf("failed=%u\n", failed_count);
    printf(
        "CODE/MOCK RESULT=%s\n",
        failed_count == 0u ? "PASS" : "FAIL"
    );

    return failed_count == 0u ? 0 : 1;
}
