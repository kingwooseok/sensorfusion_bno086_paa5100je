/*
 * witness_sensor.c의 heading gate와 IPC frame을 white-box 방식으로 검증한다.
 * main 이름만 바꾸어 같은 translation unit에 포함하므로 production이
 * 사용하는 순수 변환/전송 함수를 그대로 시험한다.
 */
#define main witness_sensor_application_main
#include "../witness_sensor.c"
#undef main

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static bool witness_test_close_enough(
    double actual,
    double expected,
    double tolerance
)
{
    return fabs(actual - expected) <= tolerance;
}

static bno086_snapshot_t witness_test_snapshot(
    double absolute_yaw_deg,
    uint64_t sequence,
    uint64_t timestamp_ns,
    uint64_t generation,
    uint8_t rotation_accuracy,
    uint8_t mag_accuracy
)
{
    bno086_snapshot_t snapshot;
    double half_yaw_rad = absolute_yaw_deg * M_PI / 360.0;

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.startup_ready = true;
    snapshot.orientation_sequence = sequence;
    snapshot.mag_sequence = sequence;
    snapshot.last_orientation_update_ns = timestamp_ns;
    snapshot.last_mag_update_ns = timestamp_ns;
    snapshot.bno_generation = generation;
    snapshot.quat_real = cos(half_yaw_rad);
    snapshot.quat_k = sin(half_yaw_rad);
    snapshot.rotation_accuracy = rotation_accuracy;
    snapshot.mag_accuracy = mag_accuracy;

    return snapshot;
}

static bool test_heading_normalization(void)
{
    return witness_testbed_config_is_valid()
        && witness_test_close_enough(
            witness_normalize_heading_deg(-1.0),
            359.0,
            1.0e-12
        )
        && witness_test_close_enough(
            witness_normalize_heading_deg(360.0),
            0.0,
            1.0e-12
        )
        && witness_test_close_enough(
            witness_normalize_heading_deg(721.0),
            1.0,
            1.0e-12
        );
}

static bool test_heading_and_payload_mapping(void)
{
    const uint64_t start_ns = UINT64_C(1000000000);
    witness_heading_state_t state;
    vehicle_state_t vehicle;
    bno086_snapshot_t snapshot;
    witness_heading_result_t result;

    witness_heading_state_initialize(&state);
    memset(&vehicle, 0, sizeof(vehicle));

    snapshot = witness_test_snapshot(37.0, 1U, start_ns, 7U, 3U, 3U);
    result = witness_heading_update(&state, &snapshot, start_ns, &vehicle);
    if (result != WITNESS_HEADING_REFERENCE_CAPTURED
        || !state.reference_valid
        || !witness_test_close_enough(
                vehicle.x_mm,
                WITNESS_FIXED_X_MM,
                1.0e-6
            )
        || !witness_test_close_enough(
                vehicle.y_mm,
                WITNESS_FIXED_Y_MM,
                1.0e-6
            )
        || !witness_test_close_enough(
                vehicle.heading_deg,
                witness_normalize_heading_deg(WITNESS_INITIAL_HEADING_DEG),
                1.0e-5
            )
        || vehicle.detection_time != (int64_t)start_ns)
    {
        return false;
    }

    snapshot = witness_test_snapshot(
        127.0,
        2U,
        start_ns + UINT64_C(10000000),
        7U,
        3U,
        3U
    );
    result = witness_heading_update(
        &state,
        &snapshot,
        snapshot.last_orientation_update_ns,
        &vehicle
    );

    return result == WITNESS_HEADING_AVAILABLE
        && witness_test_close_enough(
            vehicle.heading_deg,
            witness_normalize_heading_deg(
                WITNESS_INITIAL_HEADING_DEG
                + WITNESS_RELATIVE_HEADING_SIGN * 90.0
            ),
            1.0e-4
        )
        && vehicle.detection_time
           == (int64_t)snapshot.last_orientation_update_ns;
}

static bool test_accuracy_drop_does_not_invalidate(void)
{
    const uint64_t start_ns = UINT64_C(2000000000);
    witness_heading_state_t state;
    vehicle_state_t vehicle;
    bno086_snapshot_t snapshot;

    witness_heading_state_initialize(&state);
    snapshot = witness_test_snapshot(-170.0, 1U, start_ns, 10U, 2U, 2U);
    if (witness_heading_update(
            &state,
            &snapshot,
            start_ns,
            &vehicle
        ) != WITNESS_HEADING_REFERENCE_CAPTURED)
    {
        return false;
    }

    snapshot = witness_test_snapshot(
        -140.0,
        2U,
        start_ns + UINT64_C(10000000),
        10U,
        0U,
        0U
    );

    return witness_heading_update(
            &state,
            &snapshot,
            snapshot.last_orientation_update_ns,
            &vehicle
        ) == WITNESS_HEADING_AVAILABLE
        && state.reference_valid
        && witness_test_close_enough(
            vehicle.heading_deg,
            witness_normalize_heading_deg(
                WITNESS_INITIAL_HEADING_DEG
                + WITNESS_RELATIVE_HEADING_SIGN * 30.0
            ),
            1.0e-4
        );
}

static bool test_stale_recovery_requires_accuracy(void)
{
    const uint64_t start_ns = UINT64_C(3000000000);
    witness_heading_state_t state;
    vehicle_state_t vehicle;
    bno086_snapshot_t snapshot;

    witness_heading_state_initialize(&state);
    snapshot = witness_test_snapshot(10.0, 1U, start_ns, 15U, 3U, 3U);
    if (witness_heading_update(
            &state,
            &snapshot,
            start_ns,
            &vehicle
        ) != WITNESS_HEADING_REFERENCE_CAPTURED)
    {
        return false;
    }

    if (witness_heading_update(
            &state,
            &snapshot,
            start_ns + WITNESS_MAG_STALE_NS + 1U,
            &vehicle
        ) != WITNESS_HEADING_UNAVAILABLE
        || state.reference_valid)
    {
        return false;
    }

    snapshot = witness_test_snapshot(50.0, 2U, start_ns + UINT64_C(600000000),
                                     15U, 1U, 1U);
    if (witness_heading_update(
            &state,
            &snapshot,
            snapshot.last_orientation_update_ns,
            &vehicle
        ) != WITNESS_HEADING_UNAVAILABLE
        || state.reference_valid)
    {
        return false;
    }

    snapshot.rotation_accuracy = 2U;
    snapshot.mag_accuracy = 2U;
    return witness_heading_update(
            &state,
            &snapshot,
            snapshot.last_orientation_update_ns,
            &vehicle
        ) == WITNESS_HEADING_REFERENCE_CAPTURED
        && state.reference_valid
        && witness_test_close_enough(
            vehicle.heading_deg,
            witness_normalize_heading_deg(WITNESS_INITIAL_HEADING_DEG),
            1.0e-5
        );
}

static bool test_generation_recovery_requires_new_reference(void)
{
    const uint64_t start_ns = UINT64_C(4000000000);
    witness_heading_state_t state;
    vehicle_state_t vehicle;
    bno086_snapshot_t snapshot;

    witness_heading_state_initialize(&state);
    snapshot = witness_test_snapshot(0.0, 1U, start_ns, 20U, 3U, 3U);
    if (witness_heading_update(
            &state,
            &snapshot,
            start_ns,
            &vehicle
        ) != WITNESS_HEADING_REFERENCE_CAPTURED)
    {
        return false;
    }

    snapshot = witness_test_snapshot(
        80.0,
        2U,
        start_ns + UINT64_C(10000000),
        21U,
        3U,
        3U
    );
    if (witness_heading_update(
            &state,
            &snapshot,
            snapshot.last_orientation_update_ns,
            &vehicle
        ) != WITNESS_HEADING_UNAVAILABLE
        || state.reference_valid)
    {
        return false;
    }

    return witness_heading_update(
            &state,
            &snapshot,
            snapshot.last_orientation_update_ns,
            &vehicle
        ) == WITNESS_HEADING_REFERENCE_CAPTURED
        && state.reference_generation == 21U
        && witness_test_close_enough(
            vehicle.heading_deg,
            witness_normalize_heading_deg(WITNESS_INITIAL_HEADING_DEG),
            1.0e-5
        );
}

static bool test_startup_loss_requires_new_reference(void)
{
    const uint64_t start_ns = UINT64_C(5000000000);
    witness_heading_state_t state;
    vehicle_state_t vehicle;
    bno086_snapshot_t snapshot;

    witness_heading_state_initialize(&state);
    snapshot = witness_test_snapshot(25.0, 1U, start_ns, 30U, 3U, 3U);
    if (witness_heading_update(
            &state,
            &snapshot,
            start_ns,
            &vehicle
        ) != WITNESS_HEADING_REFERENCE_CAPTURED)
    {
        return false;
    }

    snapshot.startup_ready = false;
    if (witness_heading_update(
            &state,
            &snapshot,
            start_ns,
            &vehicle
        ) != WITNESS_HEADING_UNAVAILABLE
        || state.reference_valid)
    {
        return false;
    }

    snapshot = witness_test_snapshot(
        70.0,
        2U,
        start_ns + UINT64_C(10000000),
        31U,
        1U,
        1U
    );
    if (witness_heading_update(
            &state,
            &snapshot,
            snapshot.last_orientation_update_ns,
            &vehicle
        ) != WITNESS_HEADING_UNAVAILABLE)
    {
        return false;
    }

    snapshot.rotation_accuracy = 2U;
    snapshot.mag_accuracy = 2U;
    return witness_heading_update(
            &state,
            &snapshot,
            snapshot.last_orientation_update_ns,
            &vehicle
        ) == WITNESS_HEADING_REFERENCE_CAPTURED
        && state.reference_valid;
}

static bool test_ipc_frame_and_sequence(void)
{
    int sockets[2] = {-1, -1};
    vehicle_state_t sent;
    vehicle_state_t received;
    ipc_header_t header;
    uint32_t next_sequence = 41U;
    bool passed = false;

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) != 0)
    {
        return false;
    }

    memset(&sent, 0, sizeof(sent));
    sent.detection_time = INT64_C(5500000000);
    sent.x_mm = (float)WITNESS_FIXED_X_MM;
    sent.y_mm = (float)WITNESS_FIXED_Y_MM;
    sent.heading_deg = 123.5f;

    if (witness_send_state(sockets[0], &sent, &next_sequence) == IPC_SUCCESS
        && next_sequence == 42U
        && ipc_recv(
                sockets[1],
                &header,
                &received,
                sizeof(received)
            ) == IPC_SUCCESS
        && header.type == IPC_MSG_SENSOR_DATA
        && header.flags == IPC_FLAG_NONE
        && header.length == sizeof(vehicle_state_t)
        && header.sequence == 41U
        && header.timestamp == (uint64_t)sent.detection_time
        && memcmp(&sent, &received, sizeof(sent)) == 0)
    {
        passed = true;
    }

    if (witness_send_state(-1, &sent, &next_sequence) == IPC_SUCCESS
        || next_sequence != 42U)
    {
        passed = false;
    }

    ipc_close(sockets[0]);
    ipc_close(sockets[1]);
    return passed;
}

int main(void)
{
    struct
    {
        const char *name;
        bool (*run)(void);
    } tests[] = {
        {"heading [0,360) normalization", test_heading_normalization},
        {"heading/payload mapping", test_heading_and_payload_mapping},
        {"accuracy dip keeps reference", test_accuracy_drop_does_not_invalidate},
        {"stale recovery accuracy gate", test_stale_recovery_requires_accuracy},
        {"generation recovery re-reference", test_generation_recovery_requires_new_reference},
        {"startup recovery re-reference", test_startup_loss_requires_new_reference},
        {"IPC frame/sequence", test_ipc_frame_and_sequence}
    };
    size_t passed = 0U;
    size_t index;

    signal(SIGPIPE, SIG_IGN);
    puts("=== WITNESS SENSOR C TEST ===");

    for (index = 0U; index < sizeof(tests) / sizeof(tests[0]); index++)
    {
        bool test_passed = tests[index].run();

        printf("%-34s : %s\n", tests[index].name,
               test_passed ? "PASS" : "FAIL");
        if (test_passed)
        {
            passed++;
        }
    }

    printf("RESULT=%zu/%zu %s\n", passed,
           sizeof(tests) / sizeof(tests[0]),
           passed == sizeof(tests) / sizeof(tests[0]) ? "PASS" : "FAIL");

    return passed == sizeof(tests) / sizeof(tests[0])
        ? EXIT_SUCCESS
        : EXIT_FAILURE;
}
