/*
 * accident_sensor.c의 실제 measurement orchestration을 white-box 방식으로
 * 검증한다. main 이름만 바꾸어 같은 translation unit에 포함하므로 이 시험이
 * 호출하는 measurement_* 함수는 production run_measurement()가 호출하는
 * 함수와 정확히 같다.
 */
#define main accident_sensor_application_main
#include "../accident_sensor.c"
#undef main

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct
{
    position_imu_sample_t next_sample;
    double now_s;

    unsigned int imu_read_count;
    unsigned int flush_count;
    int16_t pending_x;
    int16_t pending_y;
    int16_t flushed_x;
    int16_t flushed_y;
} measurement_mock_t;

static bool close_enough(double actual, double expected, double tolerance)
{
    return fabs(actual - expected) <= tolerance;
}

static position_imu_sample_t make_imu_sample(
    double absolute_yaw_deg,
    uint64_t sequence,
    double timestamp_s
)
{
    position_imu_sample_t sample;
    double half_yaw_rad = absolute_yaw_deg * M_PI / 360.0;

    memset(&sample, 0, sizeof(sample));
    sample.startup_ready = true;
    sample.orientation_sequence = sequence;
    sample.gyro_sequence = sequence;
    sample.mag_sequence = sequence;
    sample.orientation_time_s = timestamp_s;
    sample.gyro_time_s = timestamp_s;
    sample.mag_time_s = timestamp_s;
    sample.quaternion.real = cos(half_yaw_rad);
    sample.quaternion.k = sin(half_yaw_rad);
    sample.rotation_accuracy = 3U;
    sample.mag_accuracy = 3U;
    return sample;
}

static paa5100je_motion_t make_motion(int16_t dx, int16_t dy)
{
    paa5100je_motion_t motion;

    memset(&motion, 0, sizeof(motion));
    motion.motion_detected = 1;
    motion.delta_x_ticks = dx;
    motion.delta_y_ticks = dy;
    motion.squal = 100U;
    return motion;
}

static int mock_read_imu(
    void *context,
    position_imu_sample_t *sample
)
{
    measurement_mock_t *mock = context;

    mock->imu_read_count++;
    *sample = mock->next_sample;
    return 0;
}

static int mock_flush_flow(
    void *context,
    int16_t *delta_x_ticks,
    int16_t *delta_y_ticks
)
{
    measurement_mock_t *mock = context;

    mock->flush_count++;
    mock->flushed_x = mock->pending_x;
    mock->flushed_y = mock->pending_y;
    *delta_x_ticks = mock->pending_x;
    *delta_y_ticks = mock->pending_y;
    mock->pending_x = 0;
    mock->pending_y = 0;
    return 0;
}

static double mock_get_time(void *context)
{
    const measurement_mock_t *mock = context;

    return mock->now_s;
}

static bool test_calibration_command_boundary(void)
{
    static const double turn_delta_deg[] = {
        45.0, 45.0, -30.0, -30.0,
        45.0, 45.0, -30.0, -30.0
    };
    const double absolute_start_yaw_deg = 37.0;
    position_imu_sample_t reference = make_imu_sample(
        absolute_start_yaw_deg,
        1U,
        1.0
    );
    position_imu_sample_t sample;
    position_fusion_t fusion;
    position_step_t step;
    measurement_mock_t mock;
    measurement_runtime_ops_t ops;
    measurement_calibration_result_t command_result;
    paa5100je_motion_t motion;
    double relative_yaw_deg = 0.0;
    double x_before_calibration;
    double y_before_calibration;
    size_t index;

    memset(&mock, 0, sizeof(mock));
    ops.read_imu = mock_read_imu;
    ops.imu_context = &mock;
    ops.flush_flow = mock_flush_flow;
    ops.flow_context = &mock;
    ops.get_time_s = mock_get_time;
    ops.time_context = &mock;

    position_fusion_init(&fusion, 1.0, 1.0, &reference);

    /* 1. calibration 전 정상 직선 이동은 그대로 적분한다. */
    sample = make_imu_sample(absolute_start_yaw_deg, 2U, 1.01);
    motion = make_motion(10, 0);
    measurement_process_motion(&fusion, &motion, &sample, 1.01, &step);
    if (step.result != POSITION_SAMPLE_APPLIED
        || !close_enough(fusion.x_cm, 10.0, 1.0e-9)
        || !close_enough(fusion.y_cm, 0.0, 1.0e-9))
    {
        return false;
    }
    x_before_calibration = fusion.x_cm;
    y_before_calibration = fusion.y_cm;

    /* 2. production 'c' 시작 경로가 새로 읽은 fresh heading을 사용한다. */
    mock.now_s = 1.02;
    mock.next_sample = make_imu_sample(
        absolute_start_yaw_deg,
        3U,
        mock.now_s
    );
    if (measurement_toggle_yaw_calibration(
            &fusion,
            true,
            &ops,
            &command_result
        ) != 0
        || command_result.action != MEASUREMENT_CALIBRATION_STARTED
        || !command_result.fresh_heading_used
        || command_result.flow_flushed
        || !fusion.yaw_compensator.learning)
    {
        return false;
    }

    /* 3. 큰 CW/CCW 회전의 모든 PAA flow는 학습만 하고 좌표에서 제외한다. */
    for (index = 0U;
         index < sizeof(turn_delta_deg) / sizeof(turn_delta_deg[0]);
         index++)
    {
        int16_t dx;

        relative_yaw_deg += turn_delta_deg[index];
        mock.now_s = 1.03 + (double)index * 0.01;
        sample = make_imu_sample(
            absolute_start_yaw_deg + relative_yaw_deg,
            (uint64_t)index + 4U,
            mock.now_s
        );
        sample.gyro_z_rad_s = turn_delta_deg[index] > 0.0 ? 1.0 : -1.0;
        dx = turn_delta_deg[index] > 0.0 ? (int16_t)4 : (int16_t)-3;
        motion = make_motion(dx, 0);

        measurement_process_motion(
            &fusion,
            &motion,
            &sample,
            mock.now_s,
            &step
        );
        if (step.result != POSITION_SAMPLE_CALIBRATION_DISCARDED
            || !close_enough(fusion.x_cm, x_before_calibration, 1.0e-9)
            || !close_enough(fusion.y_cm, y_before_calibration, 1.0e-9))
        {
            return false;
        }
    }

    /* stale snapshot으로는 종료하지도, PAA 경계를 flush하지도 않는다. */
    mock.now_s = 2.0;
    mock.next_sample = make_imu_sample(
        absolute_start_yaw_deg + 90.0,
        20U,
        mock.now_s - 1.0
    );
    mock.pending_x = 777;
    mock.pending_y = -555;
    if (measurement_toggle_yaw_calibration(
            &fusion,
            true,
            &ops,
            &command_result
        ) != 0
        || command_result.action != MEASUREMENT_CALIBRATION_UNAVAILABLE
        || !fusion.yaw_compensator.learning
        || mock.flush_count != 0U)
    {
        return false;
    }

    /* 4. fresh 재조회로 종료하고 경계에 latch된 PAA delta를 폐기한다. */
    mock.next_sample = make_imu_sample(
        absolute_start_yaw_deg + 90.0,
        21U,
        mock.now_s
    );
    if (measurement_toggle_yaw_calibration(
            &fusion,
            true,
            &ops,
            &command_result
        ) != 0
        || command_result.action != MEASUREMENT_CALIBRATION_LOCKED
        || !command_result.fresh_heading_used
        || !command_result.flow_flushed
        || fusion.yaw_compensator.learning
        || !fusion.yaw_compensator.calibrated
        || mock.flush_count != 1U
        || mock.flushed_x != 777
        || mock.flushed_y != -555
        || mock.pending_x != 0
        || mock.pending_y != 0
        || fusion.imu_was_stale
        || !close_enough(fusion.last_flow_heading_deg, 90.0, 1.0e-9))
    {
        return false;
    }

    /* 5. 첫 정상 이동은 90도 현재 heading만 사용하고 midpoint 오염이 없다. */
    mock.now_s = 2.01;
    sample = make_imu_sample(
        absolute_start_yaw_deg + 90.0,
        22U,
        mock.now_s
    );
    motion = make_motion(10, 0);
    measurement_process_motion(
        &fusion,
        &motion,
        &sample,
        mock.now_s,
        &step
    );

    return
        mock.imu_read_count == 3U
        && step.result == POSITION_SAMPLE_APPLIED
        && close_enough(step.delta_heading_deg, 0.0, 1.0e-9)
        && close_enough(step.heading_mid_deg, 90.0, 1.0e-9)
        && close_enough(step.dx_world_cm, 0.0, 1.0e-9)
        && close_enough(step.dy_world_cm, 10.0, 1.0e-9)
        && close_enough(fusion.x_cm, 10.0, 1.0e-9)
        && close_enough(fusion.y_cm, 10.0, 1.0e-9);
}

static collision_config_t make_collision_config(void)
{
    collision_config_t config;

    memset(&config, 0, sizeof(config));
    config.enabled = true;
    config.metric_mask = COLLISION_METRIC_RAW_ACCEL
                       | COLLISION_METRIC_LINEAR_ACCEL;
    config.minimum_evidence_count = 2U;
    config.delta_v_window = COLLISION_DV_WINDOW_20_MS;
    config.coincidence_window_ms = 100U;
    config.cooldown_ms = 100U;
    config.quiet_rearm_ms = 50U;
    config.require_position = true;
    config.raw_accel_threshold_mps2 = 5.0;
    config.linear_accel_threshold_mps2 = 5.0;
    return config;
}

static bno086_event_t make_collision_event(
    bno086_event_type_t type,
    uint8_t sequence,
    uint64_t generation,
    uint64_t timestamp_ns,
    double x
)
{
    bno086_event_t event;

    memset(&event, 0, sizeof(event));
    event.type = type;
    event.report_sequence = sequence;
    event.bno_generation = generation;
    event.monotonic_ns = timestamp_ns;
    event.x = x;
    return event;
}

static bool test_detector_disabled_and_single_evidence(void)
{
    collision_config_t config = make_collision_config();
    collision_detector_state_t detector;
    collision_observation_t observation;
    bno086_event_t raw = make_collision_event(
        BNO086_EVENT_ACCELEROMETER,
        1U,
        1U,
        UINT64_C(1000000000),
        10.0
    );

    if (collision_config.enabled
        || collision_config.metric_mask != 0U
        || collision_config.raw_accel_threshold_mps2 != 0.0
        || collision_config.linear_accel_threshold_mps2 != 0.0
        || collision_config.raw_jerk_threshold_mps3 != 0.0
        || collision_config.linear_jerk_threshold_mps3 != 0.0
        || collision_config.linear_delta_v_threshold_mps != 0.0)
    {
        return false;
    }

    config.enabled = false;
    collision_detector_initialize(&detector, &config, 10000U, 10000U);
    collision_detector_set_armed(&detector, true);
    if (detector.armed
        || collision_detector_process(&detector, &raw, &observation))
    {
        return false;
    }

    config.enabled = true;
    collision_detector_initialize(&detector, &config, 10000U, 10000U);
    collision_detector_set_armed(&detector, false);
    if (collision_detector_process(&detector, &raw, &observation))
    {
        return false;
    }
    collision_detector_set_armed(&detector, true);
    return !collision_detector_process(&detector, &raw, &observation)
        && detector.evidence_ns[0] == raw.monotonic_ns
        && detector.evidence_ns[1] == 0U;
}

static bool test_detector_coincidence_latch_and_rearm(void)
{
    collision_config_t config = make_collision_config();
    collision_detector_state_t detector;
    collision_observation_t observation;
    bno086_event_t event;

    collision_detector_initialize(&detector, &config, 10000U, 10000U);
    collision_detector_set_armed(&detector, true);

    event = make_collision_event(
        BNO086_EVENT_ACCELEROMETER, 1U, 1U,
        UINT64_C(1000000000), 10.0
    );
    if (collision_detector_process(&detector, &event, &observation))
    {
        return false;
    }
    event = make_collision_event(
        BNO086_EVENT_LINEAR_ACCELERATION, 1U, 1U,
        UINT64_C(1050000000), 8.0
    );
    if (!collision_detector_process(&detector, &event, &observation))
    {
        return false;
    }

    /* latch 중 추가 peak는 두 번째 이벤트가 아니다. */
    event = make_collision_event(
        BNO086_EVENT_ACCELEROMETER, 2U, 1U,
        UINT64_C(1060000000), 11.0
    );
    if (collision_detector_process(&detector, &event, &observation))
    {
        return false;
    }

    /* cooldown과 quiet가 모두 지난 낮은 sample에서만 latch를 푼다. */
    event = make_collision_event(
        BNO086_EVENT_ACCELEROMETER, 3U, 1U,
        UINT64_C(1200000000), 0.0
    );
    if (collision_detector_process(&detector, &event, &observation)
        || detector.latched)
    {
        return false;
    }
    event = make_collision_event(
        BNO086_EVENT_ACCELEROMETER, 4U, 1U,
        UINT64_C(1210000000), 10.0
    );
    if (collision_detector_process(&detector, &event, &observation))
    {
        return false;
    }
    event = make_collision_event(
        BNO086_EVENT_LINEAR_ACCELERATION, 2U, 1U,
        UINT64_C(1220000000), 8.0
    );
    return collision_detector_process(&detector, &event, &observation);
}

static bool test_detector_gap_and_generation_reset(void)
{
    collision_config_t config = make_collision_config();
    collision_detector_state_t detector;
    collision_observation_t observation;
    bno086_event_t event;

    collision_detector_initialize(&detector, &config, 10000U, 10000U);
    collision_detector_set_armed(&detector, true);
    event = make_collision_event(
        BNO086_EVENT_LINEAR_ACCELERATION, 1U, 1U,
        UINT64_C(1000000000), 0.0
    );
    (void)collision_detector_process(&detector, &event, &observation);
    event = make_collision_event(
        BNO086_EVENT_ACCELEROMETER, 1U, 1U,
        UINT64_C(1010000000), 10.0
    );
    (void)collision_detector_process(&detector, &event, &observation);
    event = make_collision_event(
        BNO086_EVENT_LINEAR_ACCELERATION, 3U, 1U,
        UINT64_C(1020000000), 10.0
    );
    if (collision_detector_process(&detector, &event, &observation)
        || observation.linear_jerk_valid
        || detector.delta_v.count != 0U
        || detector.evidence_ns[0] != 0U)
    {
        return false;
    }

    collision_detector_set_armed(&detector, true);
    event = make_collision_event(
        BNO086_EVENT_LINEAR_ACCELERATION, 1U, 1U,
        UINT64_C(2000000000), 0.0
    );
    (void)collision_detector_process(&detector, &event, &observation);
    event = make_collision_event(
        BNO086_EVENT_ACCELEROMETER, 1U, 1U,
        UINT64_C(2010000000), 10.0
    );
    (void)collision_detector_process(&detector, &event, &observation);
    event = make_collision_event(
        BNO086_EVENT_LINEAR_ACCELERATION, 2U, 2U,
        UINT64_C(2020000000), 10.0
    );
    return !collision_detector_process(&detector, &event, &observation)
        && !observation.linear_jerk_valid
        && detector.delta_v.count == 0U
        && detector.evidence_ns[0] == 0U;
}

static bool test_discontinuity_sample_cannot_trigger(void)
{
    collision_config_t config = make_collision_config();
    collision_detector_state_t detector;
    collision_observation_t observation;
    bno086_event_t event;

    config.metric_mask = COLLISION_METRIC_RAW_ACCEL;
    config.minimum_evidence_count = 1U;
    collision_detector_initialize(&detector, &config, 10000U, 10000U);
    collision_detector_set_armed(&detector, true);

    event = make_collision_event(
        BNO086_EVENT_ACCELEROMETER, 1U, 1U,
        UINT64_C(1000000000), 0.0
    );
    (void)collision_detector_process(&detector, &event, &observation);
    event = make_collision_event(
        BNO086_EVENT_ACCELEROMETER, 3U, 1U,
        UINT64_C(1010000000), 10.0
    );
    if (collision_detector_process(&detector, &event, &observation)
        || detector.evidence_ns[0] != 0U)
    {
        return false;
    }
    event = make_collision_event(
        BNO086_EVENT_ACCELEROMETER, 4U, 1U,
        UINT64_C(1020000000), 10.0
    );
    return collision_detector_process(&detector, &event, &observation);
}

static bool test_startup_loss_and_calibration_rearm_policy(void)
{
    collision_config_t config = make_collision_config();
    collision_worker_t worker;
    collision_observation_t observation;
    bno086_event_t event;
    position_fusion_t fusion;

    memset(&worker, 0, sizeof(worker));
    collision_detector_initialize(
        &worker.detector,
        &config,
        10000U,
        10000U
    );
    worker.arm_requested = true;
    worker.startup_ready_seen = true;
    collision_detector_set_armed(&worker.detector, true);
    event = make_collision_event(
        BNO086_EVENT_ACCELEROMETER, 1U, 1U,
        UINT64_C(1000000000), 10.0
    );
    (void)collision_detector_process(&worker.detector, &event, &observation);
    if (worker.detector.evidence_ns[0] == 0U
        || collision_worker_apply_startup_locked(&worker, false)
        || worker.detector.armed
        || worker.detector.evidence_ns[0] != 0U)
    {
        return false;
    }
    if (!collision_worker_apply_startup_locked(&worker, true)
        || !worker.detector.armed)
    {
        return false;
    }
    event = make_collision_event(
        BNO086_EVENT_LINEAR_ACCELERATION, 1U, 2U,
        UINT64_C(1010000000), 10.0
    );
    if (collision_detector_process(&worker.detector, &event, &observation))
    {
        return false;
    }

    memset(&fusion, 0, sizeof(fusion));
    if (!measurement_collision_rearm_allowed(false, &fusion)
        || !measurement_collision_rearm_allowed(true, &fusion))
    {
        return false;
    }
    fusion.yaw_compensator.learning = true;
    return !measurement_collision_rearm_allowed(true, &fusion);
}

static bool test_detector_delta_v_20ms_parity(void)
{
    collision_config_t config = make_collision_config();
    collision_detector_state_t detector;
    collision_observation_t observation;
    bno086_event_t event;

    config.metric_mask = COLLISION_METRIC_LINEAR_DELTA_V;
    config.minimum_evidence_count = 1U;
    config.linear_delta_v_threshold_mps = 0.014;
    collision_detector_initialize(&detector, &config, 10000U, 10000U);
    collision_detector_set_armed(&detector, true);

    event = make_collision_event(
        BNO086_EVENT_LINEAR_ACCELERATION, 1U, 1U,
        UINT64_C(1000000000), 0.0
    );
    if (collision_detector_process(&detector, &event, &observation))
    {
        return false;
    }
    event = make_collision_event(
        BNO086_EVENT_LINEAR_ACCELERATION, 2U, 1U,
        UINT64_C(1010000000), 1.0
    );
    if (collision_detector_process(&detector, &event, &observation)
        || observation.linear_delta_v_valid)
    {
        return false;
    }
    event = make_collision_event(
        BNO086_EVENT_LINEAR_ACCELERATION, 3U, 1U,
        UINT64_C(1020000000), 1.0
    );
    return collision_detector_process(&detector, &event, &observation)
        && observation.linear_delta_v_valid
        && close_enough(observation.linear_delta_v_mps, 0.015, 1.0e-12);
}

static bool test_payload_mapping_and_ipc_frame(void)
{
    collision_config_t config = make_collision_config();
    position_publication_t position;
    accident_event_payload_t sent_payload;
    accident_event_payload_t received_payload;
    ipc_header_t sent_header;
    ipc_header_t received_header;
    int sockets[2];
    bool passed;

    memset(&position, 0, sizeof(position));
    position.valid = true;
    position.heading_valid = true;
    position.x_cm = 12.34;
    position.y_cm = -5.67;
    position.heading_deg = -1.0;
    if (!collision_fill_payload(
            &config,
            &position,
            INT64_C(123456),
            &sent_payload
        ))
    {
        return false;
    }
    if (sent_payload.station_id != 0U
        || sent_payload.sequence_number != 0U
        || sent_payload.detection_time != INT64_C(123456)
        || sent_payload.reference_time != 0
        || sent_payload.latitude != 900000001
        || sent_payload.longitude != 1800000001
        || sent_payload.speed_value != 16383U
        || sent_payload.heading_value != 3590U
        || sent_payload.delta_latitude != 131072
        || sent_payload.delta_longitude != 131072
        || sent_payload.delta_altitude != 12800
        || sent_payload.local_x_mm != 123
        || sent_payload.local_y_mm != -57)
    {
        return false;
    }

    memset(&position, 0, sizeof(position));
    if (collision_fill_payload(
            &config,
            &position,
            INT64_C(1),
            &received_payload
        ))
    {
        return false;
    }
    position.valid = true;
    position.x_cm = 100000.1;
    position.y_cm = 0.0;
    if (collision_fill_payload(
            &config,
            &position,
            INT64_C(1),
            &received_payload
        ))
    {
        return false;
    }
    position.x_cm = 0.0;
    if (collision_fill_payload(
            &config,
            &position,
            INT64_C(0),
            &received_payload
        ))
    {
        return false;
    }

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) != 0)
    {
        return false;
    }
    memset(&sent_header, 0, sizeof(sent_header));
    sent_header.type = IPC_MSG_ACCIDENT_EVENT;
    sent_header.sequence = 7U;
    sent_header.timestamp = UINT64_C(987654321);
    memset(&received_header, 0, sizeof(received_header));
    memset(&received_payload, 0, sizeof(received_payload));

    passed = ipc_send(
            sockets[0],
            &sent_header,
            &sent_payload,
            sizeof(sent_payload)
        ) == IPC_SUCCESS
        && ipc_recv(
            sockets[1],
            &received_header,
            &received_payload,
            sizeof(received_payload)
        ) == IPC_SUCCESS
        && received_header.type == IPC_MSG_ACCIDENT_EVENT
        && received_header.sequence == 7U
        && received_header.timestamp == UINT64_C(987654321)
        && received_header.length == sizeof(received_payload)
        && received_payload.detection_time == sent_payload.detection_time
        && received_payload.local_x_mm == sent_payload.local_x_mm
        && received_payload.local_y_mm == sent_payload.local_y_mm
        && received_payload.heading_value == sent_payload.heading_value;
    close(sockets[0]);
    close(sockets[1]);
    return passed;
}

static bool test_accident_testbed_origin_publication(void)
{
    position_publication_t publication;
    position_publication_t snapshot;
    bool passed;

    memset(&publication, 0, sizeof(publication));
    memset(&snapshot, 0, sizeof(snapshot));
    if (!accident_testbed_config_is_valid()
        || position_publication_initialize(&publication) != 0)
    {
        return false;
    }
    accident_position_publication_update(
        &publication,
        0.0,
        0.0,
        true,
        0.0
    );
    position_publication_snapshot(&publication, &snapshot);
    passed = snapshot.valid
        && snapshot.heading_valid
        && close_enough(
            snapshot.x_cm,
            ACCIDENT_START_X_MM / 10.0,
            1.0e-12
        )
        && close_enough(
            snapshot.y_cm,
            ACCIDENT_START_Y_MM / 10.0,
            1.0e-12
        )
        && close_enough(
            snapshot.heading_deg,
            accident_testbed_heading_deg(0.0),
            1.0e-12
        );
    position_publication_deinitialize(&publication);
    return passed;
}

int main(void)
{
    bool calibration = test_calibration_command_boundary();
    bool disabled = test_detector_disabled_and_single_evidence();
    bool coincidence = test_detector_coincidence_latch_and_rearm();
    bool gap = test_detector_gap_and_generation_reset();
    bool boundary = test_discontinuity_sample_cannot_trigger();
    bool startup = test_startup_loss_and_calibration_rearm_policy();
    bool delta_v = test_detector_delta_v_20ms_parity();
    bool payload = test_payload_mapping_and_ipc_frame();
    bool testbed_origin = test_accident_testbed_origin_publication();
    bool passed = calibration && disabled && coincidence && gap
               && boundary && startup
               && delta_v && payload && testbed_origin;

    puts("=== C ACCIDENT SENSOR REGRESSION ===");
    printf(
        "normal -> c CW/CCW -> fresh finish/flush -> first flow : %s\n",
        calibration ? "PASS" : "FAIL"
    );
    printf("disabled/disarmed + single evidence                : %s\n",
           disabled ? "PASS" : "FAIL");
    printf("coincidence -> one event -> cooldown/quiet rearm   : %s\n",
           coincidence ? "PASS" : "FAIL");
    printf("sequence/generation gap resets evidence and DV    : %s\n",
           gap ? "PASS" : "FAIL");
    printf("discontinuity sample cannot trigger min=1         : %s\n",
           boundary ? "PASS" : "FAIL");
    printf("startup loss reset + calibration rearm policy     : %s\n",
           startup ? "PASS" : "FAIL");
    printf("20 ms trapezoid sliding delta-V = 0.015 m/s       : %s\n",
           delta_v ? "PASS" : "FAIL");
    printf("position/sentinel payload + socketpair IPC frame  : %s\n",
           payload ? "PASS" : "FAIL");
    printf("known testbed start position/heading publication  : %s\n",
           testbed_origin ? "PASS" : "FAIL");
    printf("FINAL RESULT=%s\n", passed ? "PASS" : "FAIL");

    return passed ? 0 : 1;
}
