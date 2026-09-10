#define _POSIX_C_SOURCE 200809L

/*
 * BNO086 C driver regression / hardware validation
 *
 * Build (deterministic mock 포함):
 *
 *   gcc -std=c11 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror \
 *       -DBNO086_TESTING -I. scratch/test_bno086_c.c bno086.c \
 *       -pthread -lm -o /tmp/test_bno086_c
 *
 * Run:
 *
 *   /tmp/test_bno086_c mock
 *   /tmp/test_bno086_c feature 10
 *   /tmp/test_bno086_c stability 60
 *   /tmp/test_bno086_c accel 30
 *
 * mock 이외 mode는 실제 /dev/i2c-1 및 BNO086 GPIO를 사용한다. PAA5100JE,
 * 위치 융합, IPC에는 접근하지 않는다.
 */

#include "bno086.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define NS_PER_SECOND       UINT64_C(1000000000)
#define NS_PER_MILLISECOND  UINT64_C(1000000)

#define RV_STALE_NS    UINT64_C(200000000)
#define GYRO_STALE_NS  UINT64_C(200000000)
#define MAG_STALE_NS   UINT64_C(500000000)
#define ACCEL_STALE_NS UINT64_C(200000000)

#define FEATURE_TIMEOUT_MS 1500u
#define REPORT_TIMEOUT_MS  1500u

typedef struct
{
    const char *short_name;
    const char *long_name;
    uint8_t sensor_id;
    bno086_event_type_t event_type;
    uint32_t requested_interval_us;
    uint64_t stale_limit_ns;

} stream_spec_t;

typedef struct
{
    uint64_t reports;
    uint64_t first_ns;
    uint64_t last_ns;
    uint64_t max_gap_ns;
    uint32_t applied_interval_us;
    bool applied_valid;

} stream_stats_t;

static volatile sig_atomic_t stop_requested;

/* 기존 feature-race 조건: Mag 25 Hz. */
static const stream_spec_t feature_streams[] = {
    {
        "RV", "Rotation Vector", BNO086_SENSOR_ROTATION_VECTOR,
        BNO086_EVENT_ORIENTATION, 10000u, RV_STALE_NS
    },
    {
        "GYRO", "Gyroscope", BNO086_SENSOR_GYROSCOPE_CALIBRATED,
        BNO086_EVENT_GYROSCOPE, 10000u, GYRO_STALE_NS
    },
    {
        "MAG", "Magnetometer", BNO086_SENSOR_MAGNETOMETER_CALIBRATED,
        BNO086_EVENT_MAGNETOMETER, 40000u, MAG_STALE_NS
    }
};

/* 실제 sensor_process/read_flow production 조건: Mag calibration 50 Hz. */
static const stream_spec_t fusion_streams[] = {
    {
        "RV", "Rotation Vector", BNO086_SENSOR_ROTATION_VECTOR,
        BNO086_EVENT_ORIENTATION, 10000u, RV_STALE_NS
    },
    {
        "GYRO", "Gyroscope", BNO086_SENSOR_GYROSCOPE_CALIBRATED,
        BNO086_EVENT_GYROSCOPE, 10000u, GYRO_STALE_NS
    },
    {
        "MAG", "Magnetometer", BNO086_SENSOR_MAGNETOMETER_CALIBRATED,
        BNO086_EVENT_MAGNETOMETER, 20000u, MAG_STALE_NS
    }
};

static const stream_spec_t acceleration_streams[] = {
    {
        "ACCEL", "Accelerometer", BNO086_SENSOR_ACCELEROMETER,
        BNO086_EVENT_ACCELEROMETER, 10000u, ACCEL_STALE_NS
    },
    {
        "LINEAR", "Linear Acceleration", BNO086_SENSOR_LINEAR_ACCELERATION,
        BNO086_EVENT_LINEAR_ACCELERATION, 10000u, ACCEL_STALE_NS
    }
};

static void handle_signal(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static uint64_t monotonic_ns(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
    {
        return 0u;
    }

    return (uint64_t)now.tv_sec * NS_PER_SECOND + (uint64_t)now.tv_nsec;
}

static void sleep_ms(unsigned int milliseconds)
{
    struct timespec request;

    request.tv_sec = (time_t)(milliseconds / 1000u);
    request.tv_nsec = (long)(milliseconds % 1000u) * 1000000L;
    while (nanosleep(&request, &request) != 0 && errno == EINTR)
    {
        /* signal 이후 남은 시간만 기다린다. */
    }
}

static double ns_to_seconds(uint64_t value)
{
    return (double)value / (double)NS_PER_SECOND;
}

static double ns_to_milliseconds(uint64_t value)
{
    return (double)value / (double)NS_PER_MILLISECOND;
}

#ifdef BNO086_TESTING

static bool nearly_equal(double actual, double expected, double tolerance)
{
    return fabs(actual - expected) <= tolerance;
}

static void write_i16_le(uint8_t *destination, int16_t value)
{
    uint16_t bits;

    memcpy(&bits, &value, sizeof(bits));
    destination[0] = (uint8_t)(bits & UINT16_C(0x00FF));
    destination[1] = (uint8_t)((bits >> 8) & UINT16_C(0x00FF));
}

static void write_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value & UINT32_C(0x000000FF));
    destination[1] = (uint8_t)((value >> 8) & UINT32_C(0x000000FF));
    destination[2] = (uint8_t)((value >> 16) & UINT32_C(0x000000FF));
    destination[3] = (uint8_t)((value >> 24) & UINT32_C(0x000000FF));
}

#endif

static int parse_positive_uint(const char *text, unsigned int *value)
{
    char *end = NULL;
    unsigned long parsed;

    if (text == NULL || value == NULL || text[0] == '\0')
    {
        return 0;
    }

    errno = 0;
    parsed = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0'
        || parsed == 0ul || parsed > (unsigned long)UINT32_MAX)
    {
        return 0;
    }

    *value = (unsigned int)parsed;
    return 1;
}

static int parse_positive_double(const char *text, double *value)
{
    char *end = NULL;
    double parsed;

    if (text == NULL || value == NULL || text[0] == '\0')
    {
        return 0;
    }

    errno = 0;
    parsed = strtod(text, &end);
    if (errno != 0 || end == text || *end != '\0'
        || !isfinite(parsed) || parsed <= 0.0 || parsed > 86400.0)
    {
        return 0;
    }

    *value = parsed;
    return 1;
}

static void print_last_driver_error(
    const bno086_snapshot_t *snapshot,
    const bno086_diagnostics_t *diagnostics
)
{
    if (snapshot != NULL && snapshot->boot_failure_reason[0] != '\0')
    {
        printf("failure_reason=%s\n", snapshot->boot_failure_reason);
    }
    if (diagnostics != NULL && diagnostics->last_io_error[0] != '\0')
    {
        printf("last_transport_error=%s", diagnostics->last_io_error);
        if (diagnostics->last_errno != 0)
        {
            printf(" (errno=%d: %s)",
                   diagnostics->last_errno,
                   strerror(diagnostics->last_errno));
        }
        putchar('\n');
    }
}

static int initialize_hardware(bno086_t *imu)
{
    bno086_config_t config;

    memset(imu, 0, sizeof(*imu));
    bno086_default_config(&config);

    if (bno086_init(imu, &config) <= 0)
    {
        fputs("bno086_init failed\n", stderr);
        return 0;
    }
    if (bno086_begin(imu) <= 0)
    {
        bno086_snapshot_t snapshot;
        bno086_diagnostics_t diagnostics;

        memset(&snapshot, 0, sizeof(snapshot));
        memset(&diagnostics, 0, sizeof(diagnostics));
        (void)bno086_get_snapshot(imu, &snapshot);
        (void)bno086_get_diagnostics(imu, &diagnostics);
        fputs("bno086_begin failed\n", stderr);
        print_last_driver_error(&snapshot, &diagnostics);
        return 0;
    }

    return 1;
}

static bool applied_interval_is_valid(uint32_t requested, int valid, uint32_t applied)
{
    if (valid <= 0)
    {
        return false;
    }

    return requested > 0u ? applied > 0u : applied == 0u;
}

static int configure_stream(
    bno086_t *imu,
    const stream_spec_t *spec,
    uint32_t *applied_interval_us,
    bool *report_received
)
{
    int enabled;
    int interval_valid;
    int report_result = 0;
    bno086_snapshot_t snapshot;

    enabled = bno086_enable_feature(
        imu,
        spec->sensor_id,
        spec->requested_interval_us,
        FEATURE_TIMEOUT_MS
    );
    interval_valid = bno086_get_feature_interval(
        imu,
        spec->sensor_id,
        applied_interval_us
    );

    memset(&snapshot, 0, sizeof(snapshot));
    if (bno086_get_snapshot(imu, &snapshot) > 0 && snapshot.startup_ready)
    {
        report_result = bno086_wait_for_sensor_report(
            imu,
            spec->sensor_id,
            REPORT_TIMEOUT_MS
        );
    }

    *report_received = report_result > 0;
    return enabled > 0
        && applied_interval_is_valid(
            spec->requested_interval_us,
            interval_valid,
            *applied_interval_us
        )
        && *report_received;
}

static uint64_t snapshot_timestamp(
    const bno086_snapshot_t *snapshot,
    uint8_t sensor_id
)
{
    switch (sensor_id)
    {
        case BNO086_SENSOR_ROTATION_VECTOR:
        case BNO086_SENSOR_GAME_ROTATION_VECTOR:
            return snapshot->last_orientation_update_ns;

        case BNO086_SENSOR_GYROSCOPE_CALIBRATED:
            return snapshot->last_gyro_update_ns;

        case BNO086_SENSOR_MAGNETOMETER_CALIBRATED:
            return snapshot->last_mag_update_ns;

        case BNO086_SENSOR_ACCELEROMETER:
            return snapshot->last_accel_update_ns;

        case BNO086_SENSOR_LINEAR_ACCELERATION:
            return snapshot->last_linear_accel_update_ns;

        default:
            return 0u;
    }
}

static void record_event(
    const bno086_event_t *event,
    const stream_spec_t *specs,
    stream_stats_t *stats,
    size_t stream_count
)
{
    size_t index;

    for (index = 0u; index < stream_count; index++)
    {
        stream_stats_t *stream = &stats[index];

        if (event->type != specs[index].event_type
            || event->report_id != specs[index].sensor_id)
        {
            continue;
        }

        if (stream->reports == 0u)
        {
            stream->first_ns = event->monotonic_ns;
        }
        else if (event->monotonic_ns >= stream->last_ns)
        {
            uint64_t gap = event->monotonic_ns - stream->last_ns;

            if (gap > stream->max_gap_ns)
            {
                stream->max_gap_ns = gap;
            }
        }

        stream->last_ns = event->monotonic_ns;
        stream->reports++;
        break;
    }
}

static int drain_cursor(
    const bno086_t *imu,
    bno086_event_cursor_t *cursor,
    const stream_spec_t *specs,
    stream_stats_t *stats,
    size_t stream_count
)
{
    bno086_event_t event;
    int result;

    for (;;)
    {
        result = bno086_event_next(imu, cursor, &event, 0);
        if (result <= 0)
        {
            return result < 0 ? 0 : 1;
        }
        record_event(&event, specs, stats, stream_count);
    }
}

static void finish_stream_boundaries(
    stream_stats_t *stats,
    size_t stream_count,
    uint64_t start_ns,
    uint64_t end_ns
)
{
    size_t index;

    for (index = 0u; index < stream_count; index++)
    {
        uint64_t first_gap;
        uint64_t final_gap;

        if (stats[index].reports == 0u)
        {
            stats[index].max_gap_ns = end_ns - start_ns;
            continue;
        }

        first_gap = stats[index].first_ns > start_ns
                  ? stats[index].first_ns - start_ns
                  : 0u;
        final_gap = end_ns > stats[index].last_ns
                  ? end_ns - stats[index].last_ns
                  : 0u;
        if (first_gap > stats[index].max_gap_ns)
        {
            stats[index].max_gap_ns = first_gap;
        }
        if (final_gap > stats[index].max_gap_ns)
        {
            stats[index].max_gap_ns = final_gap;
        }
    }
}

static bool rate_is_acceptable(
    const stream_stats_t *stats,
    double elapsed_seconds
)
{
    double measured;
    double expected;

    if (!stats->applied_valid || stats->applied_interval_us == 0u
        || elapsed_seconds <= 0.0)
    {
        return false;
    }

    measured = (double)stats->reports / elapsed_seconds;
    expected = 1000000.0 / (double)stats->applied_interval_us;
    return measured >= expected * 0.80 && measured <= expected * 1.20;
}

static void print_stream_result(
    const stream_spec_t *spec,
    const stream_stats_t *stats,
    double elapsed_seconds
)
{
    double measured_rate = elapsed_seconds > 0.0
                         ? (double)stats->reports / elapsed_seconds
                         : 0.0;
    double expected_rate = stats->applied_valid
                        && stats->applied_interval_us > 0u
                         ? 1000000.0 / (double)stats->applied_interval_us
                         : 0.0;

    printf("%s\n", spec->long_name);
    printf("reports=%" PRIu64 "\n", stats->reports);
    if (stats->applied_valid)
    {
        printf("applied_interval=%" PRIu32 " us\n",
               stats->applied_interval_us);
    }
    else
    {
        puts("applied_interval=None");
    }
    printf("expected_rate=%.3f Hz\n", expected_rate);
    printf("measured_rate=%.3f Hz\n", measured_rate);
    printf("max_gap=%.3f ms\n\n", ns_to_milliseconds(stats->max_gap_ns));
}

static int run_feature_race(unsigned int trials)
{
    unsigned int trial;
    unsigned int setup_success = 0u;
    uint64_t false_negative = 0u;
    uint64_t total_resets = 0u;
    uint64_t total_bus_errors = 0u;
    uint64_t total_validation_errors = 0u;
    uint64_t total_other_errors = 0u;
    uint64_t total_nulls = 0u;
    uint64_t total_null_success = 0u;
    uint64_t total_null_fail = 0u;

    puts("=== C FEATURE RACE TEST ===");

    for (trial = 1u; trial <= trials && !stop_requested; trial++)
    {
        bno086_t imu = {0};
        bno086_snapshot_t snapshot;
        bno086_diagnostics_t diagnostics;
        bool began;
        bool trial_pass = true;
        size_t index;

        printf("Trial %02u\n", trial);
        began = initialize_hardware(&imu) > 0;

        for (index = 0u;
             index < sizeof(feature_streams) / sizeof(feature_streams[0]);
             index++)
        {
            const stream_spec_t *spec = &feature_streams[index];
            uint32_t applied = 0u;
            bool report = false;
            int enabled = 0;
            int interval_valid = 0;
            bool row_pass = false;

            if (began)
            {
                enabled = bno086_enable_feature(
                    &imu,
                    spec->sensor_id,
                    spec->requested_interval_us,
                    FEATURE_TIMEOUT_MS
                );
                interval_valid = bno086_get_feature_interval(
                    &imu,
                    spec->sensor_id,
                    &applied
                );

                memset(&snapshot, 0, sizeof(snapshot));
                if (bno086_get_snapshot(&imu, &snapshot) > 0
                    && snapshot.startup_ready)
                {
                    report = bno086_wait_for_sensor_report(
                        &imu,
                        spec->sensor_id,
                        REPORT_TIMEOUT_MS
                    ) > 0;
                }

                row_pass = enabled > 0
                        && applied_interval_is_valid(
                            spec->requested_interval_us,
                            interval_valid,
                            applied
                        )
                        && report;
                if (enabled <= 0
                    && (applied_interval_is_valid(
                            spec->requested_interval_us,
                            interval_valid,
                            applied)
                        || report))
                {
                    false_negative++;
                }
            }

            printf("%-5s requested=%5" PRIu32 " us  enable=%-5s  ",
                   spec->short_name,
                   spec->requested_interval_us,
                   enabled > 0 ? "TRUE" : "FALSE");
            if (interval_valid > 0)
            {
                printf("applied=%-6" PRIu32 "  ", applied);
            }
            else
            {
                printf("applied=%-6s  ", "None");
            }
            printf("report=%-3s  %s\n",
                   report ? "YES" : "NO",
                   row_pass ? "PASS" : "FAIL");
            trial_pass = trial_pass && row_pass;
        }

        memset(&snapshot, 0, sizeof(snapshot));
        memset(&diagnostics, 0, sizeof(diagnostics));
        if (imu.impl != NULL)
        {
            (void)bno086_get_snapshot(&imu, &snapshot);
            (void)bno086_get_diagnostics(&imu, &diagnostics);
        }

        if (began && trial_pass)
        {
            setup_success++;
        }
        total_resets += diagnostics.reset_count;
        total_bus_errors += diagnostics.syscall_bus_error_count;
        total_validation_errors += diagnostics.shtp_validation_error_count;
        total_other_errors += diagnostics.other_transport_error_count;
        total_nulls += diagnostics.phase2_null_count;
        total_null_success += diagnostics.phase2_null_retry_success;
        total_null_fail += diagnostics.phase2_null_retry_fail;

        printf("reset_count=%" PRIu64 "  syscall_bus_errors=%" PRIu64
               "  shtp_validation_errors=%" PRIu64 "\n",
               diagnostics.reset_count,
               diagnostics.syscall_bus_error_count,
               diagnostics.shtp_validation_error_count);
        printf("phase2_null_count=%" PRIu64
               "  null_retry_success=%" PRIu64
               "  null_retry_fail=%" PRIu64 "\n",
               diagnostics.phase2_null_count,
               diagnostics.phase2_null_retry_success,
               diagnostics.phase2_null_retry_fail);
        if (!began || !trial_pass)
        {
            print_last_driver_error(&snapshot, &diagnostics);
        }

        bno086_deinit(&imu);
        putchar('\n');
        if (trial != trials && !stop_requested)
        {
            sleep_ms(100u);
        }
    }

    puts("SUMMARY");
    printf("trials=%u\n", trials);
    printf("setup_success=%u/%u\n", setup_success, trials);
    printf("false_negative=%" PRIu64 "\n", false_negative);
    printf("resets=%" PRIu64 "\n", total_resets);
    printf("syscall_bus_errors=%" PRIu64 "\n", total_bus_errors);
    printf("shtp_validation_errors=%" PRIu64 "\n", total_validation_errors);
    printf("other_transport_errors=%" PRIu64 "\n", total_other_errors);
    printf("phase2_null_count=%" PRIu64 "\n", total_nulls);
    printf("phase2_null_retry_success=%" PRIu64 "\n", total_null_success);
    printf("phase2_null_retry_fail=%" PRIu64 "\n", total_null_fail);

    return !stop_requested
        && setup_success == trials
        && false_negative == 0u
        && total_resets == 0u
        && total_bus_errors == 0u
        && total_validation_errors == 0u
        && total_other_errors == 0u
        && total_null_fail == 0u
         ? 0 : 1;
}

static int monitor_streams(
    const char *title,
    const stream_spec_t *specs,
    size_t stream_count,
    double requested_duration,
    bool verify_second_cursor
)
{
    bno086_t imu = {0};
    bno086_event_cursor_t cursor_a;
    bno086_event_cursor_t cursor_b;
    stream_stats_t stats_a[3];
    stream_stats_t stats_b[3];
    bool stale_flags[3] = {false, false, false};
    bno086_snapshot_t snapshot;
    bno086_diagnostics_t diagnostics;
    uint64_t start_ns;
    uint64_t end_ns;
    uint64_t deadline_ns;
    uint64_t stale_events = 0u;
    uint64_t startup_losses = 0u;
    bool startup_was_ready;
    bool setup_ok = true;
    bool passed;
    size_t index;

    if (stream_count == 0u || stream_count > 3u)
    {
        return 1;
    }

    memset(&cursor_a, 0, sizeof(cursor_a));
    memset(&cursor_b, 0, sizeof(cursor_b));
    memset(stats_a, 0, sizeof(stats_a));
    memset(stats_b, 0, sizeof(stats_b));
    memset(&snapshot, 0, sizeof(snapshot));
    memset(&diagnostics, 0, sizeof(diagnostics));

    if (!initialize_hardware(&imu))
    {
        bno086_deinit(&imu);
        return 1;
    }

    for (index = 0u; index < stream_count; index++)
    {
        bool report_received = false;

        setup_ok = configure_stream(
            &imu,
            &specs[index],
            &stats_a[index].applied_interval_us,
            &report_received
        ) > 0 && setup_ok;
        stats_a[index].applied_valid = bno086_get_feature_interval(
            &imu,
            specs[index].sensor_id,
            &stats_a[index].applied_interval_us
        ) > 0;
        stats_b[index].applied_interval_us = stats_a[index].applied_interval_us;
        stats_b[index].applied_valid = stats_a[index].applied_valid;

        printf("setup %-6s FC=%s applied=%" PRIu32 " report=%s\n",
               specs[index].short_name,
               stats_a[index].applied_valid ? "YES" : "NO",
               stats_a[index].applied_interval_us,
               report_received ? "YES" : "NO");
    }

    if (!setup_ok)
    {
        (void)bno086_get_snapshot(&imu, &snapshot);
        (void)bno086_get_diagnostics(&imu, &diagnostics);
        print_last_driver_error(&snapshot, &diagnostics);
        bno086_deinit(&imu);
        return 1;
    }

    if (specs == fusion_streams)
    {
        int calibration_started = bno086_start_dynamic_calibration(&imu);

        printf("dynamic_calibration=%s\n",
               calibration_started > 0 ? "YES" : "NO");
        if (calibration_started <= 0)
        {
            (void)bno086_get_snapshot(&imu, &snapshot);
            (void)bno086_get_diagnostics(&imu, &diagnostics);
            print_last_driver_error(&snapshot, &diagnostics);
            bno086_deinit(&imu);
            return 1;
        }
    }

    start_ns = monotonic_ns();
    deadline_ns = start_ns + (uint64_t)(requested_duration * 1000000000.0);
    if (bno086_event_cursor_init(&imu, &cursor_a, false) <= 0
        || (verify_second_cursor
            && bno086_event_cursor_init(&imu, &cursor_b, false) <= 0))
    {
        fputs("event cursor initialization failed\n", stderr);
        bno086_deinit(&imu);
        return 1;
    }

    (void)bno086_get_snapshot(&imu, &snapshot);
    startup_was_ready = snapshot.startup_ready;

    while (!stop_requested && monotonic_ns() < deadline_ns)
    {
        uint64_t now;

        (void)bno086_update(&imu);
        if (!drain_cursor(&imu, &cursor_a, specs, stats_a, stream_count)
            || (verify_second_cursor
                && !drain_cursor(
                    &imu,
                    &cursor_b,
                    specs,
                    stats_b,
                    stream_count)))
        {
            setup_ok = false;
            break;
        }

        now = monotonic_ns();
        (void)bno086_get_snapshot(&imu, &snapshot);
        if (startup_was_ready && !snapshot.startup_ready)
        {
            startup_losses++;
        }
        startup_was_ready = snapshot.startup_ready;

        for (index = 0u; index < stream_count; index++)
        {
            uint64_t timestamp = snapshot_timestamp(
                &snapshot,
                specs[index].sensor_id
            );
            bool stale = timestamp == 0u
                      || now < timestamp
                      || now - timestamp > specs[index].stale_limit_ns;

            if (stale && !stale_flags[index])
            {
                stale_events++;
            }
            stale_flags[index] = stale;
        }

        sleep_ms(1u);
    }

    (void)bno086_update(&imu);
    (void)drain_cursor(&imu, &cursor_a, specs, stats_a, stream_count);
    if (verify_second_cursor)
    {
        (void)drain_cursor(&imu, &cursor_b, specs, stats_b, stream_count);
    }
    end_ns = monotonic_ns();
    finish_stream_boundaries(stats_a, stream_count, start_ns, end_ns);
    if (verify_second_cursor)
    {
        finish_stream_boundaries(stats_b, stream_count, start_ns, end_ns);
    }

    (void)bno086_get_snapshot(&imu, &snapshot);
    (void)bno086_get_diagnostics(&imu, &diagnostics);

    printf("\n=== %s ===\n", title);
    printf("duration=%.3fs\n\n", ns_to_seconds(end_ns - start_ns));
    passed = setup_ok && !stop_requested;
    for (index = 0u; index < stream_count; index++)
    {
        print_stream_result(
            &specs[index],
            &stats_a[index],
            ns_to_seconds(end_ns - start_ns)
        );
        passed = passed
              && rate_is_acceptable(
                    &stats_a[index],
                    ns_to_seconds(end_ns - start_ns)
                 )
              && stats_a[index].max_gap_ns <= specs[index].stale_limit_ns;

        if (verify_second_cursor)
        {
            printf("%s cursor_B_reports=%" PRIu64 "\n",
                   specs[index].short_name,
                   stats_b[index].reports);
            passed = passed && stats_b[index].reports == stats_a[index].reports;
        }
    }

    printf("reset_count=%" PRIu64 "\n", diagnostics.reset_count);
    printf("syscall_bus_errors=%" PRIu64 "\n",
           diagnostics.syscall_bus_error_count);
    printf("shtp_validation_errors=%" PRIu64 "\n",
           diagnostics.shtp_validation_error_count);
    printf("other_transport_errors=%" PRIu64 "\n",
           diagnostics.other_transport_error_count);
    printf("phase2_null_count=%" PRIu64 "\n",
           diagnostics.phase2_null_count);
    printf("phase2_null_retry_success=%" PRIu64 "\n",
           diagnostics.phase2_null_retry_success);
    printf("phase2_null_retry_fail=%" PRIu64 "\n",
           diagnostics.phase2_null_retry_fail);
    printf("startup_losses=%" PRIu64 "\n", startup_losses);
    printf("stale_events=%" PRIu64 "\n", stale_events);
    printf("cursor_A_dropped=%" PRIu64 "\n", cursor_a.dropped);
    if (verify_second_cursor)
    {
        printf("cursor_B_dropped=%" PRIu64 "\n", cursor_b.dropped);
    }

    if (specs == fusion_streams)
    {
        printf("RV_accuracy=%u\n", (unsigned int)snapshot.rotation_accuracy);
        printf("Mag_accuracy=%u\n", (unsigned int)snapshot.mag_accuracy);
    }
    else
    {
        printf("Accel_accuracy=%u\n", (unsigned int)snapshot.accel_accuracy);
        printf("Linear_accuracy=%u\n",
               (unsigned int)snapshot.linear_accel_accuracy);
        printf("Accel_final=(%+.4f, %+.4f, %+.4f) m/s^2\n",
               snapshot.accel_x_mps2,
               snapshot.accel_y_mps2,
               snapshot.accel_z_mps2);
        printf("Linear_final=(%+.4f, %+.4f, %+.4f) m/s^2\n",
               snapshot.linear_accel_x_mps2,
               snapshot.linear_accel_y_mps2,
               snapshot.linear_accel_z_mps2);
    }

    passed = passed
          && diagnostics.reset_count == 0u
          && diagnostics.syscall_bus_error_count == 0u
          && diagnostics.shtp_validation_error_count == 0u
          && diagnostics.other_transport_error_count == 0u
          && diagnostics.phase2_null_retry_fail == 0u
          && startup_losses == 0u
          && stale_events == 0u
          && cursor_a.dropped == 0u
          && (!verify_second_cursor || cursor_b.dropped == 0u)
          && snapshot.startup_ready;

    if (!passed)
    {
        print_last_driver_error(&snapshot, &diagnostics);
    }
    printf("\nRESULT=%s\n", passed ? "PASS" : "FAIL");
    bno086_deinit(&imu);
    return passed ? 0 : 1;
}

#ifdef BNO086_TESTING

static int make_mock_imu(bno086_t *imu)
{
    memset(imu, 0, sizeof(*imu));
    bno086_test_transport_reset();
    if (bno086_init(imu, NULL) <= 0)
    {
        return 0;
    }
    if (bno086_test_make_virtual_open(imu) <= 0)
    {
        bno086_deinit(imu);
        return 0;
    }
    return 1;
}

static bool mock_null_retry_success(void)
{
    bno086_t imu = {0};
    const uint8_t phase1[] = {0x15u, 0x00u, 0x02u, 0x1Au};
    const uint8_t null_header[] = {0u, 0u, 0u, 0u};
    const uint8_t continuation[] = {
        0x15u, 0x80u, 0x02u, 0x1Bu,
        0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u,
        9u, 10u, 11u, 12u, 13u, 14u, 15u, 16u
    };
    const bno086_test_read_step_t script[] = {
        {phase1, sizeof(phase1), 0, 0u},
        {null_header, sizeof(null_header), 0, 0u},
        {continuation, sizeof(continuation), 0, 0u}
    };
    bno086_test_packet_t packet;
    bno086_diagnostics_t diagnostics;
    bool passed;

    memset(&packet, 0, sizeof(packet));
    memset(&diagnostics, 0, sizeof(diagnostics));
    if (!make_mock_imu(&imu))
    {
        return false;
    }
    bno086_test_set_read_script(script, sizeof(script) / sizeof(script[0]));

    passed = bno086_test_read_packet(&imu, &packet) > 0
          && packet.total_length == 21u
          && packet.channel == 2u
          && packet.sequence == 0x1Bu
          && packet.cargo_length == 17u
          && memcmp(packet.cargo, continuation + 4u, 17u) == 0
          && bno086_test_read_call_count() == 3u;
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    passed = passed
          && diagnostics.phase2_null_count == 1u
          && diagnostics.phase2_null_retry_success == 1u
          && diagnostics.phase2_null_retry_fail == 0u
          && diagnostics.syscall_bus_error_count == 0u
          && diagnostics.shtp_validation_error_count == 0u;

    bno086_deinit(&imu);
    return passed;
}

static bool mock_null_retry_failure(void)
{
    bno086_t imu = {0};
    const uint8_t phase1[] = {0x0Du, 0x00u, 0x02u, 0x20u};
    const uint8_t null_header[] = {0u, 0u, 0u, 0u};
    const bno086_test_read_step_t script[] = {
        {phase1, sizeof(phase1), 0, 0u},
        {null_header, sizeof(null_header), 0, 0u},
        {null_header, sizeof(null_header), 0, 0u}
    };
    bno086_test_packet_t packet;
    bno086_diagnostics_t diagnostics;
    bool passed;

    memset(&packet, 0, sizeof(packet));
    memset(&diagnostics, 0, sizeof(diagnostics));
    if (!make_mock_imu(&imu))
    {
        return false;
    }
    bno086_test_set_read_script(script, sizeof(script) / sizeof(script[0]));

    passed = bno086_test_read_packet(&imu, &packet) == 0;
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    passed = passed
          && diagnostics.phase2_null_count == 1u
          && diagnostics.phase2_null_retry_success == 0u
          && diagnostics.phase2_null_retry_fail == 1u
          && diagnostics.shtp_validation_error_count == 1u
          && diagnostics.syscall_bus_error_count == 0u;

    bno086_deinit(&imu);
    return passed;
}

static bool mock_validation_and_bus_categories(void)
{
    bno086_t imu = {0};
    const uint8_t phase1[] = {0x0Du, 0x00u, 0x02u, 0x30u};
    const uint8_t wrong_sequence[] = {
        0x0Du, 0x80u, 0x02u, 0x44u,
        0xFCu, 0x05u, 0u, 0u, 0u, 0x10u, 0x27u, 0u, 0u
    };
    const bno086_test_read_step_t validation_script[] = {
        {phase1, sizeof(phase1), 0, 0u},
        {wrong_sequence, sizeof(wrong_sequence), 0, 0u}
    };
    const bno086_test_read_step_t bus_script[] = {
        {NULL, 0u, EIO, 0u}
    };
    bno086_test_packet_t packet;
    bno086_diagnostics_t diagnostics;
    bool passed;

    memset(&packet, 0, sizeof(packet));
    memset(&diagnostics, 0, sizeof(diagnostics));
    if (!make_mock_imu(&imu))
    {
        return false;
    }

    bno086_test_set_read_script(
        validation_script,
        sizeof(validation_script) / sizeof(validation_script[0])
    );
    passed = bno086_test_read_packet(&imu, &packet) == 0;
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    passed = passed
          && diagnostics.shtp_validation_error_count == 1u
          && diagnostics.syscall_bus_error_count == 0u
          && diagnostics.phase2_null_count == 0u;

    bno086_test_set_read_script(bus_script, sizeof(bus_script) / sizeof(bus_script[0]));
    passed = passed && bno086_test_read_packet(&imu, &packet) == 0;
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    passed = passed
          && diagnostics.shtp_validation_error_count == 1u
          && diagnostics.syscall_bus_error_count == 1u
          && diagnostics.last_errno == EIO;

    bno086_deinit(&imu);
    return passed;
}

static bool mock_multichunk_assembly(void)
{
    bno086_t imu = {0};
    const uint8_t phase1[] = {0x15u, 0x00u, 0x03u, 0x40u};
    const uint8_t first_chunk[] = {
        0x15u, 0x80u, 0x03u, 0x41u, 0u, 1u, 2u, 3u, 4u, 5u
    };
    const uint8_t second_chunk[] = {
        0x0Fu, 0x80u, 0x03u, 0x42u,
        6u, 7u, 8u, 9u, 10u, 11u, 12u, 13u, 14u, 15u, 16u
    };
    const bno086_test_read_step_t script[] = {
        {phase1, sizeof(phase1), 0, 0u},
        {first_chunk, sizeof(first_chunk), 0, 0u},
        {second_chunk, sizeof(second_chunk), 0, 0u}
    };
    bno086_test_packet_t packet;
    bno086_diagnostics_t diagnostics;
    size_t index;
    bool passed;

    memset(&packet, 0, sizeof(packet));
    memset(&diagnostics, 0, sizeof(diagnostics));
    if (!make_mock_imu(&imu))
    {
        return false;
    }
    bno086_test_set_read_script(script, sizeof(script) / sizeof(script[0]));

    passed = bno086_test_read_packet(&imu, &packet) > 0
          && packet.cargo_length == 17u
          && bno086_test_read_call_count() == 3u;
    for (index = 0u; index < 17u; index++)
    {
        passed = passed && packet.cargo[index] == (uint8_t)index;
    }
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    passed = passed && diagnostics.io_error_count == 0u;

    bno086_deinit(&imu);
    return passed;
}

static bool mock_feature_race(void)
{
    bno086_t imu = {0};
    uint8_t zero_fc[13] = {
        0x0Du, 0x80u, 0x02u, 0x51u,
        0xFCu, BNO086_SENSOR_ROTATION_VECTOR, 0u, 0u, 0u,
        0u, 0u, 0u, 0u
    };
    uint8_t valid_fc[13] = {
        0x0Du, 0x80u, 0x02u, 0x53u,
        0xFCu, BNO086_SENSOR_ROTATION_VECTOR, 0u, 0u, 0u,
        0u, 0u, 0u, 0u
    };
    const uint8_t zero_phase1[] = {0x0Du, 0x00u, 0x02u, 0x50u};
    const uint8_t valid_phase1[] = {0x0Du, 0x00u, 0x02u, 0x52u};
    bno086_test_read_step_t script[4];
    uint8_t write_record[32];
    uint32_t applied = 0u;
    size_t write_size;
    bool passed;

    write_u32_le(valid_fc + 9u, 10000u);
    script[0] = (bno086_test_read_step_t){
        zero_phase1, sizeof(zero_phase1), 0, 2u
    };
    script[1] = (bno086_test_read_step_t){
        zero_fc, sizeof(zero_fc), 0, 2u
    };
    script[2] = (bno086_test_read_step_t){
        valid_phase1, sizeof(valid_phase1), 0, 3u
    };
    script[3] = (bno086_test_read_step_t){
        valid_fc, sizeof(valid_fc), 0, 3u
    };

    if (!make_mock_imu(&imu))
    {
        return false;
    }
    (void)bno086_test_set_startup_ready(&imu, true);
    bno086_test_set_read_script(script, sizeof(script) / sizeof(script[0]));

    passed = bno086_enable_feature(
        &imu,
        BNO086_SENSOR_ROTATION_VECTOR,
        10000u,
        200u
    ) > 0;
    passed = passed
          && bno086_get_feature_interval(
                &imu,
                BNO086_SENSOR_ROTATION_VECTOR,
                &applied
             ) > 0
          && applied == 10000u
          && bno086_test_write_count() == 3u;

    memset(write_record, 0, sizeof(write_record));
    write_size = bno086_test_copy_write(0u, write_record, sizeof(write_record));
    passed = passed
          && write_size == 21u
          && write_record[2] == 2u
          && write_record[4] == 0xFDu
          && write_record[5] == BNO086_SENSOR_ROTATION_VECTOR
          && write_record[9] == 0x10u
          && write_record[10] == 0x27u
          && write_record[11] == 0u
          && write_record[12] == 0u;

    write_size = bno086_test_copy_write(1u, write_record, sizeof(write_record));
    passed = passed
          && write_size == 6u
          && write_record[4] == 0xFEu
          && write_record[5] == BNO086_SENSOR_ROTATION_VECTOR;
    write_size = bno086_test_copy_write(2u, write_record, sizeof(write_record));
    passed = passed
          && write_size == 6u
          && write_record[4] == 0xFEu
          && write_record[5] == BNO086_SENSOR_ROTATION_VECTOR;

    bno086_deinit(&imu);
    return passed;
}

static bool mock_dynamic_calibration_command(void)
{
    bno086_t imu = {0};
    const uint8_t phase1[] = {0x0Au, 0x00u, 0x02u, 0x20u};
    const uint8_t continuation[] = {
        0x0Au, 0x80u, 0x02u, 0x21u,
        0xF1u, 0x00u, 0x07u, 0x00u, 0x00u, 0x00u
    };
    const bno086_test_read_step_t script[] = {
        {phase1, sizeof(phase1), 0, 1u},
        {continuation, sizeof(continuation), 0, 1u}
    };
    uint8_t write_record[32];
    size_t write_size;
    bool passed;

    memset(write_record, 0, sizeof(write_record));
    if (!make_mock_imu(&imu))
    {
        return false;
    }
    (void)bno086_test_set_startup_ready(&imu, true);
    bno086_test_set_read_script(script, sizeof(script) / sizeof(script[0]));

    passed = bno086_start_dynamic_calibration(&imu) > 0
          && bno086_test_write_count() == 1u;
    write_size = bno086_test_copy_write(
        0u,
        write_record,
        sizeof(write_record)
    );
    passed = passed
          && write_size == 16u
          && write_record[2] == 2u
          && write_record[4] == 0xF2u
          && write_record[5] == 0u
          && write_record[6] == 0x07u
          && write_record[7] == 0u
          && write_record[8] == 1u
          && write_record[9] == 1u
          && write_record[10] == 0u;

    bno086_deinit(&imu);
    return passed;
}

static size_t make_vector_record(
    uint8_t *destination,
    uint8_t report_id,
    uint8_t sequence,
    uint8_t accuracy,
    int16_t x,
    int16_t y,
    int16_t z
)
{
    destination[0] = report_id;
    destination[1] = sequence;
    destination[2] = accuracy;
    destination[3] = 0u;
    write_i16_le(destination + 4u, x);
    write_i16_le(destination + 6u, y);
    write_i16_le(destination + 8u, z);
    return 10u;
}

static bool mock_parser_and_spmc_ring(void)
{
    bno086_t imu = {0};
    bno086_event_cursor_t cursor_a;
    bno086_event_cursor_t cursor_b;
    bno086_event_cursor_t lagging_cursor;
    uint8_t cargo[54];
    uint8_t vector[10];
    size_t offset = 0u;
    size_t index;
    bno086_snapshot_t snapshot;
    bno086_diagnostics_t diagnostics;
    bno086_event_t event;
    bno086_event_type_t expected_types[] = {
        BNO086_EVENT_ORIENTATION,
        BNO086_EVENT_GYROSCOPE,
        BNO086_EVENT_MAGNETOMETER,
        BNO086_EVENT_ACCELEROMETER,
        BNO086_EVENT_LINEAR_ACCELERATION
    };
    bool passed = true;
    uint64_t wait_start;
    uint64_t wait_elapsed;

    memset(&cursor_a, 0, sizeof(cursor_a));
    memset(&cursor_b, 0, sizeof(cursor_b));
    memset(&lagging_cursor, 0, sizeof(lagging_cursor));
    memset(&snapshot, 0, sizeof(snapshot));
    memset(&diagnostics, 0, sizeof(diagnostics));
    memset(cargo, 0, sizeof(cargo));
    if (!make_mock_imu(&imu))
    {
        return false;
    }

    passed = bno086_event_cursor_init(&imu, &cursor_a, false) > 0
          && bno086_event_cursor_init(&imu, &cursor_b, false) > 0;

    cargo[offset + 0u] = BNO086_SENSOR_ROTATION_VECTOR;
    cargo[offset + 1u] = 7u;
    cargo[offset + 2u] = 3u;
    cargo[offset + 3u] = 0u;
    write_i16_le(cargo + offset + 4u, 0);
    write_i16_le(cargo + offset + 6u, 0);
    write_i16_le(cargo + offset + 8u, 11585);
    write_i16_le(cargo + offset + 10u, 11585);
    write_i16_le(cargo + offset + 12u, 0);
    offset += 14u;

    offset += make_vector_record(
        cargo + offset,
        BNO086_SENSOR_GYROSCOPE_CALIBRATED,
        8u,
        2u,
        512,
        -256,
        1024
    );
    offset += make_vector_record(
        cargo + offset,
        BNO086_SENSOR_MAGNETOMETER_CALIBRATED,
        9u,
        1u,
        160,
        -320,
        80
    );
    offset += make_vector_record(
        cargo + offset,
        BNO086_SENSOR_ACCELEROMETER,
        10u,
        3u,
        256,
        -512,
        2511
    );
    offset += make_vector_record(
        cargo + offset,
        BNO086_SENSOR_LINEAR_ACCELERATION,
        11u,
        2u,
        64,
        -128,
        192
    );

    passed = passed
          && offset == sizeof(cargo)
          && bno086_test_feed_packet(&imu, 3u, 1u, cargo, offset) == 5;
    (void)bno086_get_snapshot(&imu, &snapshot);
    passed = passed
          && snapshot.orientation_sequence == 1u
          && snapshot.gyro_sequence == 1u
          && snapshot.mag_sequence == 1u
          && snapshot.accel_sequence == 1u
          && snapshot.linear_accel_sequence == 1u
          && nearly_equal(snapshot.yaw_deg, 90.0, 0.02)
          && nearly_equal(snapshot.gyro_x_rad_s, 1.0, 0.0001)
          && nearly_equal(snapshot.gyro_y_rad_s, -0.5, 0.0001)
          && nearly_equal(snapshot.mag_x_ut, 10.0, 0.0001)
          && nearly_equal(snapshot.mag_y_ut, -20.0, 0.0001)
          && nearly_equal(snapshot.accel_z_mps2, 2511.0 / 256.0, 0.0001)
          && nearly_equal(snapshot.linear_accel_x_mps2, 0.25, 0.0001);

    for (index = 0u; index < sizeof(expected_types) / sizeof(expected_types[0]); index++)
    {
        passed = passed
              && bno086_event_next(&imu, &cursor_a, &event, 0) > 0
              && event.type == expected_types[index];
    }
    passed = passed && bno086_event_next(&imu, &cursor_a, &event, 0) == 0;

    for (index = 0u; index < sizeof(expected_types) / sizeof(expected_types[0]); index++)
    {
        passed = passed
              && bno086_event_next(&imu, &cursor_b, &event, 0) > 0
              && event.type == expected_types[index];
    }
    passed = passed && cursor_a.dropped == 0u && cursor_b.dropped == 0u;

    wait_start = monotonic_ns();
    passed = passed && bno086_event_next(&imu, &cursor_b, &event, 10) == 0;
    wait_elapsed = monotonic_ns() - wait_start;
    passed = passed
          && wait_elapsed >= UINT64_C(5000000)
          && wait_elapsed < UINT64_C(200000000);

    passed = passed
          && bno086_event_cursor_init(&imu, &lagging_cursor, false) > 0;
    for (index = 0u; index < BNO086_EVENT_RING_CAPACITY + 3u; index++)
    {
        (void)make_vector_record(
            vector,
            BNO086_SENSOR_ACCELEROMETER,
            (uint8_t)(index & 0xFFu),
            2u,
            0,
            0,
            256
        );
        passed = passed
              && bno086_test_feed_packet(&imu, 3u, 2u, vector, sizeof(vector)) == 1;
    }
    passed = passed
          && bno086_event_next(&imu, &lagging_cursor, &event, 0) > 0
          && lagging_cursor.dropped == 3u;
    (void)bno086_get_diagnostics(&imu, &diagnostics);
    passed = passed && diagnostics.event_overwrite_count == 8u;

    bno086_deinit(&imu);
    return passed;
}

static int run_mock_tests(void)
{
    struct mock_case
    {
        const char *name;
        bool (*function)(void);
    };
    const struct mock_case cases[] = {
        {"Phase-2 null one-shot recovery", mock_null_retry_success},
        {"Phase-2 repeated null failure", mock_null_retry_failure},
        {"validation/bus error categories", mock_validation_and_bus_categories},
        {"partial continuation assembly", mock_multichunk_assembly},
        {"0us -> non-zero FC race", mock_feature_race},
        {"dynamic calibration command/response", mock_dynamic_calibration_command},
        {"parser + independent SPMC cursors", mock_parser_and_spmc_ring}
    };
    size_t index;
    size_t passed_count = 0u;

    puts("=== BNO086 C MOCK TEST ===");
    for (index = 0u; index < sizeof(cases) / sizeof(cases[0]); index++)
    {
        bool passed = cases[index].function();

        printf("[%s] %s\n", passed ? "PASS" : "FAIL", cases[index].name);
        if (passed)
        {
            passed_count++;
        }
    }
    printf("RESULT=%zu/%zu %s\n",
           passed_count,
           sizeof(cases) / sizeof(cases[0]),
           passed_count == sizeof(cases) / sizeof(cases[0]) ? "PASS" : "FAIL");
    return passed_count == sizeof(cases) / sizeof(cases[0]) ? 0 : 1;
}

#else

static int run_mock_tests(void)
{
    fputs("mock mode requires -DBNO086_TESTING\n", stderr);
    return 1;
}

#endif

static void print_usage(const char *program)
{
    fprintf(
        stderr,
        "Usage:\n"
        "  %s mock\n"
        "  %s feature [trials, default 10]\n"
        "  %s stability [seconds, default 60]\n"
        "  %s accel [seconds, default 30]\n",
        program,
        program,
        program,
        program
    );
}

int main(int argc, char **argv)
{
    struct sigaction action;

    memset(&action, 0, sizeof(action));
    action.sa_handler = handle_signal;
    (void)sigemptyset(&action.sa_mask);
    (void)sigaction(SIGINT, &action, NULL);
    (void)sigaction(SIGTERM, &action, NULL);

    if (argc < 2 || argc > 3)
    {
        print_usage(argv[0]);
        return 2;
    }

    if (strcmp(argv[1], "mock") == 0)
    {
        if (argc != 2)
        {
            print_usage(argv[0]);
            return 2;
        }
        return run_mock_tests();
    }
    if (strcmp(argv[1], "feature") == 0)
    {
        unsigned int trials = 10u;

        if (argc == 3 && !parse_positive_uint(argv[2], &trials))
        {
            fputs("invalid trial count\n", stderr);
            return 2;
        }
        return run_feature_race(trials);
    }
    if (strcmp(argv[1], "stability") == 0)
    {
        double duration = 60.0;

        if (argc == 3 && !parse_positive_double(argv[2], &duration))
        {
            fputs("invalid duration\n", stderr);
            return 2;
        }
        return monitor_streams(
            "60s BNO086 C STREAM TEST",
            fusion_streams,
            sizeof(fusion_streams) / sizeof(fusion_streams[0]),
            duration,
            false
        );
    }
    if (strcmp(argv[1], "accel") == 0)
    {
        double duration = 30.0;

        if (argc == 3 && !parse_positive_double(argv[2], &duration))
        {
            fputs("invalid duration\n", stderr);
            return 2;
        }
        return monitor_streams(
            "BNO086 ACCELERATION / SPMC TEST",
            acceleration_streams,
            sizeof(acceleration_streams) / sizeof(acceleration_streams[0]),
            duration,
            true
        );
    }

    print_usage(argv[0]);
    return 2;
}
