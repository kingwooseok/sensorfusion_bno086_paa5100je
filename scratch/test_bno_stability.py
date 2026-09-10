#!/usr/bin/env python3
"""Measure three BNO086 streams using the production driver for 60 seconds."""

from __future__ import annotations

import argparse
import time

from bno_test_support import ObservedBNO086, REPORT_PLAN, configure_one
from bno086 import SENSOR_MAGNETOMETER_CALIBRATED


STALE_LIMIT_S = {
    0x05: 0.20,
    0x02: 0.20,
    0x03: 0.50,
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--mag-interval-us", type=int, default=40_000)
    return parser.parse_args()


def max_gap_seconds(timestamps, start, end):
    observed = [value for value in timestamps if start <= value <= end]
    boundaries = [start, *observed, end]
    return max(
        (later - earlier for earlier, later in zip(boundaries, boundaries[1:])),
        default=end - start,
    )


def print_stream(name, reports, elapsed, applied, max_gap_s):
    measured_rate = reports / elapsed if elapsed > 0.0 else 0.0
    expected_rate = 1_000_000.0 / applied if applied else 0.0
    print(name)
    print(f"reports={reports}")
    print(f"applied_interval={applied if applied is not None else 'None'} us")
    print(f"expected_rate={expected_rate:.3f} Hz")
    print(f"measured_rate={measured_rate:.3f} Hz")
    print(f"max_gap={max_gap_s * 1000.0:.3f} ms")
    print()
    return measured_rate, expected_rate


def main():
    args = parse_args()
    report_plan = tuple(
        (name, sensor_id, args.mag_interval_us, timeout_s)
        if sensor_id == SENSOR_MAGNETOMETER_CALIBRATED
        else (name, sensor_id, interval_us, timeout_s)
        for name, sensor_id, interval_us, timeout_s in REPORT_PLAN
    )
    imu = ObservedBNO086(i2c_bus=1, i2c_addr=0x4B, int_pin=18, rst_pin=23)
    setup_rows = {}
    exit_code = 1
    try:
        if not imu.begin():
            print("=== 60s BNO086 STREAM TEST ===")
            print("duration=0.000s")
            print(f"setup_error={imu.snapshot()['boot_failure_reason']}")
            print("RESULT=FAIL")
            return 1

        for name, sensor_id, requested, report_timeout_s in report_plan:
            row = configure_one(imu, sensor_id, requested, report_timeout_s)
            setup_rows[sensor_id] = row
            if not (row["enabled"] and row["report"]):
                print("=== 60s BNO086 STREAM TEST ===")
                print("duration=0.000s")
                print(
                    f"setup_error={name}: "
                    f"{imu.snapshot()['boot_failure_reason'] or 'unknown failure'}"
                )
                print("RESULT=FAIL")
                return 1

        initial = imu.snapshot()
        sequence_keys = {
            0x05: "orientation_sequence",
            0x02: "gyro_sequence",
            0x03: "mag_sequence",
        }
        update_keys = {
            0x05: "last_orientation_update",
            0x02: "last_gyro_update",
            0x03: "last_mag_update",
        }
        baseline_sequences = {
            sensor_id: initial[key]
            for sensor_id, key in sequence_keys.items()
        }
        report_log_starts = {
            sensor_id: len(imu.report_times[sensor_id])
            for sensor_id in sequence_keys
        }
        stale_flags = {
            sensor_id: False for sensor_id in sequence_keys
        }
        stale_events = 0
        startup_losses = 0
        startup_was_ready = initial["startup_ready"]

        start = time.monotonic()
        deadline = start + args.duration
        while time.monotonic() < deadline:
            imu.update()
            now = time.monotonic()
            state = imu.snapshot()

            startup_ready = state["startup_ready"]
            if startup_was_ready and not startup_ready:
                startup_losses += 1
            startup_was_ready = startup_ready

            for sensor_id, update_key in update_keys.items():
                stale = (
                    state[update_key] <= 0.0
                    or now - state[update_key] > STALE_LIMIT_S[sensor_id]
                )
                if stale and not stale_flags[sensor_id]:
                    stale_events += 1
                stale_flags[sensor_id] = stale
            time.sleep(0.001)

        end = time.monotonic()
        elapsed = end - start
        final = imu.snapshot()
        reports = {
            sensor_id: final[key] - baseline_sequences[sensor_id]
            for sensor_id, key in sequence_keys.items()
        }
        applied = {
            sensor_id: final["feature_responses"].get(sensor_id)
            for sensor_id in sequence_keys
        }
        gaps = {
            sensor_id: max_gap_seconds(
                imu.report_times[sensor_id][report_log_starts[sensor_id]:],
                start,
                end,
            )
            for sensor_id in sequence_keys
        }

        print("=== 60s BNO086 STREAM TEST ===")
        print(f"duration={elapsed:.3f}s")
        print()
        rv_rate, rv_expected = print_stream(
            "Rotation Vector", reports[0x05], elapsed, applied[0x05], gaps[0x05]
        )
        gyro_rate, gyro_expected = print_stream(
            "Gyroscope", reports[0x02], elapsed, applied[0x02], gaps[0x02]
        )
        mag_rate, mag_expected = print_stream(
            "Magnetometer", reports[0x03], elapsed, applied[0x03], gaps[0x03]
        )
        print(f"reset_count={final['reset_count']}")
        print(f"i2c_syscall_errors={len(imu.i2c_syscall_errors)}")
        print(f"shtp_validation_errors={len(imu.shtp_validation_errors)}")
        print(f"other_transport_errors={len(imu.other_transport_errors)}")
        print(f"phase2_null_count={final['phase2_null_count']}")
        print(
            "phase2_null_retry_success="
            f"{final['phase2_null_retry_success']}"
        )
        print(f"phase2_null_retry_fail={final['phase2_null_retry_fail']}")
        print(f"startup_losses={startup_losses}")
        print(f"stale_events={stale_events}")
        print(f"RV_accuracy={final['rotation_accuracy']}")
        print(f"Mag_accuracy={final['mag_accuracy']}")
        print()

        rates_ok = all(
            expected > 0.0 and 0.80 * expected <= measured <= 1.20 * expected
            for measured, expected in (
                (rv_rate, rv_expected),
                (gyro_rate, gyro_expected),
                (mag_rate, mag_expected),
            )
        )
        gaps_ok = all(
            gaps[sensor_id] <= STALE_LIMIT_S[sensor_id]
            for sensor_id in gaps
        )
        passed = (
            rates_ok
            and gaps_ok
            and final["reset_count"] == 0
            and not imu.i2c_syscall_errors
            and not imu.shtp_validation_errors
            and not imu.other_transport_errors
            and final["phase2_null_retry_fail"] == 0
            and startup_losses == 0
            and stale_events == 0
            and final["startup_ready"]
        )
        print(f"RESULT={'PASS' if passed else 'FAIL'}")
        exit_code = 0 if passed else 1
    finally:
        imu.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
