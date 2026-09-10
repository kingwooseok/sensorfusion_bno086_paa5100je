#!/usr/bin/env python3
"""Repeat BNO086 reset/setup and expose every FC interval in arrival order."""

from __future__ import annotations

import argparse
import time

from bno_test_support import (
    ObservedBNO086,
    REPORT_PLAN,
    applied_state_is_valid,
    configure_one,
)
from bno086 import SENSOR_MAGNETOMETER_CALIBRATED


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--mag-interval-us", type=int, default=40_000)
    return parser.parse_args()


def main():
    args = parse_args()
    report_plan = tuple(
        (name, sensor_id, args.mag_interval_us, timeout_s)
        if sensor_id == SENSOR_MAGNETOMETER_CALIBRATED
        else (name, sensor_id, interval_us, timeout_s)
        for name, sensor_id, interval_us, timeout_s in REPORT_PLAN
    )
    print("=== FEATURE RACE TEST ===")

    setup_success = 0
    false_negative = 0
    total_resets = 0
    total_syscall_errors = 0
    total_validation_errors = 0
    total_other_errors = 0
    total_phase2_nulls = 0
    total_null_retry_success = 0
    total_null_retry_fail = 0

    for trial_number in range(1, args.trials + 1):
        imu = ObservedBNO086(i2c_bus=1, i2c_addr=0x4B, int_pin=18, rst_pin=23)
        rows = []
        print(f"Trial {trial_number:02d}")
        try:
            began = imu.begin()
            for name, sensor_id, requested, report_timeout_s in report_plan:
                if began and imu.snapshot()["startup_ready"]:
                    row = configure_one(
                        imu, sensor_id, requested, report_timeout_s
                    )
                else:
                    row = {
                        "enabled": False,
                        "fc_seen": [],
                        "applied": None,
                        "report": False,
                    }

                valid_applied = applied_state_is_valid(requested, row["applied"])
                passed = row["enabled"] and valid_applied and row["report"]
                if not row["enabled"] and (valid_applied or row["report"]):
                    false_negative += 1
                rows.append(passed)

                applied_text = (
                    str(row["applied"])
                    if row["applied"] is not None
                    else "None"
                )
                print(
                    f"{name:<4} requested={requested:5d} us  "
                    f"FC_seen={row['fc_seen']!s:<18} "
                    f"enable={'TRUE' if row['enabled'] else 'FALSE':<5}  "
                    f"applied={applied_text:<6}  "
                    f"report={'YES' if row['report'] else 'NO':<3}  "
                    f"{'PASS' if passed else 'FAIL'}"
                )

            state = imu.snapshot()
            if began and all(rows):
                setup_success += 1
            total_resets += state["reset_count"]
            total_syscall_errors += len(imu.i2c_syscall_errors)
            total_validation_errors += len(imu.shtp_validation_errors)
            total_other_errors += len(imu.other_transport_errors)
            total_phase2_nulls += state["phase2_null_count"]
            total_null_retry_success += state["phase2_null_retry_success"]
            total_null_retry_fail += state["phase2_null_retry_fail"]
            print(
                f"reset_count={state['reset_count']}  "
                f"i2c_syscall_errors={len(imu.i2c_syscall_errors)}  "
                f"shtp_validation_errors={len(imu.shtp_validation_errors)}"
            )
            print(
                f"phase2_null_count={state['phase2_null_count']}  "
                f"null_retry_success={state['phase2_null_retry_success']}  "
                f"null_retry_fail={state['phase2_null_retry_fail']}"
            )
            if not began or not all(rows):
                print(
                    "failure_reason="
                    f"{state['boot_failure_reason'] or 'unknown failure'}"
                )
        except Exception as error:
            state = imu.snapshot()
            total_resets += state["reset_count"]
            total_syscall_errors += len(imu.i2c_syscall_errors)
            total_validation_errors += len(imu.shtp_validation_errors)
            total_other_errors += len(imu.other_transport_errors)
            total_phase2_nulls += state["phase2_null_count"]
            total_null_retry_success += state["phase2_null_retry_success"]
            total_null_retry_fail += state["phase2_null_retry_fail"]
            print(f"trial_exception={type(error).__name__}: {error}")
        finally:
            imu.close()
        print()
        if trial_number != args.trials:
            time.sleep(0.10)

    print("SUMMARY")
    print(f"trials={args.trials}")
    print(f"setup_success={setup_success}/{args.trials}")
    print(f"false_negative={false_negative}")
    print(f"resets={total_resets}")
    print(f"i2c_syscall_errors={total_syscall_errors}")
    print(f"shtp_validation_errors={total_validation_errors}")
    print(f"other_transport_errors={total_other_errors}")
    print(f"phase2_null_count={total_phase2_nulls}")
    print(f"phase2_null_retry_success={total_null_retry_success}")
    print(f"phase2_null_retry_fail={total_null_retry_fail}")
    passed = (
        setup_success == args.trials
        and false_negative == 0
        and total_resets == 0
        and total_syscall_errors == 0
        and total_validation_errors == 0
        and total_other_errors == 0
        and total_null_retry_fail == 0
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
