#!/usr/bin/env python3
"""Interactive BNO086 relative-heading closure checks without a PAA5100."""

from __future__ import annotations

import select
import sys
import time

from bno_test_support import ObservedBNO086, configure_one
from bno086 import (
    SENSOR_GYROSCOPE_CALIBRATED,
    SENSOR_MAGNETOMETER_CALIBRATED,
    SENSOR_ROTATION_VECTOR,
)
from read_flow import (
    IMU_STALE_TIMEOUT_S,
    RelativeHeadingTracker,
    angle_delta_deg,
)


# CEVA's calibration procedure requires the calibrated Magnetic Field report
# at 50 Hz.  The production fusion plan now uses the same calibration rate.
HEADING_REPORT_PLAN = (
    ("RV", SENSOR_ROTATION_VECTOR, 10_000, 0.75),
    ("GYRO", SENSOR_GYROSCOPE_CALIBRATED, 10_000, 0.75),
    ("MAG", SENSOR_MAGNETOMETER_CALIBRATED, 20_000, 1.00),
)


class SessionHealth:
    def __init__(self):
        self.stale_events = 0
        self._was_stale = False

    def observe(self, snapshot, now):
        stale = (
            not snapshot["startup_ready"]
            or snapshot["last_orientation_update"] <= 0.0
            or now - snapshot["last_orientation_update"] > IMU_STALE_TIMEOUT_S
        )
        if stale and not self._was_stale:
            self.stale_events += 1
        self._was_stale = stale
        return stale


def service_until_enter(imu, health, message, tracker=None):
    """Keep draining BNO packets while waiting for the operator's Enter."""
    print(message)
    previous_report_heading = None
    previous_report_sequence = None
    unwrapped_heading = 0.0
    while True:
        imu.update()
        now = time.monotonic()
        snapshot = imu.snapshot()
        stale = health.observe(snapshot, now)

        if tracker is not None and not stale:
            # Accumulate turns only from new reports.  Repeatedly accumulating
            # the tracker's short extrapolation between reports would count
            # the same angular motion more than once.
            report_heading = tracker.heading_at(
                snapshot, snapshot["last_orientation_update"]
            )
            report_sequence = snapshot["orientation_sequence"]
            if report_sequence != previous_report_sequence:
                if previous_report_heading is None:
                    unwrapped_heading = report_heading
                else:
                    unwrapped_heading += angle_delta_deg(
                        report_heading, previous_report_heading
                    )
                previous_report_heading = report_heading
                previous_report_sequence = report_sequence
            heading = tracker.heading_at(snapshot, now)
            print(
                f"\rrelative={heading:+8.3f} deg  "
                f"unwrapped={unwrapped_heading:+9.3f} deg  "
                f"RV/Mag={snapshot['rotation_accuracy']}/"
                f"{snapshot['mag_accuracy']}  "
                "[Enter=finish]",
                end="",
                flush=True,
            )

        ready, _, _ = select.select([sys.stdin], [], [], 0.01)
        if ready:
            sys.stdin.readline()
            if tracker is not None:
                print()
            return imu.snapshot()


def fresh_reference(imu, health, prompt):
    while True:
        snapshot = service_until_enter(imu, health, prompt)
        now = time.monotonic()
        if (
            snapshot["startup_ready"]
            and snapshot["last_orientation_update"] > 0.0
            and now - snapshot["last_orientation_update"] <= IMU_STALE_TIMEOUT_S
        ):
            return snapshot
        print("[!] Rotation Vector가 stale입니다. 스트림 복구 후 다시 Enter를 누르세요.")


def wait_for_calibration(imu, health):
    service_until_enter(
        imu,
        health,
        "센서를 움직이지 않는 평평한 곳에 놓고 Enter를 누르세요. "
        "Gyro를 3초간 보정합니다.",
    )
    stationary_deadline = time.monotonic() + 3.0
    while time.monotonic() < stationary_deadline:
        imu.update()
        now = time.monotonic()
        health.observe(imu.snapshot(), now)
        remaining = max(0.0, stationary_deadline - now)
        print(
            f"\rGYRO STATIONARY CALIBRATION: {remaining:3.1f}s remaining",
            end="",
            flush=True,
        )
        time.sleep(0.02)
    print("\nGyro 정지 보정 구간이 끝났습니다.")
    print()

    print("Magnetometer 보정 동작:")
    print("  1. Roll 축으로 약 180° 회전했다가 시작 자세로 복귀")
    print("  2. Pitch 축으로 약 180° 회전했다가 시작 자세로 복귀")
    print("  3. Yaw 축으로 약 180° 회전했다가 시작 자세로 복귀")
    print("각 축 왕복을 약 2초 속도로 천천히 반복하세요.")
    print("Magnetic Field report는 보정에 필요한 50Hz로 설정되어 있습니다.")
    print("RV와 Mag가 모두 2 이상일 때 Enter를 누르면 closure 시험을 시작합니다.")
    next_status_at = 0.0
    while True:
        imu.update()
        now = time.monotonic()
        snapshot = imu.snapshot()
        stale = health.observe(snapshot, now)
        if now >= next_status_at:
            print(
                f"\rCALIBRATION RV/Mag={snapshot['rotation_accuracy']}/"
                f"{snapshot['mag_accuracy']}  "
                f"stream={'STALE' if stale else 'OK'}  "
                "[Enter=check]",
                end="",
                flush=True,
            )
            next_status_at = now + 0.20
        ready, _, _ = select.select([sys.stdin], [], [], 0.02)
        if ready:
            sys.stdin.readline()
            print()
            if (
                not stale
                and snapshot["rotation_accuracy"] >= 2
                and snapshot["mag_accuracy"] >= 2
            ):
                print("RV/Mag accuracy가 closure 시험 기준(2/2 이상)에 도달했습니다.")
                print()
                return snapshot
            print(
                "[!] 현재 accuracy는 자력계 보정 완료로 간주하지 않습니다. "
                "8자 회전을 계속한 뒤 다시 Enter를 누르세요."
            )


def main():
    print("=== HEADING CLOSURE TEST ===")
    print("센서를 수평으로 두고 주변 자성체/전원선에서 가능한 한 떨어뜨리세요.")
    print("각 회전은 지정 방향과 횟수로 수행한 뒤 물리적 시작 방향에 정확히 맞춥니다.")
    print("대기 중에도 BNO086 패킷을 계속 읽으므로 안내가 나온 뒤 Enter를 누르세요.")
    print()

    imu = ObservedBNO086(i2c_bus=1, i2c_addr=0x4B, int_pin=18, rst_pin=23)
    health = SessionHealth()
    results = []
    try:
        if not imu.begin():
            print(f"setup_error={imu.snapshot()['boot_failure_reason']}")
            return 1

        for name, sensor_id, requested, report_timeout_s in HEADING_REPORT_PLAN:
            result = configure_one(imu, sensor_id, requested, report_timeout_s)
            if not (result["enabled"] and result["report"]):
                print(
                    f"setup_error={name}: "
                    f"{imu.snapshot()['boot_failure_reason'] or 'unknown failure'}"
                )
                return 1
            print(
                f"{name} requested={requested} us  "
                f"applied={result['applied']} us  report=YES"
            )

        if not imu.start_dynamic_calibration():
            print(
                "setup_error=동적 보정 시작 응답을 받지 못해 "
                "heading closure를 시작하지 않습니다."
            )
            return 1

        print("동적 Gyro/Magnetometer 보정을 시작했습니다.")
        wait_for_calibration(imu, health)

        tests = (
            ("CW 360", "시작 방향에서 센서를 정지시킨 뒤 Enter를 누르세요."),
            ("CCW 360", "센서를 같은 시작 방향에 정지시킨 뒤 Enter를 누르세요."),
            ("CW 1080", "센서를 같은 시작 방향에 정지시킨 뒤 Enter를 누르세요."),
        )

        initial_snapshot = None
        for label, reference_prompt in tests:
            reference = fresh_reference(imu, health, f"[{label}] {reference_prompt}")
            if initial_snapshot is None:
                initial_snapshot = reference
                print(f"start absolute yaw = {reference['yaw']:+.3f} deg")
                print(
                    "start accuracy RV/Mag = "
                    f"{reference['rotation_accuracy']}/{reference['mag_accuracy']}"
                )
                print()

            tracker = RelativeHeadingTracker(reference)
            final_snapshot = service_until_enter(
                imu,
                health,
                f"[{label}] 지금 회전하고 시작 방향에 다시 맞춘 뒤 Enter를 누르세요.",
                tracker=tracker,
            )
            final_heading = tracker.heading_at(final_snapshot, time.monotonic())
            closure_error = abs(final_heading)
            results.append((label, final_heading, closure_error))
            print(label)
            print(f"final relative heading = {final_heading:+.3f} deg")
            print(f"closure error = {closure_error:.3f} deg")
            print()

        state = imu.snapshot()
        print(f"reset_count={state['reset_count']}")
        print(f"stale_events={health.stale_events}")
        return 0 if state["reset_count"] == 0 and health.stale_events == 0 else 1
    except KeyboardInterrupt:
        print("\n측정이 사용자에 의해 중단되었습니다.")
        return 130
    finally:
        imu.close()


if __name__ == "__main__":
    raise SystemExit(main())
