#!/usr/bin/env python3
"""Exercise the existing read_flow fusion math without PAA5100 hardware."""

from __future__ import annotations

import contextlib
import io
import math
import re
import threading
import time
from unittest import mock

import bno_test_support  # noqa: F401  # Adds the project root to sys.path.
from read_flow import (
    AutoYawFlowCompensator,
    RelativeHeadingTracker,
    angle_delta_deg,
    run_measurement,
)


START_YAWS = (0.0, 37.0, 123.0, -170.0)


def yaw_quaternion(yaw_deg):
    half_yaw = math.radians(yaw_deg) * 0.5
    return math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)


def heading_snapshot(yaw_deg, sequence, timestamp):
    real, i, j, k = yaw_quaternion(yaw_deg)
    return {
        "quat_real": real,
        "quat_i": i,
        "quat_j": j,
        "quat_k": k,
        "orientation_sequence": sequence,
        "last_orientation_update": timestamp,
    }


class SyntheticImu:
    """Minimal state source used to drive read_flow.run_measurement itself."""

    def __init__(self, initial_yaw):
        self.initial_yaw = initial_yaw
        self.relative_yaw = 0.0
        self.sequence = 1
        now = time.monotonic()
        self.orientation_time = now
        self.gyro_time = now
        self.mag_time = now
        self._lock = threading.Lock()

    def set_relative_yaw(self, relative_yaw):
        with self._lock:
            self.relative_yaw = relative_yaw
            self.sequence += 1
            self.orientation_time = time.monotonic()

    def update(self):
        return False

    def tare_all_axes(self):
        raise AssertionError("production measurement must not call hardware tare")

    def publish_orientation(self):
        with self._lock:
            self.sequence += 1
            now = time.monotonic()
            self.orientation_time = now
            self.gyro_time = now
            self.mag_time = now
            return self.snapshot_unlocked()

    def snapshot_unlocked(self):
        real, i, j, k = yaw_quaternion(
            self.initial_yaw + self.relative_yaw
        )
        return {
            "startup_ready": True,
            "boot_failure_reason": None,
            "orientation_sequence": self.sequence,
            "gyro_sequence": 1,
            "mag_sequence": 1,
            "last_orientation_update": self.orientation_time,
            "last_gyro_update": self.gyro_time,
            "last_mag_update": self.mag_time,
            "quat_real": real,
            "quat_i": i,
            "quat_j": j,
            "quat_k": k,
            "yaw": self.initial_yaw + self.relative_yaw,
            "gyro_x": 0.0,
            "gyro_y": 0.0,
            "gyro_z": 0.0,
            "rotation_accuracy": 3,
            "mag_accuracy": 3,
        }

    def snapshot(self):
        with self._lock:
            return self.snapshot_unlocked()


class SyntheticSquareFlow:
    """Ideal body-forward samples plus in-place 90-degree heading updates."""

    def __init__(self, imu):
        self.imu = imu
        self.height_cm = 1.0
        self.scaler = 1.0
        self.last_squal = 255
        self._cleared = False
        self._index = 0
        self._events = (
            (0.0, 100, 0),
            (90.0, 0, 0),
            (90.0, 100, 0),
            (180.0, 0, 0),
            (180.0, 100, 0),
            (270.0, 0, 0),
            (270.0, 100, 0),
        )

    def read_motion_count(self):
        if not self._cleared:
            self._cleared = True
            return 0, 0
        if self._index >= len(self._events):
            raise KeyboardInterrupt
        relative_yaw, dx, dy = self._events[self._index]
        self._index += 1
        self.imu.set_relative_yaw(relative_yaw)
        return dx, dy


def run_square_through_production_loop(initial_yaw):
    """Run the actual run_measurement body-to-world block and parse its summary."""
    imu = SyntheticImu(initial_yaw)
    flow = SyntheticSquareFlow(imu)
    captured = io.StringIO()
    with (
        contextlib.redirect_stdout(captured),
        mock.patch("builtins.input", return_value=""),
        mock.patch("read_flow.select.select", return_value=([], [], [])),
        mock.patch(
            "read_flow.wait_for_new_orientation_report",
            side_effect=lambda target, _sequence, timeout_s=0.75: (
                target.publish_orientation()
            ),
        ),
    ):
        run_measurement(flow, imu)

    match = re.search(
        r"최종 좌표 \(X, Y\) : \(\s*([+-]?\d+(?:\.\d+)?) cm,\s*"
        r"([+-]?\d+(?:\.\d+)?) cm\s*\)",
        captured.getvalue(),
    )
    if match is None:
        raise RuntimeError("could not read run_measurement final coordinates")
    return float(match.group(1)), float(match.group(2))


def main():
    all_passed = True
    print("=== FUSION MATH TEST ===")
    print()

    print("[ANGLE WRAP]")
    wrapped_delta = angle_delta_deg(-179.0, 179.0)
    passed = abs(wrapped_delta - 2.0) < 1e-9
    all_passed &= passed
    print(
        f"179 -> -179 : delta={wrapped_delta:+.3f} deg : "
        f"{'PASS' if passed else 'FAIL'}"
    )
    print()

    print("[RELATIVE QUATERNION]")
    for initial_yaw in START_YAWS:
        reference = heading_snapshot(initial_yaw, 1, 1.0)
        current = heading_snapshot(initial_yaw + 90.0, 2, 2.0)
        tracker = RelativeHeadingTracker(reference)
        relative_heading = tracker.heading_at(current, 2.0)
        passed = abs(relative_heading - 90.0) < 1e-6
        all_passed &= passed
        print(
            f"start={initial_yaw:4.0f} deg -> "
            f"relative90={relative_heading:+.3f} : "
            f"{'PASS' if passed else 'FAIL'}"
        )
    print()

    print("[SQUARE CLOSURE]")
    for initial_yaw in START_YAWS:
        x_cm, y_cm = run_square_through_production_loop(initial_yaw)
        closure_error = math.hypot(x_cm, y_cm)
        passed = closure_error < 0.01
        all_passed &= passed
        print(
            f"start_yaw={initial_yaw:4.0f} : "
            f"X={x_cm:+.3f} Y={y_cm:+.3f} "
            f"error={closure_error:.3f} : "
            f"{'PASS' if passed else 'FAIL'}"
        )
    print()

    print("[LEVER ARM]")
    compensator = AutoYawFlowCompensator()
    lever_x_cm = 5.0
    lever_y_cm = -3.0
    compensator.start_calibration()
    for sample_index in range(40):
        delta_yaw_rad = math.radians(2.0 if sample_index % 2 == 0 else -2.0)
        raw_dx = lever_x_cm * delta_yaw_rad
        raw_dy = lever_y_cm * delta_yaw_rad
        corrected_dx, corrected_dy = compensator.compensate(
            raw_dx,
            raw_dy,
            delta_yaw_rad,
            gyro_magnitude=1.0,
        )
        if corrected_dx != 0.0 or corrected_dy != 0.0:
            raise AssertionError("calibration displacement entered position")
    calibration_frozen = compensator.finish_calibration()

    frozen_sample_count = compensator.sample_count
    frozen_x_per_rad = compensator.x_per_rad
    frozen_y_per_rad = compensator.y_per_rad
    raw_squared = []
    residual_squared = []
    # Evaluate on held-out samples after the estimate has been frozen.
    for sample_index in range(20):
        delta_yaw_rad = math.radians(2.0 if sample_index % 2 == 0 else -2.0)
        raw_dx = lever_x_cm * delta_yaw_rad
        raw_dy = lever_y_cm * delta_yaw_rad
        corrected_dx, corrected_dy = compensator.compensate(
            raw_dx,
            raw_dy,
            delta_yaw_rad,
            gyro_magnitude=1.0,
        )
        raw_squared.append(raw_dx * raw_dx + raw_dy * raw_dy)
        residual_squared.append(
            corrected_dx * corrected_dx + corrected_dy * corrected_dy
        )

    raw_rms = math.sqrt(sum(raw_squared) / len(raw_squared))
    residual_rms = math.sqrt(sum(residual_squared) / len(residual_squared))
    lever_passed = (
        calibration_frozen
        and compensator.sample_count == frozen_sample_count
        and compensator.x_per_rad == frozen_x_per_rad
        and compensator.y_per_rad == frozen_y_per_rad
        and residual_rms < raw_rms * 0.05
    )
    all_passed &= lever_passed
    print(f"samples={compensator.sample_count}")
    print(f"raw_rotation_flow={raw_rms:.6f} cm RMS")
    print(f"corrected_residual={residual_rms:.6f} cm RMS")
    print("PASS" if lever_passed else "FAIL")
    print()
    print(f"FINAL RESULT={'PASS' if all_passed else 'FAIL'}")
    return 0 if all_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
