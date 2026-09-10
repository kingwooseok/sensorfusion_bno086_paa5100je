#!/usr/bin/env python3
"""Exercise read_flow's production control/position paths without a PAA5100."""

from __future__ import annotations

import contextlib
import io
import math
import re
import struct
import threading
import time
from unittest import mock

import bno_test_support  # noqa: F401  # Adds the project root to sys.path.
import pmw3901
import read_flow
from bno086 import (
    SENSOR_GYROSCOPE_CALIBRATED,
    SENSOR_MAGNETOMETER_CALIBRATED,
    SENSOR_ROTATION_VECTOR,
)


def yaw_quaternion(yaw_deg):
    half_yaw = math.radians(yaw_deg) * 0.5
    return math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)


class SyntheticImu:
    """Thread-safe BNO snapshot source driven by optical-flow test events."""

    def __init__(self, initial_yaw=0.0):
        self.initial_yaw = initial_yaw
        self.relative_yaw = 0.0
        self.sequence = 1
        self.startup_ready = True
        self.gyro = (0.0, 0.0, 0.0)
        self.rotation_accuracy = 3
        self.mag_accuracy = 3
        now = time.monotonic()
        self.orientation_time = now
        self.gyro_time = now
        self.mag_time = now
        self._lock = threading.Lock()

    def apply(self, event):
        with self._lock:
            self.relative_yaw = event.get("yaw", self.relative_yaw)
            self.sequence += 1
            self.startup_ready = event.get("startup_ready", True)
            self.gyro = event.get("gyro", (0.0, 0.0, 0.0))
            self.rotation_accuracy = event.get(
                "rotation_accuracy", self.rotation_accuracy
            )
            self.mag_accuracy = event.get("mag_accuracy", self.mag_accuracy)
            now = time.monotonic()
            if event.get("fresh", True):
                self.orientation_time = now
                self.gyro_time = now
                self.mag_time = now
            else:
                self.orientation_time = now - 1.0
                self.gyro_time = now - 1.0
                self.mag_time = now - 1.0

    def update(self):
        return False

    def tare_all_axes(self):
        raise AssertionError("production measurement must not call hardware tare")

    def publish_orientation(self, yaw=None, *, refresh_gyro=True, refresh_mag=True):
        with self._lock:
            if yaw is not None:
                self.relative_yaw = yaw
            self.sequence += 1
            now = time.monotonic()
            self.orientation_time = now
            if refresh_gyro:
                self.gyro_time = now
            if refresh_mag:
                self.mag_time = now

    def status_summary(self):
        return "synthetic"

    def snapshot(self):
        with self._lock:
            real, i, j, k = yaw_quaternion(
                self.initial_yaw + self.relative_yaw
            )
            gyro_x, gyro_y, gyro_z = self.gyro
            return {
                "startup_ready": self.startup_ready,
                "boot_failure_reason": None,
                "orientation_sequence": self.sequence,
                "gyro_sequence": self.sequence,
                "mag_sequence": self.sequence,
                "last_orientation_update": self.orientation_time,
                "last_gyro_update": self.gyro_time,
                "last_mag_update": self.mag_time,
                "quat_real": real,
                "quat_i": i,
                "quat_j": j,
                "quat_k": k,
                "yaw": self.initial_yaw + self.relative_yaw,
                "gyro_x": gyro_x,
                "gyro_y": gyro_y,
                "gyro_z": gyro_z,
                "rotation_accuracy": self.rotation_accuracy,
                "mag_accuracy": self.mag_accuracy,
            }


class ScenarioFlow:
    """Finite PAA-like event stream that terminates run_measurement cleanly."""

    def __init__(self, events, imu=None, height_cm=1.0, scaler=1.0):
        self.events = list(events)
        self.imu = imu
        self.height_cm = height_cm
        self.scaler = scaler
        self.last_squal = 255
        self._cleared = False
        self._index = 0

    def read_motion_count(self):
        # run_measurement deliberately clears one latched delta before starting.
        if not self._cleared:
            self._cleared = True
            return 0, 0
        if self._index >= len(self.events):
            raise KeyboardInterrupt
        event = self.events[self._index]
        self._index += 1
        self.last_squal = event.get("q", 255)
        if self.imu is not None:
            self.imu.apply(event)
        return event.get("dx", 0), event.get("dy", 0)


class ResetSelector:
    def __init__(self, stream):
        self.stream = stream
        self.calls = 0

    def __call__(self, *_args, **_kwargs):
        self.calls += 1
        if self.calls == 1:
            return [self.stream], [], []
        return [], [], []


class CommandSelector:
    """Expose scripted stdin commands after selected production-loop samples."""

    def __init__(self, stream, schedule):
        self.stream = stream
        self.schedule = dict(schedule)
        self.calls = 0

    def __call__(self, *_args, **_kwargs):
        self.calls += 1
        callback = self.schedule.get(self.calls)
        if callback is None and self.calls not in self.schedule:
            return [], [], []
        if callback is not None:
            callback()
        return [self.stream], [], []


def parse_summary(output):
    position = re.search(
        r"최종 좌표 \(X, Y\) : \(\s*([+-]?\d+(?:\.\d+)?) cm,\s*"
        r"([+-]?\d+(?:\.\d+)?) cm\s*\)",
        output,
    )
    ticks = re.search(r"총 원시 틱\s*: X = ([+-]?\d+), Y = ([+-]?\d+)", output)
    samples = re.search(r"회전 오프셋 학습 : (\d+) samples", output)
    accuracy_low = re.search(r"운행 중 accuracy 저하: (\d+) events", output)
    if position is None or ticks is None:
        raise AssertionError("run_measurement summary could not be parsed")
    return {
        "x": float(position.group(1)),
        "y": float(position.group(2)),
        "ticks_x": int(ticks.group(1)),
        "ticks_y": int(ticks.group(2)),
        "samples": int(samples.group(1)) if samples else None,
        "accuracy_low_events": int(accuracy_low.group(1)) if accuracy_low else None,
        "output": output,
    }


def run_scenario(
    events,
    *,
    imu=None,
    height_cm=1.0,
    scaler=1.0,
    reset=False,
    compensator=None,
    reference_yaw=None,
    commands=(),
):
    flow = ScenarioFlow(events, imu=imu, height_cm=height_cm, scaler=scaler)
    captured = io.StringIO()
    if commands:
        fake_stdin = io.StringIO("".join(f"{command}\n" for _, command, _ in commands))
        selector = CommandSelector(
            fake_stdin,
            ((call_number, callback) for call_number, _, callback in commands),
        )
    else:
        fake_stdin = io.StringIO("r\n" if reset else "")
        selector = ResetSelector(fake_stdin) if reset else lambda *_a, **_k: ([], [], [])

    def next_reference_report(target_imu, _previous_sequence, timeout_s=0.75):
        del timeout_s
        target_imu.publish_orientation(reference_yaw)
        return target_imu.snapshot()

    compensator_patch = (
        mock.patch("read_flow.AutoYawFlowCompensator", return_value=compensator)
        if compensator is not None
        else contextlib.nullcontext()
    )
    with (
        contextlib.redirect_stdout(captured),
        mock.patch("builtins.input", return_value=""),
        mock.patch.object(read_flow.sys, "stdin", fake_stdin),
        mock.patch("read_flow.select.select", side_effect=selector),
        mock.patch(
            "read_flow.wait_for_new_orientation_report",
            side_effect=next_reference_report,
        ),
        compensator_patch,
    ):
        read_flow.run_measurement(flow, imu)
    return parse_summary(captured.getvalue())


def assert_close(actual, expected, tolerance=0.02):
    if abs(actual - expected) > tolerance:
        raise AssertionError(f"expected {expected:.6f}, got {actual:.6f}")


def test_paa_burst_decode():
    sensor = pmw3901.PMW3901.__new__(pmw3901.PMW3901)
    sensor.cs_device = None
    sensor.last_squal = 0
    sensor.last_shutter = 0
    sensor._transfer_with_turnaround = mock.Mock(
        return_value=struct.pack(
            "<BBhhBBBBBB",
            0x80,
            0x00,
            -1234,
            2345,
            77,
            0,
            0,
            0,
            0x12,
            0x34,
        )
    )
    dx, dy = sensor.read_motion_count()
    if (dx, dy, sensor.last_squal, sensor.last_shutter) != (
        -1234,
        2345,
        77,
        0x1234,
    ):
        raise AssertionError("signed delta/SQUAL/shutter burst decode mismatch")
    sensor._transfer_with_turnaround.assert_called_once_with(
        bytes((sensor.REG_MOTION_BURST,)),
        bytes(12),
        sensor.MOTION_BURST_ADDRESS_DELAY_US,
        read=True,
    )


def test_standalone_filters_and_scale():
    state = run_scenario(
        (
            {"dx": 10, "dy": 0, "q": read_flow.MIN_SQUAL_FOR_POSITION - 1},
            {"dx": 1, "dy": -1, "q": 255},
            {"dx": 4, "dy": -4, "q": read_flow.MIN_SQUAL_FOR_POSITION},
        ),
        height_cm=2.0,
        scaler=4.0,
    )
    assert_close(state["x"], 2.0)
    assert_close(state["y"], -2.0)
    if (state["ticks_x"], state["ticks_y"]) != (15, -5):
        raise AssertionError("raw diagnostic tick totals were not retained")


def test_runtime_origin_reset():
    state = run_scenario(
        ({"dx": 10, "dy": 5}, {"dx": 4, "dy": -2}),
        reset=True,
    )
    assert_close(state["x"], 4.0)
    assert_close(state["y"], -2.0)
    if (state["ticks_x"], state["ticks_y"]) != (4, -2):
        raise AssertionError("r command did not reset raw tick totals")


def test_absolute_yaw_offset_and_world_rotation():
    for initial_yaw in (0.0, 37.0, 123.0, -170.0):
        imu = SyntheticImu(initial_yaw)
        state = run_scenario(
            ({"yaw": 90.0}, {"yaw": 90.0, "dx": 10}),
            imu=imu,
        )
        assert_close(state["x"], 0.0)
        assert_close(state["y"], 10.0)


def test_wrap_and_heading_midpoint():
    imu = SyntheticImu(37.0)
    state = run_scenario(
        ({"yaw": 179.0}, {"yaw": 181.0, "dx": 100}),
        imu=imu,
    )
    # The second sample crosses +179 -> -179.  Its +2-degree midpoint is 180.
    assert_close(state["x"], -100.0)
    assert_close(state["y"], 0.0)


def test_low_quality_heading_sync():
    imu = SyntheticImu()
    state = run_scenario(
        (
            {"yaw": 90.0, "dx": 100, "q": read_flow.MIN_SQUAL_FOR_POSITION - 1},
            {"yaw": 90.0, "dx": 10, "q": 255},
        ),
        imu=imu,
    )
    # The rejected turn sample must still advance heading; otherwise the next
    # good forward sample would be incorrectly rotated by the 45-degree midpoint.
    assert_close(state["x"], 0.0)
    assert_close(state["y"], 10.0)


def test_stale_discard_and_reanchor():
    imu = SyntheticImu()
    state = run_scenario(
        (
            {"yaw": 0.0, "dx": 10},
            {"yaw": 90.0, "dx": 100, "fresh": False},
            {"yaw": 90.0, "dx": 10, "fresh": True},
        ),
        imu=imu,
    )
    # The stale 100 ticks are discarded.  Recovery is re-anchored at 90 degrees.
    assert_close(state["x"], 10.0)
    assert_close(state["y"], 10.0)


def test_low_quality_plus_stale_reanchor():
    imu = SyntheticImu()
    state = run_scenario(
        (
            {"yaw": 0.0, "dx": 10},
            {
                "yaw": 90.0,
                "dx": 100,
                "q": read_flow.MIN_SQUAL_FOR_POSITION - 1,
                "fresh": False,
            },
            {"yaw": 90.0, "dx": 10, "q": 255, "fresh": True},
        ),
        imu=imu,
    )
    # A simultaneous camera-quality outage and IMU outage must behave like
    # every other IMU outage: discard the bad interval and re-anchor heading
    # before integrating the first recovered sample.
    assert_close(state["x"], 10.0)
    assert_close(state["y"], 10.0)


def test_low_quality_recovery_clears_stale_boundary():
    imu = SyntheticImu()
    state = run_scenario(
        (
            {"yaw": 0.0, "dx": 10},
            {
                "yaw": 45.0,
                "dx": 100,
                "q": read_flow.MIN_SQUAL_FOR_POSITION - 1,
                "fresh": False,
            },
            {
                "yaw": 90.0,
                "dx": 100,
                "q": read_flow.MIN_SQUAL_FOR_POSITION - 1,
                "fresh": True,
            },
            {"yaw": 100.0, "dx": 10, "q": 255},
        ),
        imu=imu,
    )
    assert_close(state["x"], 10.0 + 10.0 * math.cos(math.radians(95.0)))
    assert_close(state["y"], 10.0 * math.sin(math.radians(95.0)))


def test_startup_loss_discard_and_reanchor():
    imu = SyntheticImu()
    state = run_scenario(
        (
            {"yaw": 0.0, "dx": 10},
            {"yaw": 90.0, "dx": 100, "startup_ready": False},
            {"yaw": 90.0, "dx": 10},
        ),
        imu=imu,
    )
    assert_close(state["x"], 10.0)
    assert_close(state["y"], 10.0)


def test_runtime_accuracy_drop_does_not_discard_or_reanchor():
    imu = SyntheticImu()
    state = run_scenario(
        (
            {"yaw": 90.0, "rotation_accuracy": 1, "mag_accuracy": 1},
            {
                "yaw": 90.0,
                "dx": 10,
                "rotation_accuracy": 1,
                "mag_accuracy": 1,
            },
            {
                "yaw": 100.0,
                "dx": 10,
                "rotation_accuracy": 3,
                "mag_accuracy": 3,
            },
        ),
        imu=imu,
    )
    assert_close(state["x"], 10.0 * math.cos(math.radians(95.0)))
    assert_close(state["y"], 10.0 + 10.0 * math.sin(math.radians(95.0)))
    if state["accuracy_low_events"] != 1:
        raise AssertionError("runtime accuracy transition was not recorded once")


def test_calibration_gate_and_post_enter_report():
    imu = SyntheticImu()
    imu.rotation_accuracy = 0
    imu.mag_accuracy = 0
    stream = io.StringIO("s\n")
    with (
        contextlib.redirect_stdout(io.StringIO()),
        mock.patch.object(read_flow.sys, "stdin", stream),
        mock.patch("read_flow.select.select", return_value=([stream], [], [])),
    ):
        if read_flow.wait_for_calibration_ready(imu) is not None:
            raise AssertionError("0/0 calibration state incorrectly passed startup gate")

    imu.rotation_accuracy = 2
    imu.mag_accuracy = 2
    with (
        contextlib.redirect_stdout(io.StringIO()),
        mock.patch("read_flow.select.select", return_value=([], [], [])),
    ):
        if read_flow.wait_for_calibration_ready(imu) is None:
            raise AssertionError("fresh 2/2 calibration state did not pass startup gate")

    previous_sequence = imu.snapshot()["orientation_sequence"]
    timer = threading.Timer(0.02, lambda: imu.publish_orientation(45.0))
    timer.start()
    try:
        report = read_flow.wait_for_new_orientation_report(
            imu, previous_sequence, timeout_s=0.20
        )
    finally:
        timer.join()
    if report is None or report["orientation_sequence"] <= previous_sequence:
        raise AssertionError("post-Enter RV sequence was not required")
    assert_close(report["yaw"], 45.0, tolerance=1e-6)

    # Passing the 2/2 gate is not a lifetime exemption.  If Mag becomes stale
    # while the operator is positioning the vehicle, a later RV alone must not
    # start fusion.
    imu = SyntheticImu()
    with imu._lock:
        imu.mag_time = time.monotonic() - 1.0
    previous_sequence = imu.snapshot()["orientation_sequence"]
    timer = threading.Timer(
        0.02,
        lambda: imu.publish_orientation(
            45.0, refresh_gyro=False, refresh_mag=False
        ),
    )
    timer.start()
    try:
        report = read_flow.wait_for_new_orientation_report(
            imu, previous_sequence, timeout_s=0.08
        )
    finally:
        timer.join()
    if report is not None:
        raise AssertionError("post-Enter RV accepted a stale Magnetometer")


def test_reference_uses_post_enter_quaternion():
    imu = SyntheticImu()
    state = run_scenario(
        ({"yaw": 90.0, "dx": 10},),
        imu=imu,
        reference_yaw=90.0,
    )
    assert_close(state["x"], 10.0)
    assert_close(state["y"], 0.0)


def calibrated_compensator():
    compensator = read_flow.AutoYawFlowCompensator()
    compensator.start_calibration()
    for index in range(20):
        sign = 1 if index % 2 == 0 else -1
        delta_yaw_rad = sign * 0.2
        corrected = compensator.compensate(
            sign * 2.0,
            0.0,
            delta_yaw_rad,
            gyro_magnitude=1.0,
        )
        if corrected != (0.0, 0.0):
            raise AssertionError("calibration motion entered the position estimate")
    if not compensator.finish_calibration():
        raise AssertionError("bidirectional explicit calibration did not freeze")
    return compensator


def test_auto_yaw_explicit_learning_and_freeze():
    normal = read_flow.AutoYawFlowCompensator()
    raw = normal.compensate(0.1, 0.0, math.radians(1.0), gyro_magnitude=1.0)
    assert_close(raw[0], 0.1, tolerance=1e-9)
    if normal.sample_count != 0:
        raise AssertionError("normal driving unexpectedly trained lever-arm state")

    one_direction = read_flow.AutoYawFlowCompensator()
    one_direction.start_calibration()
    for _ in range(one_direction.MIN_SAMPLES):
        one_direction.compensate(2.0, 0.0, 0.2, gyro_magnitude=1.0)
    if one_direction.finish_calibration():
        raise AssertionError("one-direction calibration was incorrectly accepted")

    frozen = calibrated_compensator()
    sample_count = frozen.sample_count
    learned_x = frozen.x_per_rad
    held_out = frozen.compensate(1.0, 0.0, 0.1, gyro_magnitude=1.0)
    assert_close(held_out[0], 0.0, tolerance=1e-6)
    if frozen.sample_count != sample_count or frozen.x_per_rad != learned_x:
        raise AssertionError("frozen estimate changed during normal driving")


def test_lever_arm_through_production_loop():
    imu = SyntheticImu()
    compensator = calibrated_compensator()
    events = []
    yaw = 0.0
    delta_deg = math.degrees(0.2)
    for index in range(20):
        sign = 1 if index % 2 == 0 else -1
        yaw += sign * delta_deg
        events.append(
            {
                "yaw": yaw,
                "dx": sign * 2,
                "dy": 0,
                "gyro": (0.0, 0.0, 1.0),
            }
        )
    state = run_scenario(events, imu=imu, compensator=compensator)
    assert_close(state["x"], 0.0)
    assert_close(state["y"], 0.0)
    if state["samples"] != compensator.sample_count:
        raise AssertionError(
            "frozen lever sample count changed inside production loop"
        )


def test_lever_calibration_exit_boundary():
    imu = SyntheticImu()
    calibration_events = []
    yaw = 0.0
    delta_yaws = (45.0, 45.0, -30.0, -30.0) * 2
    for delta_yaw in delta_yaws:
        yaw += delta_yaw
        calibration_events.append(
            {
                "yaw": yaw,
                "dx": 4 if delta_yaw > 0.0 else -3,
                "dy": 0,
                "gyro": (0.0, 0.0, 1.0),
            }
        )

    events = [
        {"yaw": 0.0, "dx": 10, "dy": 0},
        *calibration_events,
        {"yaw": 90.0, "dx": 10, "dy": 0},
    ]

    # Command 1 starts calibration after the initial normal movement.  Just
    # before command 2 freezes it, publish a newer 90-degree BNO report.  This
    # makes the exit re-anchor observable: without it, the first normal sample
    # would be rotated using the erroneous 0 -> 90 degree midpoint.
    state = run_scenario(
        events,
        imu=imu,
        commands=(
            (1, "c", None),
            (9, "c", lambda: imu.publish_orientation(90.0)),
        ),
    )

    # Initial +X movement remains.  All eight calibration bursts are consumed
    # but excluded from position.  The first post-calibration +X body movement
    # is transformed solely at the freshly re-anchored 90-degree heading.
    assert_close(state["x"], 10.0)
    assert_close(state["y"], 10.0)
    if state["samples"] != 8:
        raise AssertionError("explicit calibration did not consume all 8 bursts")
    if (state["ticks_x"], state["ticks_y"]) != (24, 0):
        raise AssertionError("calibration bursts were not continuously drained")


class MainFlow:
    def __init__(self, begin_result=True):
        self.begin_result = begin_result
        self.height_cm = 3.5
        self.scaler = 400.8
        self.closed = False

    def begin(self):
        return self.begin_result

    def close(self):
        self.closed = True


class MainImu:
    def __init__(self, fail_report=None):
        self.fail_report = fail_report
        self.boot_failure_reason = None
        self.enable_calls = []
        self.wait_calls = []
        self.calibration_calls = 0
        self.closed = False

    def begin(self):
        return True

    def enable_rotation_vector(self, interval):
        self.enable_calls.append((SENSOR_ROTATION_VECTOR, interval))
        return True

    def enable_gyro(self, interval):
        self.enable_calls.append((SENSOR_GYROSCOPE_CALIBRATED, interval))
        return True

    def enable_magnetometer(self, interval):
        self.enable_calls.append((SENSOR_MAGNETOMETER_CALIBRATED, interval))
        return True

    def wait_for_sensor_report(self, sensor_id, timeout):
        self.wait_calls.append((sensor_id, timeout))
        return sensor_id != self.fail_report

    def start_dynamic_calibration(self):
        self.calibration_calls += 1
        return True

    def status_summary(self):
        return "synthetic"

    def close(self):
        self.closed = True


def run_main_with_mocks(imu, selection="2"):
    flow = MainFlow()
    measurement_args = []
    captured = io.StringIO()
    with (
        contextlib.redirect_stdout(captured),
        mock.patch("read_flow.PMW3901", return_value=flow),
        mock.patch("read_flow.BNO086", return_value=imu),
        mock.patch("read_flow.run_measurement", side_effect=lambda *args: measurement_args.append(args)),
        mock.patch("builtins.input", return_value=selection),
    ):
        read_flow.main()
    return flow, measurement_args, captured.getvalue()


def test_main_fusion_setup_order_and_cleanup():
    imu = MainImu()
    flow, calls, output = run_main_with_mocks(imu)
    expected = [
        (SENSOR_ROTATION_VECTOR, 10_000),
        (SENSOR_GYROSCOPE_CALIBRATED, 10_000),
        (SENSOR_MAGNETOMETER_CALIBRATED, 20_000),
    ]
    if imu.enable_calls != expected:
        raise AssertionError(f"unexpected feature setup order: {imu.enable_calls}")
    if [sensor_id for sensor_id, _ in imu.wait_calls] != [item[0] for item in expected]:
        raise AssertionError("each feature was not followed by its input report check")
    if imu.calibration_calls != 1 or calls != [(flow, imu)]:
        raise AssertionError("fusion main orchestration did not reach measurement")
    if "Magnetometer(50Hz)" not in output:
        raise AssertionError("production setup did not report the 50Hz Mag rate")
    if not flow.closed or not imu.closed:
        raise AssertionError("main did not close both sensor objects")


def test_main_imu_failure_falls_back_to_paa():
    imu = MainImu(fail_report=SENSOR_GYROSCOPE_CALIBRATED)
    flow, calls, output = run_main_with_mocks(imu)
    if calls != [(flow, None)]:
        raise AssertionError("IMU report failure did not select PAA-only measurement")
    if imu.calibration_calls != 0 or not imu.closed or not flow.closed:
        raise AssertionError("fallback cleanup/calibration behavior mismatch")
    if "안전 전환" not in output:
        raise AssertionError("operator was not told about the safe fallback")


TESTS = (
    ("PAA burst signed decode", test_paa_burst_decode),
    ("standalone SQUAL/deadband/scale", test_standalone_filters_and_scale),
    ("runtime origin reset", test_runtime_origin_reset),
    ("absolute-yaw invariant world rotation", test_absolute_yaw_offset_and_world_rotation),
    ("heading wrap and midpoint", test_wrap_and_heading_midpoint),
    ("low-SQUAL heading synchronization", test_low_quality_heading_sync),
    ("IMU stale discard/re-anchor", test_stale_discard_and_reanchor),
    ("low-SQUAL + IMU stale re-anchor", test_low_quality_plus_stale_reanchor),
    ("low-SQUAL recovery boundary", test_low_quality_recovery_clears_stale_boundary),
    ("startup loss discard/re-anchor", test_startup_loss_discard_and_reanchor),
    ("runtime accuracy remains continuous", test_runtime_accuracy_drop_does_not_discard_or_reanchor),
    ("startup calibration gate/post-Enter RV", test_calibration_gate_and_post_enter_report),
    ("post-Enter quaternion reference", test_reference_uses_post_enter_quaternion),
    ("AutoYaw explicit learning/freeze", test_auto_yaw_explicit_learning_and_freeze),
    ("lever-arm production loop", test_lever_arm_through_production_loop),
    ("lever calibration exit boundary", test_lever_calibration_exit_boundary),
    ("main fusion setup order/cleanup", test_main_fusion_setup_order_and_cleanup),
    ("main IMU failure safe fallback", test_main_imu_failure_falls_back_to_paa),
)


def main():
    print("=== POSITION PIPELINE TEST ===")
    failures = []
    for name, test in TESTS:
        try:
            test()
        except Exception as exc:  # Keep running to report every independent case.
            failures.append((name, exc))
            print(f"{name}: FAIL ({type(exc).__name__}: {exc})")
        else:
            print(f"{name}: PASS")
    print()
    print(f"passed={len(TESTS) - len(failures)}/{len(TESTS)}")
    print(f"FINAL RESULT={'PASS' if not failures else 'FAIL'}")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
