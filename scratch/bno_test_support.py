#!/usr/bin/env python3
"""Shared, project-local support for the BNO086 diagnostic scripts."""

from __future__ import annotations

import struct
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from bno086 import (  # noqa: E402
    BNO086,
    CHAN_CONTROL,
    REPORT_GET_FEATURE_RESP,
    SENSOR_GYROSCOPE_CALIBRATED,
    SENSOR_MAGNETOMETER_CALIBRATED,
    SENSOR_ROTATION_VECTOR,
)


REPORT_PLAN = (
    ("RV", SENSOR_ROTATION_VECTOR, 10_000, 0.75),
    ("GYRO", SENSOR_GYROSCOPE_CALIBRATED, 10_000, 0.75),
    ("MAG", SENSOR_MAGNETOMETER_CALIBRATED, 40_000, 1.00),
)


class ObservedBNO086(BNO086):
    """BNO086 with packet-level observations kept only by scratch tests."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.i2c_syscall_errors = []
        self.shtp_validation_errors = []
        self.other_transport_errors = []
        self.fc_observations = {
            sensor_id: [] for _, sensor_id, _, _ in REPORT_PLAN
        }
        self.report_times = {
            SENSOR_ROTATION_VECTOR: [],
            SENSOR_GYROSCOPE_CALIBRATED: [],
            SENSOR_MAGNETOMETER_CALIBRATED: [],
        }

    def _record_io_error(self, error):
        """Split the driver's aggregate counter into diagnostic categories."""
        message = str(error)
        observation = {
            "timestamp": time.time(),
            "message": message,
            "errno": error.errno if isinstance(error, OSError) else None,
        }
        protocol_markers = (
            "continuation bit",
            "channel mismatch",
            "sequence mismatch",
            "length mismatch",
            "invalid continuation",
            "invalid SHTP length",
        )
        syscall_markers = (
            "short Phase",
            "short continuation",
            "short SHTP write",
            "H_INTN timeout",
        )
        if isinstance(error, OSError) or any(
            marker in message for marker in syscall_markers
        ):
            self.i2c_syscall_errors.append(observation)
        elif any(marker in message for marker in protocol_markers):
            self.shtp_validation_errors.append(observation)
        else:
            self.other_transport_errors.append(observation)
        super()._record_io_error(error)

    def _process_packet(self, packet):
        _, channel, _, cargo = packet
        if (
            channel == CHAN_CONTROL
            and len(cargo) >= 9
            and cargo[0] == REPORT_GET_FEATURE_RESP
        ):
            interval_us = struct.unpack_from("<I", cargo, 5)[0]
            self.fc_observations.setdefault(cargo[1], []).append(
                (time.monotonic(), interval_us)
            )

        before = (
            self.orientation_sequence,
            self.gyro_sequence,
            self.mag_sequence,
        )
        result = super()._process_packet(packet)
        after = (
            self.orientation_sequence,
            self.gyro_sequence,
            self.mag_sequence,
        )
        timestamps = (
            self.last_orientation_update,
            self.last_gyro_update,
            self.last_mag_update,
        )
        sensor_ids = (
            SENSOR_ROTATION_VECTOR,
            SENSOR_GYROSCOPE_CALIBRATED,
            SENSOR_MAGNETOMETER_CALIBRATED,
        )
        for sensor_id, old_sequence, new_sequence, timestamp in zip(
            sensor_ids, before, after, timestamps
        ):
            if new_sequence > old_sequence:
                self.report_times[sensor_id].extend(
                    [timestamp] * (new_sequence - old_sequence)
                )
        return result


def configure_one(imu, sensor_id, interval_us, report_timeout_s):
    """Run the production two-stage FC then input-report confirmation."""
    observation_start = len(imu.fc_observations.get(sensor_id, ()))
    enabled = imu.enable_feature(sensor_id, interval_us)
    report_received = False
    if imu.snapshot()["startup_ready"]:
        report_received = imu.wait_for_sensor_report(sensor_id, report_timeout_s)
    state = imu.snapshot()
    observations = imu.fc_observations.get(sensor_id, ())[observation_start:]
    return {
        "enabled": enabled,
        "fc_seen": [interval for _, interval in observations],
        "applied": state["feature_responses"].get(sensor_id),
        "report": report_received,
    }


def applied_state_is_valid(requested_interval_us, applied_interval_us):
    if applied_interval_us is None:
        return False
    if requested_interval_us > 0:
        return applied_interval_us > 0
    return applied_interval_us == 0
