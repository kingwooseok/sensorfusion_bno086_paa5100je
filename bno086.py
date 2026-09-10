#!/usr/bin/env python3
"""
BNO086 9DoF IMU Driver for Raspberry Pi 5 (RP1 GPIO + I2C-1)
- Model: SZH-MIN086
- I2C Bus: 1 (Pin 3: SDA, Pin 5: SCL) @ Address 0x4B
- INT Pin: Pin 12 (GPIO 18)
- RST Pin: Pin 16 (GPIO 23) or Pin 17 (3.3V)
"""

import os
import time
import fcntl
import struct
import math
import threading
import lgpio

# SHTP Channels
CHAN_COMMAND = 0
CHAN_EXECUTABLE = 1
CHAN_CONTROL = 2
CHAN_INPUT_NORMAL = 3
CHAN_INPUT_WAKE = 4

# SHTP Report IDs
REPORT_COMMAND_REQ = 0xF2
REPORT_COMMAND_RESP = 0xF1
REPORT_SET_FEATURE = 0xFD
REPORT_GET_FEATURE_REQ = 0xFE
REPORT_GET_FEATURE_RESP = 0xFC
REPORT_PRODUCT_ID_REQ = 0xF9
REPORT_PRODUCT_ID_RESP = 0xF8
REPORT_TIMEBASE_REF = 0xFB

# Sensor Report IDs
SENSOR_ACCELEROMETER = 0x01
SENSOR_GYROSCOPE_CALIBRATED = 0x02
SENSOR_MAGNETOMETER_CALIBRATED = 0x03
SENSOR_ROTATION_VECTOR = 0x05
SENSOR_GAME_ROTATION_VECTOR = 0x08

# Motion Engine command IDs
COMMAND_TARE = 0x03
COMMAND_ME_CALIBRATE = 0x07

# SHTP / SH-2 startup values.  A Product-ID response by itself is not a
# sufficient indication that the sensor hub is ready to accept feature
# commands: it can still be in the middle of its startup advertisement.
EXECUTABLE_RESET_COMPLETE = 0x01
COMMAND_INITIALIZE = 0x84
MAX_I2C_TRANSFER = 32
MAX_SHTP_PACKET = 284

class BNO086:
    def __init__(self, i2c_bus=1, i2c_addr=0x4B, int_pin=18, rst_pin=23):
        self.i2c_bus = i2c_bus
        self.i2c_addr = i2c_addr
        self.int_pin = int_pin
        self.rst_pin = rst_pin
        
        self.fd = -1
        self.gpio_handle = -1
        
        # Latest sensor states
        self.roll = 0.0      # deg
        self.pitch = 0.0     # deg
        self.yaw = 0.0       # deg
        self.gyro_x = 0.0    # rad/s
        self.gyro_y = 0.0    # rad/s
        self.gyro_z = 0.0    # rad/s
        self.mag_x = 0.0     # uT
        self.mag_y = 0.0     # uT
        self.mag_z = 0.0     # uT
        self.mag_accuracy = 0
        self.rotation_accuracy = 0
        self.gyro_accuracy = 0

        # Keep the quaternion as well as Euler angles.  The measurement layer
        # takes its reference from the first valid sample, so a fixed mounting
        # offset never needs to be entered by hand.
        self.quat_i = 0.0
        self.quat_j = 0.0
        self.quat_k = 0.0
        self.quat_real = 1.0

        # Use monotonic timestamps for report freshness checks.  Wall-clock
        # adjustments must not make a stale IMU report look current.
        self.last_update = 0.0
        self.last_orientation_update = 0.0
        self.last_gyro_update = 0.0
        self.last_mag_update = 0.0
        self.orientation_sequence = 0
        self.gyro_sequence = 0
        self.mag_sequence = 0
        self.last_command_id = None
        self.last_command_status = None
        self._state_lock = threading.Lock()
        self._io_lock = threading.Lock()

        # Transport/boot diagnostics.  These make an electrical or SHTP
        # startup failure explicit instead of letting the caller mistake a
        # Product-ID reply for a usable IMU.
        self.startup_ready = False
        self.boot_advertisement_seen = False
        self.boot_reset_seen = False
        self.boot_initialize_seen = False
        self.boot_failure_reason = None
        self.startup_warning = None
        self.last_reset_cause = None
        self.reset_count = 0
        self.io_error_count = 0
        self.last_io_error = None
        self.phase2_null_count = 0
        self.phase2_null_retry_success = 0
        self.phase2_null_retry_fail = 0
        self.last_shtp_errors = []
        self.feature_responses = {}
        self.packet_counts = [0] * 6
        
        # Sequence counter per channel
        self.seq_out = [0] * 6
        self.command_seq = 0

    def open(self):
        """Open I2C and GPIO resources"""
        try:
            self.gpio_handle = lgpio.gpiochip_open(0)
            
            # Setup the active-low reset line.  It is pulsed in begin() so that
            # every session starts from a known SHTP state.
            if self.rst_pin is not None and self.rst_pin >= 0:
                lgpio.gpio_claim_output(self.gpio_handle, self.rst_pin, 1)
                os.system(f"pinctrl set {self.rst_pin} op dh >/dev/null 2>&1")
            
            # Setup INT pin with pull-up
            if self.int_pin is not None and self.int_pin >= 0:
                lgpio.gpio_claim_input(self.gpio_handle, self.int_pin, lgpio.SET_PULL_UP)
                
            self.fd = os.open(f"/dev/i2c-{self.i2c_bus}", os.O_RDWR)
            fcntl.ioctl(self.fd, 0x0703, self.i2c_addr)
            return True
        except Exception as e:
            print(f"[!] BNO086 open error: {e}")
            self.close()
            return False

    def reset_hardware(self):
        """Pulse active-low RST so stale SHTP state cannot survive a restart."""
        if self.rst_pin is not None and self.gpio_handle >= 0:
            lgpio.gpio_write(self.gpio_handle, self.rst_pin, 0)
            time.sleep(0.010)
            lgpio.gpio_write(self.gpio_handle, self.rst_pin, 1)
            # H_INTN normally asserts roughly 90 ms after reset.  Do not send
            # any SHTP control traffic here; begin() waits for and consumes the
            # complete advertisement/startup sequence first.
            time.sleep(0.005)

    def is_int_asserted(self):
        """Check if INT pin is LOW (data ready)"""
        if self.gpio_handle >= 0 and self.int_pin >= 0:
            return lgpio.gpio_read(self.gpio_handle, self.int_pin) == 0
        return True

    def wait_for_int(self, timeout_s=0.5):
        """Wait until INT pin goes LOW"""
        t0 = time.monotonic()
        while not self.is_int_asserted():
            if time.monotonic() - t0 > timeout_s:
                return False
            time.sleep(0.0002)
        return True

    def _record_io_error(self, error):
        """Save I2C errors for diagnosis without flooding the update thread."""
        with self._state_lock:
            self.io_error_count += 1
            self.last_io_error = str(error)

    def read_packet(self, timeout_s=0.020):
        """Read one complete SHTP packet using Phase-1 header and Phase-2 continuation."""
        if not self.is_int_asserted():
            return None

        phase2_null_retried = False
        try:
            with self._io_lock:
                # Phase 1: Read exactly the 4-byte SHTP header
                hdr = os.read(self.fd, 4)
                if len(hdr) < 4:
                    return None

                raw_length = hdr[0] | (hdr[1] << 8)
                total_len = raw_length & 0x7FFF

                # BNO086 zero-length packet (length == 0) is a normal null packet: ignore without error
                if total_len == 0:
                    return None

                chan = hdr[2]
                seq = hdr[3]

                # If total_len == 4, there is no payload (empty SHTP packet)
                if total_len == 4:
                    return total_len, chan, seq, b''

                if total_len > MAX_SHTP_PACKET:
                    self._record_io_error(f"invalid SHTP length {total_len}")
                    return None

                # Phase 1 -> Phase 2 INT handshake: preserve handshake before reading continuation
                if not self.is_int_asserted():
                    if not self.wait_for_int(timeout_s):
                        self._record_io_error("H_INTN timeout waiting for Phase 2 continuation")
                        return None

                # Phase 2: Read retransmission with continuation header
                chunk = os.read(self.fd, total_len)
                if len(chunk) < 4:
                    self._record_io_error("short Phase 2 SHTP header")
                    return None

                # A BNO086 may transiently return an exact null header even
                # though Phase 1 established that this cargo is still pending.
                # Preserve the Phase-1 channel/sequence/length expectations
                # and retry this same Phase-2 read exactly once.  No other
                # malformed continuation is eligible for this retry.
                if chunk[:4] == b'\x00\x00\x00\x00':
                    with self._state_lock:
                        self.phase2_null_count += 1
                    phase2_null_retried = True
                    chunk = os.read(self.fd, total_len)
                    if len(chunk) < 4:
                        with self._state_lock:
                            self.phase2_null_retry_fail += 1
                        self._record_io_error("short Phase 2 SHTP header")
                        return None

                p2_raw_length = chunk[0] | (chunk[1] << 8)
                p2_len = p2_raw_length & 0x7FFF
                p2_cont = bool(p2_raw_length & 0x8000)
                p2_chan = chunk[2]
                p2_seq = chunk[3]

                expected_seq = (seq + 1) & 0xFF
                if not p2_cont:
                    if phase2_null_retried:
                        with self._state_lock:
                            self.phase2_null_retry_fail += 1
                    self._record_io_error("Phase 2 continuation bit not set")
                    return None
                if p2_chan != chan:
                    if phase2_null_retried:
                        with self._state_lock:
                            self.phase2_null_retry_fail += 1
                    self._record_io_error(f"Phase 2 channel mismatch: {p2_chan} != {chan}")
                    return None
                if p2_seq != expected_seq:
                    if phase2_null_retried:
                        with self._state_lock:
                            self.phase2_null_retry_fail += 1
                    self._record_io_error(f"Phase 2 sequence mismatch: {p2_seq} != {expected_seq}")
                    return None
                if p2_len != total_len:
                    if phase2_null_retried:
                        with self._state_lock:
                            self.phase2_null_retry_fail += 1
                    self._record_io_error(f"Phase 2 length mismatch: {p2_len} != {total_len}")
                    return None

                if phase2_null_retried:
                    with self._state_lock:
                        self.phase2_null_retry_success += 1
                    # The null-specific recovery is complete.  Any later
                    # additional-continuation failure is independent of it.
                    phase2_null_retried = False

                cargo = bytearray(chunk[4:])
                remaining = (total_len - 4) - len(cargo)

                while remaining > 0:
                    if not self.wait_for_int(timeout_s):
                        self._record_io_error("H_INTN timeout during continuation chunk")
                        return None
                    req_len = remaining + 4
                    c_chunk = os.read(self.fd, req_len)
                    if len(c_chunk) < 4:
                        self._record_io_error("short continuation chunk")
                        return None
                    c_raw_length = c_chunk[0] | (c_chunk[1] << 8)
                    c_len = c_raw_length & 0x7FFF
                    c_cont = bool(c_raw_length & 0x8000)
                    if not c_cont or c_chunk[2] != chan or c_len != remaining + 4:
                        self._record_io_error(f"invalid continuation chunk: len={c_len}, expected={remaining+4}")
                        return None
                    take = min(remaining, len(c_chunk) - 4)
                    if take <= 0:
                        break
                    cargo.extend(c_chunk[4:4 + take])
                    remaining -= take

                return total_len, chan, p2_seq, bytes(cargo)
        except OSError as error:
            if phase2_null_retried:
                with self._state_lock:
                    self.phase2_null_retry_fail += 1
            self._record_io_error(error)
            return None
        except Exception as error:
            if phase2_null_retried:
                with self._state_lock:
                    self.phase2_null_retry_fail += 1
            self._record_io_error(error)
            return None

    def send_packet(self, chan, payload):
        """Send SHTP packet to BNO086 with retries"""
        seq = self.seq_out[chan] & 0xFF
        self.seq_out[chan] = (self.seq_out[chan] + 1) & 0xFF
        
        pkt_len = len(payload) + 4
        pkt = bytearray([pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, chan, seq]) + bytearray(payload)
        for attempt in range(5):
            try:
                with self._io_lock:
                    written = os.write(self.fd, pkt)
                if written == len(pkt):
                    return True
                self._record_io_error(f"short SHTP write {written}/{len(pkt)}")
            except OSError as error:
                self._record_io_error(error)
            time.sleep(0.02)
        return False

    def drain_boot_packets(self, timeout_s=1.5):
        """Consume and validate the full BNO086 SHTP startup handshake.

        The hub must finish its unsolicited channel-0 advertisement before the
        host writes.  Normal SH-2 firmware then reports reset-complete on
        channel 1 and unsolicited Initialize (F1/84) on channel 2.  A few
        BNO086 firmware revisions omit the latter packet on I2C; for those we
        retain a warning and require Product-ID plus feature/input confirmation
        before enabling fusion instead of falsely accepting a raw reset packet.
        """
        t0 = time.monotonic()
        self.startup_ready = False
        self.boot_advertisement_seen = False
        self.boot_reset_seen = False
        self.boot_initialize_seen = False
        self.boot_failure_reason = None
        self.startup_warning = None
        # The alternate startup path is valid only after the hub has been
        # quiet (H_INTN released) for a little while.  Time elapsed since
        # reset-complete alone is not enough: a hub that is still advertising,
        # reporting an error, or rebooting also reaches that elapsed time.
        quiet_since = None

        while time.monotonic() - t0 < timeout_s:
            if self.is_int_asserted():
                quiet_since = None
                pkt = self.read_packet(timeout_s=0.1)
                if pkt:
                    _, chan, _, cargo = pkt
                    if chan == CHAN_COMMAND and cargo and cargo[0] == 0x00:
                        # A new advertisement after reset-complete is a new
                        # boot attempt, not the completion of the old one.
                        if self.boot_reset_seen or self.boot_initialize_seen:
                            self.boot_reset_seen = False
                            self.boot_initialize_seen = False
                        self.boot_advertisement_seen = True
                    elif chan == CHAN_COMMAND and cargo and cargo[0] == 0x01:
                        # Preserve an early SHTP error as a useful diagnostic
                        # if the hub never reaches the control-ready state.
                        with self._state_lock:
                            self.last_shtp_errors = list(cargo[1:])
                    elif (
                        self.boot_advertisement_seen
                        and chan == CHAN_EXECUTABLE
                        and cargo == bytes([EXECUTABLE_RESET_COMPLETE])
                    ):
                        self.boot_reset_seen = True
                    elif (
                        self.boot_advertisement_seen
                        and self.boot_reset_seen
                        and chan == CHAN_CONTROL
                        and len(cargo) >= 7
                        and cargo[0] == REPORT_COMMAND_RESP
                        and cargo[2] == COMMAND_INITIALIZE
                        and cargo[3] == 0
                        and cargo[5] == 0
                        and cargo[6] == 1
                    ):
                        self.boot_initialize_seen = True

                    if (
                        self.boot_advertisement_seen
                        and self.boot_reset_seen
                        and self.boot_initialize_seen
                    ):
                        self.startup_ready = True
                        return True
                else:
                    # Avoid a busy loop when an asserted INT is accompanied
                    # by a transport error; the timeout below will expose it.
                    time.sleep(0.001)
            else:
                if self.boot_advertisement_seen and self.boot_reset_seen:
                    if quiet_since is None:
                        quiet_since = time.monotonic()
                    elif time.monotonic() - quiet_since >= 0.250:
                        # Some deployed BNO086 builds answer normal control
                        # requests after advertisement/reset-complete without
                        # the unsolicited F1/84 packet.  Accept that variant
                        # only after a real quiet period; Product-ID, FC and an
                        # actual input report remain mandatory later in setup.
                        self.startup_ready = True
                        self.startup_warning = (
                            "SH-2 initialize(ch2/F1-84) not observed; "
                            "using strict control/report confirmation"
                        )
                        return True
                time.sleep(0.001)

        missing = []
        if not self.boot_advertisement_seen:
            missing.append("advertisement(ch0)")
        if not self.boot_reset_seen:
            missing.append("reset-complete(ch1)")
        if not self.boot_initialize_seen:
            missing.append("SH-2 initialize(ch2/F1-84)")
        if self.boot_advertisement_seen and self.boot_reset_seen and self.io_error_count:
            self.boot_failure_reason = (
                "startup never became idle after reset-complete "
                f"(I2C: {self.last_io_error})"
            )
        else:
            self.boot_failure_reason = "missing " + ", ".join(missing)
        return False

    def begin(self):
        """Initialize and verify BNO086 communication"""
        if not self.open():
            return False
            
        self.reset_hardware()
        print("[*] Waiting for BNO086 startup sequence...")
        if not self.drain_boot_packets(2.0):
            print(
                "[!] BNO086 startup handshake incomplete: "
                f"{self.boot_failure_reason}. No feature command was sent."
            )
            return False

        # A reset restarts every host-to-hub SHTP sequence counter.
        self.seq_out = [0] * 6
        self.command_seq = 0
        
        # Verify with Product ID Request
        print("[*] Querying Product ID...")
        if not self.send_packet(CHAN_CONTROL, [REPORT_PRODUCT_ID_REQ, 0x00]):
            self.boot_failure_reason = "Product ID request write failed"
            print("[!] Failed to send Product ID request to BNO086!")
            return False

        product_packet = self._wait_for_packet(
            lambda packet: (
                packet[1] == CHAN_CONTROL
                and len(packet[3]) >= 2
                and packet[3][0] == REPORT_PRODUCT_ID_RESP
            ),
            timeout_s=0.8,
        )
        if product_packet is None:
            if not self.boot_failure_reason:
                self.boot_failure_reason = "no Product ID response"
            print("[!] Failed to verify Product ID from BNO086!")
            return False

        self.last_reset_cause = product_packet[3][1]
        if self.startup_warning:
            print(f"[*] BNO086 startup warning: {self.startup_warning}")
        print("[+] BNO086 connected and verified successfully!")
        return True

    def _wait_for_packet(self, predicate, timeout_s=0.75):
        """Service packets until a particular response arrives or the hub resets."""
        deadline = time.monotonic() + timeout_s
        reset_count_at_start = self.reset_count
        while time.monotonic() < deadline:
            if self.is_int_asserted():
                pkt = self.read_packet()
                if pkt is not None:
                    self._process_packet(pkt)
                    if predicate(pkt):
                        return pkt
                    if self.reset_count != reset_count_at_start:
                        self.boot_failure_reason = "BNO086 reset while waiting for response"
                        return None
            else:
                time.sleep(0.001)
        return None

    def enable_feature(self, sensor_id, interval_us=20000, timeout_s=0.75):
        """Set one SH-2 feature and confirm its current state with FE/FC.

        SH-2 may emit an unsolicited FC while changing a feature rate.  In
        particular, an enable can transiently report a zero interval before
        the requested feature becomes active.  Keep querying within one
        overall deadline until FC reports a state consistent with the request.
        """
        if not self.startup_ready:
            self.boot_failure_reason = "feature requested before a complete SH-2 startup"
            return False

        deadline = time.monotonic() + timeout_s
        reset_count_at_start = self.reset_count

        # Step 1: Send Set Feature (0xFD) in the existing 17-byte format.
        pkt = bytearray(17)
        pkt[0] = REPORT_SET_FEATURE
        pkt[1] = sensor_id
        struct.pack_into('<I', pkt, 5, interval_us)
        if not self.send_packet(CHAN_CONTROL, pkt):
            self.boot_failure_reason = f"failed to send 0xFD set feature for sensor 0x{sensor_id:02X}"
            return False

        # Query the feature state.  A zero FC after an enable (or a non-zero FC
        # after a disable) is a stale/transitional observation, not a final
        # failure.  Wait briefly, issue FE again, and only accept an FC seen
        # after that re-query.  _process_packet() keeps feature_responses up to
        # date, so on success it contains the final applied interval.
        get_feature_pkt = bytes([REPORT_GET_FEATURE_REQ, sensor_id])
        requery_delay_s = 0.010
        next_query_at = time.monotonic()
        response_is_eligible = False

        while time.monotonic() < deadline:
            if (
                self.reset_count != reset_count_at_start
                or not self.startup_ready
            ):
                if not self.boot_failure_reason:
                    self.boot_failure_reason = (
                        f"BNO086 startup lost while configuring sensor 0x{sensor_id:02X}"
                    )
                return False

            now = time.monotonic()
            if now >= next_query_at:
                if self.send_packet(CHAN_CONTROL, get_feature_pkt):
                    response_is_eligible = True
                    # With no FC, the single overall deadline remains the
                    # authority.  Another FE is scheduled only after an
                    # inconsistent FC (or after a failed FE write).
                    next_query_at = deadline
                else:
                    response_is_eligible = False
                    next_query_at = min(deadline, time.monotonic() + requery_delay_s)

                if (
                    self.reset_count != reset_count_at_start
                    or not self.startup_ready
                ):
                    if not self.boot_failure_reason:
                        self.boot_failure_reason = (
                            f"BNO086 startup lost while configuring sensor 0x{sensor_id:02X}"
                        )
                    return False

            if self.is_int_asserted():
                packet = self.read_packet()
                if packet is None:
                    time.sleep(0.001)
                    continue

                self._process_packet(packet)
                if (
                    self.reset_count != reset_count_at_start
                    or not self.startup_ready
                ):
                    if not self.boot_failure_reason:
                        self.boot_failure_reason = (
                            f"BNO086 startup lost while configuring sensor 0x{sensor_id:02X}"
                        )
                    return False

                cargo = packet[3]
                matching_fc = (
                    packet[1] == CHAN_CONTROL
                    and len(cargo) >= 9
                    and cargo[0] == REPORT_GET_FEATURE_RESP
                    and cargo[1] == sensor_id
                )
                if matching_fc and response_is_eligible:
                    applied_interval_us = struct.unpack_from('<I', cargo, 5)[0]
                    state_matches_request = (
                        applied_interval_us > 0
                        if interval_us > 0
                        else applied_interval_us == 0
                    )
                    if state_matches_request:
                        return True

                    # Record-only happens in _process_packet().  Do not treat
                    # this stale/transitional FC as rejection; require a new
                    # FE before another FC can complete the operation.
                    response_is_eligible = False
                    next_query_at = min(
                        deadline,
                        time.monotonic() + requery_delay_s,
                    )
            else:
                time.sleep(0.001)

        expected_state = "non-zero" if interval_us > 0 else "zero"
        self.boot_failure_reason = (
            f"no valid {expected_state} FC feature response for sensor "
            f"0x{sensor_id:02X} before timeout"
        )
        return False

    def wait_for_sensor_report(self, sensor_id, timeout_s=0.75):
        """Require one real input report after enabling a feature."""
        sequence_key = {
            SENSOR_ROTATION_VECTOR: "orientation_sequence",
            SENSOR_GAME_ROTATION_VECTOR: "orientation_sequence",
            SENSOR_GYROSCOPE_CALIBRATED: "gyro_sequence",
            SENSOR_MAGNETOMETER_CALIBRATED: "mag_sequence",
        }.get(sensor_id)
        if sequence_key is None:
            return False

        initial_sequence = self.snapshot()[sequence_key]
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.update()
            if self.snapshot()[sequence_key] > initial_sequence:
                return True
            if not self.startup_ready:
                return False
            time.sleep(0.001)
        self.boot_failure_reason = f"no input report for sensor 0x{sensor_id:02X}"
        return False

    def enable_rotation_vector(self, interval_us=20000):
        """Enable magnetometer-corrected Rotation Vector (0x05)."""
        return self.enable_feature(SENSOR_ROTATION_VECTOR, interval_us)

    def enable_game_rotation_vector(self, interval_us=20000):
        """Enable the gyro-only Game Rotation Vector when drift is acceptable."""
        return self.enable_feature(SENSOR_GAME_ROTATION_VECTOR, interval_us)

    def enable_gyro(self, interval_us=20000):
        """Enable Calibrated Gyroscope (0x02) in microseconds"""
        return self.enable_feature(SENSOR_GYROSCOPE_CALIBRATED, interval_us)

    def enable_magnetometer(self, interval_us=40000):
        """Enable the calibrated magnetic-field report used to monitor heading quality."""
        return self.enable_feature(SENSOR_MAGNETOMETER_CALIBRATED, interval_us)

    def start_dynamic_calibration(self):
        """Ask the BNO086 Motion Engine to calibrate gyro and magnetometer this session.

        This does not write calibration data to persistent storage.  It lets the
        BNO086 adapt to the magnetic environment around the vehicle (motors,
        battery, chassis) while it is used.
        """
        command_seq = self.command_seq & 0xFF
        self.command_seq = (self.command_seq + 1) & 0xFF
        # Command Request: ID, command sequence, command, then R0..R8.
        # R0/R1/R2 select accel/gyro/magnetometer calibration and R3 selects
        # ME_CAL_CONFIG (0).  Accelerometer calibration is deliberately left
        # off because the vehicle is normally moving when this is called.
        payload = bytearray([
            REPORT_COMMAND_REQ, command_seq, COMMAND_ME_CALIBRATE,
            0, 1, 1, 0, 0, 0, 0, 0, 0,
        ])
        if not self.send_packet(CHAN_CONTROL, payload):
            return False

        response = self._wait_for_packet(
            lambda packet: (
                packet[1] == CHAN_CONTROL
                and len(packet[3]) >= 6
                and packet[3][0] == REPORT_COMMAND_RESP
                and packet[3][2] == COMMAND_ME_CALIBRATE
                and packet[3][3] == command_seq
            ),
            timeout_s=0.75,
        )
        if response is None:
            self.boot_failure_reason = "no response to dynamic calibration command"
            return False
        if response[3][5] != 0:
            self.boot_failure_reason = (
                f"dynamic calibration command failed (R0={response[3][5]})"
            )
            return False
        return True

    def tare_all_axes(self):
        """Apply a runtime-only all-axis tare using Rotation Vector (0x05).

        Tare uses the current, stationary pose as the device frame.  It is not
        persisted, so every run can establish a fresh origin without changing
        BNO086 flash/DCD settings.
        """
        command_seq = self.command_seq & 0xFF
        self.command_seq = (self.command_seq + 1) & 0xFF
        # Tare command: TARE_NOW(0), axes X|Y|Z(0x07), Rotation Vector basis(0).
        payload = bytearray([
            REPORT_COMMAND_REQ, command_seq, COMMAND_TARE,
            0, 0x07, 0x00, 0, 0, 0, 0, 0, 0,
        ])
        if not self.startup_ready:
            return False
        return self.send_packet(CHAN_CONTROL, payload)

    def snapshot(self):
        """Return one internally consistent copy of the latest IMU state."""
        with self._state_lock:
            return {
                "roll": self.roll,
                "pitch": self.pitch,
                "yaw": self.yaw,
                "gyro_x": self.gyro_x,
                "gyro_y": self.gyro_y,
                "gyro_z": self.gyro_z,
                "mag_x": self.mag_x,
                "mag_y": self.mag_y,
                "mag_z": self.mag_z,
                "mag_accuracy": self.mag_accuracy,
                "rotation_accuracy": self.rotation_accuracy,
                "gyro_accuracy": self.gyro_accuracy,
                "quat_i": self.quat_i,
                "quat_j": self.quat_j,
                "quat_k": self.quat_k,
                "quat_real": self.quat_real,
                "last_update": self.last_update,
                "last_orientation_update": self.last_orientation_update,
                "last_gyro_update": self.last_gyro_update,
                "last_mag_update": self.last_mag_update,
                "orientation_sequence": self.orientation_sequence,
                "gyro_sequence": self.gyro_sequence,
                "mag_sequence": self.mag_sequence,
                "last_command_id": self.last_command_id,
                "last_command_status": self.last_command_status,
                "startup_ready": self.startup_ready,
                "boot_advertisement_seen": self.boot_advertisement_seen,
                "boot_reset_seen": self.boot_reset_seen,
                "boot_initialize_seen": self.boot_initialize_seen,
                "boot_failure_reason": self.boot_failure_reason,
                "startup_warning": self.startup_warning,
                "last_reset_cause": self.last_reset_cause,
                "reset_count": self.reset_count,
                "io_error_count": self.io_error_count,
                "last_io_error": self.last_io_error,
                "phase2_null_count": self.phase2_null_count,
                "phase2_null_retry_success": self.phase2_null_retry_success,
                "phase2_null_retry_fail": self.phase2_null_retry_fail,
                "last_shtp_errors": tuple(self.last_shtp_errors),
                "feature_responses": dict(self.feature_responses),
                "packet_counts": tuple(self.packet_counts),
            }

    def status_summary(self):
        """Compact transport state for user-facing setup/fallback messages."""
        state = self.snapshot()
        boot = (
            f"boot A/R/I={int(state['boot_advertisement_seen'])}/"
            f"{int(state['boot_reset_seen'])}/"
            f"{int(state['boot_initialize_seen'])}"
        )
        io = (
            f"I2C errors={state['io_error_count']}"
            + (f" ({state['last_io_error']})" if state['last_io_error'] else "")
        )
        reset = f"resets={state['reset_count']}"
        shtp_errors = (
            "SHTP errors=" + " ".join(f"{value:02X}" for value in state["last_shtp_errors"])
            if state["last_shtp_errors"] else "SHTP errors=none"
        )
        return f"ready={int(state['startup_ready'])}, {boot}, {reset}, {io}, {shtp_errors}"

    def _process_packet(self, packet):
        """Update transport diagnostics and decode one complete SHTP packet."""
        _, chan, _, cargo = packet
        if 0 <= chan < len(self.packet_counts):
            with self._state_lock:
                self.packet_counts[chan] += 1

        # A new advertisement or executable reset after startup means the hub
        # discarded all feature settings.  Never keep integrating position as
        # though the old heading stream were still valid.
        if chan == CHAN_COMMAND and cargo:
            if cargo[0] == 0x00:
                with self._state_lock:
                    if self.startup_ready:
                        self.startup_ready = False
                        self.boot_failure_reason = "BNO086 restarted (new advertisement)"
            elif cargo[0] == 0x01:
                with self._state_lock:
                    self.last_shtp_errors = list(cargo[1:])
            return False

        if chan == CHAN_EXECUTABLE and cargo == bytes([EXECUTABLE_RESET_COMPLETE]):
            with self._state_lock:
                self.reset_count += 1
                self.startup_ready = False
                self.boot_failure_reason = "BNO086 reset-complete received during session"
            return False

        if chan == CHAN_CONTROL and cargo:
            if cargo[0] == REPORT_PRODUCT_ID_RESP and len(cargo) >= 2:
                with self._state_lock:
                    self.last_reset_cause = cargo[1]
            elif cargo[0] == REPORT_GET_FEATURE_RESP and len(cargo) >= 2:
                interval_us = None
                if len(cargo) >= 9:
                    interval_us = struct.unpack_from('<I', cargo, 5)[0]
                with self._state_lock:
                    self.feature_responses[cargo[1]] = interval_us
            elif cargo[0] == REPORT_COMMAND_RESP and len(cargo) >= 6:
                # Command response byte 5 is R0, the Motion Engine status.
                with self._state_lock:
                    self.last_command_id = cargo[2]
                    self.last_command_status = cargo[5]
            return False

        if chan not in (CHAN_INPUT_NORMAL, CHAN_INPUT_WAKE) or len(cargo) < 5:
            return False

        updated = False
        cursor = 0
        while cursor < len(cargo):
            report_id = cargo[cursor]
            if report_id == REPORT_TIMEBASE_REF and cursor + 5 <= len(cargo):
                cursor += 5
            elif (
                report_id == SENSOR_GAME_ROTATION_VECTOR and cursor + 12 <= len(cargo)
            ) or (
                report_id == SENSOR_ROTATION_VECTOR and cursor + 14 <= len(cargo)
            ):
                qi, qj, qk, qr = struct.unpack_from('<hhhh', cargo, cursor + 4)
                scale = 1.0 / 16384.0
                fi, fj, fk, fr = qi * scale, qj * scale, qk * scale, qr * scale

                norm = math.sqrt(fr * fr + fi * fi + fj * fj + fk * fk)
                if norm > 0:
                    fr /= norm
                    fi /= norm
                    fj /= norm
                    fk /= norm

                sinr_cosp = 2.0 * (fr * fi + fj * fk)
                cosr_cosp = 1.0 - 2.0 * (fi * fi + fj * fj)
                roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
                sinp = 2.0 * (fr * fj - fk * fi)
                pitch = (
                    math.copysign(90.0, sinp)
                    if abs(sinp) >= 1.0
                    else math.degrees(math.asin(sinp))
                )
                siny_cosp = 2.0 * (fr * fk + fi * fj)
                cosy_cosp = 1.0 - 2.0 * (fj * fj + fk * fk)
                yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
                now = time.monotonic()
                with self._state_lock:
                    self.roll = roll
                    self.pitch = pitch
                    self.yaw = yaw
                    self.quat_i = fi
                    self.quat_j = fj
                    self.quat_k = fk
                    self.quat_real = fr
                    self.rotation_accuracy = cargo[cursor + 2] & 0x03
                    self.last_update = now
                    self.last_orientation_update = now
                    self.orientation_sequence += 1
                updated = True
                cursor += 14 if report_id == SENSOR_ROTATION_VECTOR else 12
            elif report_id == SENSOR_GYROSCOPE_CALIBRATED and cursor + 10 <= len(cargo):
                gx, gy, gz = struct.unpack_from('<hhh', cargo, cursor + 4)
                now = time.monotonic()
                with self._state_lock:
                    self.gyro_x = gx / 512.0
                    self.gyro_y = gy / 512.0
                    self.gyro_z = gz / 512.0
                    self.gyro_accuracy = cargo[cursor + 2] & 0x03
                    self.last_update = now
                    self.last_gyro_update = now
                    self.gyro_sequence += 1
                updated = True
                cursor += 10
            elif report_id == SENSOR_MAGNETOMETER_CALIBRATED and cursor + 10 <= len(cargo):
                mx, my, mz = struct.unpack_from('<hhh', cargo, cursor + 4)
                now = time.monotonic()
                with self._state_lock:
                    # Calibrated magnetic-field reports use Q4, in uT.
                    self.mag_x = mx / 16.0
                    self.mag_y = my / 16.0
                    self.mag_z = mz / 16.0
                    self.mag_accuracy = cargo[cursor + 2] & 0x03
                    self.last_update = now
                    self.last_mag_update = now
                    self.mag_sequence += 1
                updated = True
                cursor += 10
            else:
                # Unknown report in a combined input payload.  Advance one
                # byte rather than losing later, known reports in that packet.
                cursor += 1
        return updated

    def update(self):
        """Service pending SHTP packets without blocking the caller."""
        updated = False
        for _ in range(8):
            if not self.is_int_asserted():
                break
            pkt = self.read_packet()
            if pkt is None:
                break
            updated = self._process_packet(pkt) or updated
        return updated

    def close(self):
        """Release resources and ensure RST pin stays HIGH"""
        if self.fd >= 0:
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = -1
        if self.gpio_handle >= 0:
            try:
                lgpio.gpiochip_close(self.gpio_handle)
            except Exception:
                pass
            self.gpio_handle = -1
        # Guarantee Pin 16 stays at 3.3V (op dh) even after Python exits
        if self.rst_pin is not None and self.rst_pin >= 0:
            os.system(f"pinctrl set {self.rst_pin} op dh >/dev/null 2>&1")
