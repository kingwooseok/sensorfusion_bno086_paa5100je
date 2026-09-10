#!/usr/bin/env python3
"""Instrument the existing Phase-1/wait/Phase-2 BNO086 read sequence."""

from __future__ import annotations

import argparse
import collections
import datetime
import os
import time

import lgpio

from bno_test_support import ObservedBNO086, REPORT_PLAN, configure_one
from bno086 import MAX_SHTP_PACKET


def int_text(asserted):
    return "LOW(asserted)" if asserted else "HIGH(deasserted)"


def header_fields(header):
    if header is None or len(header) < 4:
        return None
    raw_length = header[0] | (header[1] << 8)
    return {
        "raw_length": raw_length,
        "total_length": raw_length & 0x7FFF,
        "continuation": bool(raw_length & 0x8000),
        "channel": header[2],
        "sequence": header[3],
    }


class TimingDiagnosticBNO086(ObservedBNO086):
    """Exact current read flow with raw timing captured on every failure."""

    def __init__(
        self,
        *args,
        max_cases=50,
        skip_phase2_int_wait=False,
        phase2_delay_us=0.0,
        retry_null_phase2=False,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.max_cases = max_cases
        self.skip_phase2_int_wait = skip_phase2_int_wait
        self.phase2_delay_us = phase2_delay_us
        self.retry_null_phase2 = retry_null_phase2
        self.error_cases = []
        self.recovered_cases = []
        self.read_stats = collections.Counter()
        self.phase2_success_delays_us = []
        self.phase2_null_delays_us = []
        self.int_edges = []
        self._int_callback = None

    def open(self):
        if not super().open():
            return False
        if self.int_pin is not None and self.int_pin >= 0:
            lgpio.gpio_free(self.gpio_handle, self.int_pin)
            result = lgpio.gpio_claim_alert(
                self.gpio_handle,
                self.int_pin,
                lgpio.BOTH_EDGES,
                lgpio.SET_PULL_UP,
            )
            if result != 0:
                raise RuntimeError(f"gpio_claim_alert failed: {result}")

            def record_edge(_chip, _gpio, level, timestamp_ns):
                self.int_edges.append(
                    (timestamp_ns, level, time.monotonic_ns())
                )

            self._int_callback = lgpio.callback(
                self.gpio_handle,
                self.int_pin,
                lgpio.BOTH_EDGES,
                record_edge,
            )
        return True

    def close(self):
        if self._int_callback is not None:
            self._int_callback.cancel()
            self._int_callback = None
        super().close()

    def _save_case(self, trace, error, category):
        trace["error"] = str(error)
        trace["category"] = category
        trace["oserror"] = isinstance(error, OSError)
        trace["errno"] = error.errno if isinstance(error, OSError) else None
        if len(self.error_cases) < self.max_cases:
            self.error_cases.append(trace)
        self._record_io_error(error)

    def read_packet(self, timeout_s=0.020):
        if not self.is_int_asserted():
            return None

        trace = {
            "timestamp": datetime.datetime.now().astimezone().isoformat(
                timespec="microseconds"
            ),
            "monotonic_s": time.monotonic(),
            "phase1_int_before": self.is_int_asserted(),
            "phase1_header": None,
            "phase1": None,
            "phase1_int_after": None,
            "wait_for_int_called": False,
            "phase2_int_before": None,
            "phase1_to_phase2_us": None,
            "phase2_header": None,
            "phase2": None,
            "phase2_retry_header": None,
            "phase2_retry": None,
            "additional_reads": [],
            "stage": "phase1",
            "phase1_start_ns": None,
            "phase1_end_ns": None,
            "phase2_start_ns": None,
            "phase2_end_ns": None,
        }
        self.read_stats["phase1_reads"] += 1

        try:
            with self._io_lock:
                trace["phase1_start_ns"] = time.monotonic_ns()
                header = os.read(self.fd, 4)
                trace["phase1_end_ns"] = time.monotonic_ns()
                phase1_end = time.monotonic()
                trace["phase1_header"] = bytes(header)
                trace["phase1"] = header_fields(header)
                trace["phase1_int_after"] = self.is_int_asserted()
                self.read_stats[
                    "phase1_int_after_low"
                    if trace["phase1_int_after"]
                    else "phase1_int_after_high"
                ] += 1
                if len(header) < 4:
                    self._save_case(trace, "short Phase 1 SHTP header", "syscall/bus")
                    return None

                raw_length = header[0] | (header[1] << 8)
                total_len = raw_length & 0x7FFF
                channel = header[2]
                sequence = header[3]

                if total_len == 0:
                    return None
                if total_len == 4:
                    return total_len, channel, sequence, b""
                if total_len > MAX_SHTP_PACKET:
                    self._save_case(
                        trace,
                        f"invalid SHTP length {total_len}",
                        "SHTP-validation",
                    )
                    return None

                # Preserve the production behavior under diagnosis: if Phase
                # 1 deasserted INT, wait for a new LOW before starting Phase 2.
                if not self.is_int_asserted() and not self.skip_phase2_int_wait:
                    trace["wait_for_int_called"] = True
                    self.read_stats["phase2_wait_calls"] += 1
                    if not self.wait_for_int(timeout_s):
                        trace["phase2_int_before"] = self.is_int_asserted()
                        self._save_case(
                            trace,
                            "H_INTN timeout waiting for Phase 2 continuation",
                            "syscall/bus",
                        )
                        return None

                if self.phase2_delay_us > 0.0:
                    time.sleep(self.phase2_delay_us / 1_000_000.0)

                trace["stage"] = "phase2"
                trace["phase2_int_before"] = self.is_int_asserted()
                trace["phase2_start_ns"] = time.monotonic_ns()
                phase2_start = time.monotonic()
                trace["phase1_to_phase2_us"] = (
                    phase2_start - phase1_end
                ) * 1_000_000.0
                chunk = os.read(self.fd, total_len)
                trace["phase2_end_ns"] = time.monotonic_ns()
                trace["phase2_header"] = bytes(chunk[:4])
                trace["phase2"] = header_fields(chunk[:4])
                if len(chunk) < 4:
                    self._save_case(trace, "short Phase 2 SHTP header", "syscall/bus")
                    return None

                phase2_raw_length = chunk[0] | (chunk[1] << 8)
                phase2_len = phase2_raw_length & 0x7FFF
                phase2_continuation = bool(phase2_raw_length & 0x8000)
                phase2_channel = chunk[2]
                phase2_sequence = chunk[3]
                expected_sequence = (sequence + 1) & 0xFF

                if (
                    not phase2_continuation
                    and phase2_len == 0
                    and self.retry_null_phase2
                ):
                    self.read_stats["phase2_null_headers_observed"] += 1
                    retry_chunk = os.read(self.fd, total_len)
                    trace["phase2_retry_header"] = bytes(retry_chunk[:4])
                    trace["phase2_retry"] = header_fields(retry_chunk[:4])
                    if len(retry_chunk) >= 4:
                        chunk = retry_chunk
                        phase2_raw_length = chunk[0] | (chunk[1] << 8)
                        phase2_len = phase2_raw_length & 0x7FFF
                        phase2_continuation = bool(phase2_raw_length & 0x8000)
                        phase2_channel = chunk[2]
                        phase2_sequence = chunk[3]
                        if (
                            phase2_continuation
                            and phase2_channel == channel
                            and phase2_sequence == expected_sequence
                            and phase2_len == total_len
                        ):
                            self.read_stats["phase2_null_retry_recovered"] += 1
                            if len(self.recovered_cases) < self.max_cases:
                                self.recovered_cases.append(trace.copy())

                if not phase2_continuation:
                    if phase2_len == 0:
                        self.read_stats["phase2_null_headers"] += 1
                        self.phase2_null_delays_us.append(
                            trace["phase1_to_phase2_us"]
                        )
                    else:
                        self.read_stats["phase2_new_noncontinuation_headers"] += 1
                    self._save_case(
                        trace,
                        "Phase 2 continuation bit not set",
                        "SHTP-validation",
                    )
                    return None
                if phase2_channel != channel:
                    self._save_case(
                        trace,
                        f"Phase 2 channel mismatch: {phase2_channel} != {channel}",
                        "SHTP-validation",
                    )
                    return None
                if phase2_sequence != expected_sequence:
                    self._save_case(
                        trace,
                        f"Phase 2 sequence mismatch: {phase2_sequence} != {expected_sequence}",
                        "SHTP-validation",
                    )
                    return None
                if phase2_len != total_len:
                    self._save_case(
                        trace,
                        f"Phase 2 length mismatch: {phase2_len} != {total_len}",
                        "SHTP-validation",
                    )
                    return None

                self.read_stats["phase2_valid_continuations"] += 1
                self.phase2_success_delays_us.append(
                    trace["phase1_to_phase2_us"]
                )

                cargo = bytearray(chunk[4:])
                remaining = (total_len - 4) - len(cargo)
                while remaining > 0:
                    continuation = {
                        "int_before_wait": self.is_int_asserted(),
                        "wait_for_int_called": True,
                        "int_before_read": None,
                        "previous_to_read_us": None,
                        "header": None,
                        "fields": None,
                    }
                    previous_end = time.monotonic()
                    if not self.wait_for_int(timeout_s):
                        continuation["int_before_read"] = self.is_int_asserted()
                        trace["additional_reads"].append(continuation)
                        self._save_case(
                            trace,
                            "H_INTN timeout during continuation chunk",
                            "syscall/bus",
                        )
                        return None
                    continuation["int_before_read"] = self.is_int_asserted()
                    continuation["previous_to_read_us"] = (
                        time.monotonic() - previous_end
                    ) * 1_000_000.0
                    continuation_chunk = os.read(self.fd, remaining + 4)
                    continuation["header"] = bytes(continuation_chunk[:4])
                    continuation["fields"] = header_fields(continuation_chunk[:4])
                    trace["additional_reads"].append(continuation)
                    if len(continuation_chunk) < 4:
                        self._save_case(
                            trace, "short continuation chunk", "syscall/bus"
                        )
                        return None
                    continuation_raw_length = (
                        continuation_chunk[0] | (continuation_chunk[1] << 8)
                    )
                    continuation_len = continuation_raw_length & 0x7FFF
                    continuation_bit = bool(continuation_raw_length & 0x8000)
                    if (
                        not continuation_bit
                        or continuation_chunk[2] != channel
                        or continuation_len != remaining + 4
                    ):
                        self._save_case(
                            trace,
                            "invalid continuation chunk: "
                            f"len={continuation_len}, expected={remaining + 4}",
                            "SHTP-validation",
                        )
                        return None
                    take = min(remaining, len(continuation_chunk) - 4)
                    if take <= 0:
                        break
                    cargo.extend(continuation_chunk[4:4 + take])
                    remaining -= take

                return total_len, channel, phase2_sequence, bytes(cargo)
        except OSError as error:
            self._save_case(trace, error, "syscall/bus")
            return None
        except Exception as error:
            self._save_case(trace, error, "other")
            return None


def raw_hex(value):
    if value is None:
        return "None"
    return " ".join(f"{byte:02X}" for byte in value)


def print_fields(prefix, fields):
    if fields is None:
        print(f"{prefix}=unavailable")
        return
    print(f"{prefix}_raw_length=0x{fields['raw_length']:04X}")
    print(f"{prefix}_total_length={fields['total_length']}")
    print(f"{prefix}_continuation={int(fields['continuation'])}")
    print(f"{prefix}_channel={fields['channel']}")
    print(f"{prefix}_sequence={fields['sequence']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--max-cases", type=int, default=50)
    parser.add_argument(
        "--skip-phase2-int-wait",
        action="store_true",
        help="diagnostic A/B only: issue Phase 2 immediately even when INT is high",
    )
    parser.add_argument(
        "--phase2-delay-us",
        type=float,
        default=0.0,
        help="diagnostic A/B only: delay between Phase 1 and Phase 2",
    )
    parser.add_argument(
        "--retry-null-phase2",
        action="store_true",
        help="diagnostic A/B only: immediately retry a null Phase-2 read once",
    )
    args = parser.parse_args()

    imu = TimingDiagnosticBNO086(
        max_cases=args.max_cases,
        skip_phase2_int_wait=args.skip_phase2_int_wait,
        phase2_delay_us=args.phase2_delay_us,
        retry_null_phase2=args.retry_null_phase2,
    )
    print("=== BNO086 PARTIAL-READ TIMING DIAGNOSTIC ===")
    print(
        "phase2_mode="
        f"{'immediate-no-wait' if args.skip_phase2_int_wait else 'current-conditional-wait'}"
    )
    print(f"diagnostic_phase2_delay_us={args.phase2_delay_us:.1f}")
    print(f"diagnostic_retry_null_phase2={args.retry_null_phase2}")
    try:
        if not imu.begin():
            print(f"setup_error={imu.snapshot()['boot_failure_reason']}")
        else:
            for name, sensor_id, interval_us, report_timeout_s in REPORT_PLAN:
                result = configure_one(
                    imu, sensor_id, interval_us, report_timeout_s
                )
                print(
                    f"setup_{name}={'PASS' if result['enabled'] and result['report'] else 'FAIL'}"
                )

            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline and imu.snapshot()["startup_ready"]:
                imu.update()
                time.sleep(0.001)

        state = imu.snapshot()
        print()
        print(f"aggregate_io_error_count={state['io_error_count']}")
        print(f"i2c_syscall_or_bus_errors={len(imu.i2c_syscall_errors)}")
        print(f"shtp_validation_errors={len(imu.shtp_validation_errors)}")
        print(f"other_transport_errors={len(imu.other_transport_errors)}")
        print(f"captured_error_cases={len(imu.error_cases)}")
        print(f"captured_recovered_cases={len(imu.recovered_cases)}")
        print(f"phase1_reads={imu.read_stats['phase1_reads']}")
        print(f"phase1_int_after_low={imu.read_stats['phase1_int_after_low']}")
        print(f"phase1_int_after_high={imu.read_stats['phase1_int_after_high']}")
        print(f"phase2_wait_calls={imu.read_stats['phase2_wait_calls']}")
        print(
            "phase2_valid_continuations="
            f"{imu.read_stats['phase2_valid_continuations']}"
        )
        print(f"phase2_null_headers={imu.read_stats['phase2_null_headers']}")
        print(
            "phase2_null_headers_observed="
            f"{imu.read_stats['phase2_null_headers_observed']}"
        )
        print(
            "phase2_null_retry_recovered="
            f"{imu.read_stats['phase2_null_retry_recovered']}"
        )
        print(
            "phase2_new_noncontinuation_headers="
            f"{imu.read_stats['phase2_new_noncontinuation_headers']}"
        )
        if imu.phase2_success_delays_us:
            delays = imu.phase2_success_delays_us
            print(
                "phase2_success_delay_us="
                f"min={min(delays):.3f} avg={sum(delays) / len(delays):.3f} "
                f"max={max(delays):.3f}"
            )
        if imu.phase2_null_delays_us:
            delays = imu.phase2_null_delays_us
            print(
                "phase2_null_delay_us="
                f"min={min(delays):.3f} avg={sum(delays) / len(delays):.3f} "
                f"max={max(delays):.3f}"
            )

        for case_number, case in enumerate(imu.error_cases, 1):
            print()
            print(f"--- ERROR CASE {case_number:02d} ---")
            print(f"timestamp={case['timestamp']}")
            print(f"category={case['category']}")
            print(f"error={case['error']}")
            print(f"oserror={case['oserror']} errno={case['errno']}")
            print(f"phase1_raw_header={raw_hex(case['phase1_header'])}")
            print_fields("phase1", case["phase1"])
            print(
                "phase1_int_before="
                f"{int_text(case['phase1_int_before'])}"
            )
            print(
                "phase1_int_after="
                f"{int_text(case['phase1_int_after'])}"
            )
            print(f"wait_for_int_called={case['wait_for_int_called']}")
            phase2_int = case["phase2_int_before"]
            print(
                "phase2_int_before="
                f"{int_text(phase2_int) if phase2_int is not None else 'unavailable'}"
            )
            delay = case["phase1_to_phase2_us"]
            print(
                "phase1_to_phase2_us="
                f"{delay:.3f}" if delay is not None else "phase1_to_phase2_us=unavailable"
            )
            print(f"phase2_raw_header={raw_hex(case['phase2_header'])}")
            print_fields("phase2", case["phase2"])
            print(
                "phase2_retry_raw_header="
                f"{raw_hex(case['phase2_retry_header'])}"
            )
            print_fields("phase2_retry", case["phase2_retry"])
            start_ns = case["phase1_start_ns"]
            end_ns = case["phase2_end_ns"] or time.monotonic_ns()
            if start_ns is not None:
                nearby_edges = [
                    (timestamp_ns, level, callback_ns)
                    for timestamp_ns, level, callback_ns in imu.int_edges
                    if start_ns - 100_000 <= callback_ns <= end_ns + 5_000_000
                ]
                if nearby_edges:
                    edge_text = ", ".join(
                        f"kernel={((timestamp_ns - start_ns) / 1000.0):+.3f}us/"
                        f"callback={((callback_ns - start_ns) / 1000.0):+.3f}us:"
                        f"{'HIGH' if level == 1 else 'LOW'}"
                        for timestamp_ns, level, callback_ns in nearby_edges
                    )
                else:
                    edge_text = "none captured"
                print(f"INT_edges_relative_to_phase1_start={edge_text}")
                if case["phase1_end_ns"] is not None:
                    print(
                        "phase1_read_duration_us="
                        f"{(case['phase1_end_ns'] - start_ns) / 1000.0:.3f}"
                    )
                if (
                    case["phase2_start_ns"] is not None
                    and case["phase2_end_ns"] is not None
                ):
                    print(
                        "phase2_read_duration_us="
                        f"{(case['phase2_end_ns'] - case['phase2_start_ns']) / 1000.0:.3f}"
                    )

        for case_number, case in enumerate(imu.recovered_cases, 1):
            print()
            print(f"--- RECOVERED NULL CASE {case_number:02d} ---")
            print(f"timestamp={case['timestamp']}")
            print(f"phase1_raw_header={raw_hex(case['phase1_header'])}")
            print_fields("phase1", case["phase1"])
            print(f"phase1_int_after={int_text(case['phase1_int_after'])}")
            print(f"wait_for_int_called={case['wait_for_int_called']}")
            print(f"phase2_raw_header={raw_hex(case['phase2_header'])}")
            print_fields("phase2", case["phase2"])
            print(
                "phase2_retry_raw_header="
                f"{raw_hex(case['phase2_retry_header'])}"
            )
            print_fields("phase2_retry", case["phase2_retry"])
    finally:
        imu.close()


if __name__ == "__main__":
    raise SystemExit(main())
