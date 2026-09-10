#!/usr/bin/env python3
"""Mock-check the PAA5100/PMW3901 segmented SPI transport.

This test deliberately does not claim electrical timing or sensor operation:
no PAA5100 is connected.  It verifies the Linux SPI message emitted by the
project driver, the RX pointer plumbing, burst decoding, and ID validation.
"""

from __future__ import annotations

import ctypes
import errno
import struct
import sys
import types
from unittest import mock

import bno_test_support  # noqa: F401  # Adds the project root to sys.path.
import pmw3901


class FakeSpi:
    def __init__(self, fd=73):
        self.fd = fd

    def fileno(self):
        return self.fd

    def xfer(self, *_args, **_kwargs):
        raise AssertionError("legacy combined xfer path was used")

    def xfer2(self, *_args, **_kwargs):
        raise AssertionError("legacy combined xfer2 path was used")

    def xfer3(self, *_args, **_kwargs):
        raise AssertionError("legacy combined xfer3 path was used")


class ConfigurableSpi(FakeSpi):
    def __init__(self, *, fail_no_cs=False):
        super().__init__()
        self.fail_no_cs = fail_no_cs
        self.open_calls = []
        self.max_speed_hz = None
        self.mode = None
        self._no_cs = False
        self.closed = False

    def open(self, bus, chip_select):
        self.open_calls.append((bus, chip_select))

    @property
    def no_cs(self):
        return self._no_cs

    @no_cs.setter
    def no_cs(self, value):
        if self.fail_no_cs:
            raise OSError(errno.ENOTSUP, "no_cs unavailable")
        self._no_cs = value

    def close(self):
        self.closed = True


class FakeCs:
    def __init__(self, *args, events=None, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.events = events if events is not None else []
        self.closed = False

    def on(self):
        self.events.append("LOW")

    def off(self):
        self.events.append("HIGH")

    def close(self):
        self.closed = True


class IoctlRecorder:
    def __init__(self, response=b"", *, result=None, error=None):
        self.response = bytes(response)
        self.result = result
        self.error = error
        self.calls = []

    def __call__(self, fd, request, message, mutate_flag=True):
        if self.error is not None:
            raise self.error
        if not isinstance(message, bytearray) or not mutate_flag:
            raise AssertionError("ioctl descriptor array was not mutable")

        array_type = pmw3901._SpiIocTransfer * 2
        if len(message) != ctypes.sizeof(array_type):
            raise AssertionError(f"unexpected ioctl message size: {len(message)}")
        descriptors = array_type.from_buffer(message)

        snapshot = {
            "fd": fd,
            "request": request,
            "segments": [],
        }
        for descriptor in descriptors:
            fields = {
                name: getattr(descriptor, name)
                for name, _ctype in pmw3901._SpiIocTransfer._fields_
            }
            fields["tx"] = (
                ctypes.string_at(descriptor.tx_buf, descriptor.len)
                if descriptor.tx_buf
                else b""
            )
            snapshot["segments"].append(fields)

        data_segment = descriptors[1]
        if self.response:
            if not data_segment.rx_buf:
                raise AssertionError("mock response has no RX destination")
            if len(self.response) != data_segment.len:
                raise AssertionError("mock response length does not match RX phase")
            ctypes.memmove(data_segment.rx_buf, self.response, len(self.response))

        self.calls.append(snapshot)
        if self.result is not None:
            return self.result
        return sum(segment.len for segment in descriptors)


def make_sensor():
    sensor = pmw3901.PMW3901.__new__(pmw3901.PMW3901)
    sensor.spi = FakeSpi()
    sensor.speed_hz = 2_000_000
    sensor.cs_device = None
    sensor.last_squal = 0
    sensor.last_shutter = 0
    return sensor


def assert_common_message(call, *, command, data_length, delay_us, read):
    if call["fd"] != 73:
        raise AssertionError("SpiDev file descriptor was not forwarded")
    if call["request"] != 0x40406B00:
        raise AssertionError(f"wrong SPI_IOC_MESSAGE(2): 0x{call['request']:08X}")

    address, data = call["segments"]
    if address["tx"] != command or address["len"] != 1:
        raise AssertionError("address phase mismatch")
    if address["rx_buf"] != 0:
        raise AssertionError("address phase unexpectedly has an RX buffer")
    if data["len"] != data_length:
        raise AssertionError("data phase length mismatch")
    if bool(data["rx_buf"]) != read:
        raise AssertionError("data phase RX direction mismatch")

    for index, segment in enumerate((address, data)):
        if segment["speed_hz"] != 2_000_000:
            raise AssertionError(f"segment {index} speed mismatch")
        if segment["bits_per_word"] != 8:
            raise AssertionError(f"segment {index} bits-per-word mismatch")
        if segment["cs_change"] != 0:
            raise AssertionError(f"segment {index} would release CS early")
        for field in ("tx_nbits", "rx_nbits", "word_delay_usecs", "pad"):
            if segment[field] != 0:
                raise AssertionError(f"segment {index} unexpected {field}")
    if address["delay_usecs"] != delay_us or data["delay_usecs"] != 0:
        raise AssertionError("address-to-data delay mismatch")


def test_linux_uapi_layout():
    if ctypes.sizeof(pmw3901._SpiIocTransfer) != 32:
        raise AssertionError("spi_ioc_transfer is not 32 bytes")
    for field, expected in pmw3901._SPI_IOC_TRANSFER_OFFSETS.items():
        actual = getattr(pmw3901._SpiIocTransfer, field).offset
        if actual != expected:
            raise AssertionError(f"{field} offset {actual}, expected {expected}")
    if pmw3901._spi_ioc_message(2) != 0x40406B00:
        raise AssertionError("SPI_IOC_MESSAGE(2) ioctl encoding mismatch")


def test_register_write_message():
    sensor = make_sensor()
    recorder = IoctlRecorder()
    with (
        mock.patch("pmw3901.fcntl.ioctl", side_effect=recorder),
        mock.patch("pmw3901.time.sleep", return_value=None),
    ):
        sensor.register_write(0x12, 0xA5)
    if len(recorder.calls) != 1:
        raise AssertionError("register write was not exactly one ioctl message")
    call = recorder.calls[0]
    assert_common_message(
        call,
        command=b"\x92",
        data_length=1,
        delay_us=sensor.REGISTER_WRITE_ADDRESS_DELAY_US,
        read=False,
    )
    if call["segments"][1]["tx"] != b"\xA5":
        raise AssertionError("register write value mismatch")


def test_register_read_message_and_pointer():
    sensor = make_sensor()
    recorder = IoctlRecorder(b"\x7A")
    with (
        mock.patch("pmw3901.fcntl.ioctl", side_effect=recorder),
        mock.patch("pmw3901.time.sleep", return_value=None),
    ):
        result = sensor.register_read(0xA3)
    if result != 0x7A or len(recorder.calls) != 1:
        raise AssertionError("register RX pointer/result mismatch")
    call = recorder.calls[0]
    assert_common_message(
        call,
        command=b"\x23",
        data_length=1,
        delay_us=sensor.REGISTER_READ_ADDRESS_DELAY_US,
        read=True,
    )
    if call["segments"][1]["tx"] != b"\x00":
        raise AssertionError("register read did not transmit one dummy byte")


def burst_payload(*, motion, x, y, squal, shutter):
    return struct.pack(
        "<BBhhBBBBBB",
        0x80 if motion else 0x00,
        0x00,
        x,
        y,
        squal,
        1,
        2,
        3,
        (shutter >> 8) & 0xFF,
        shutter & 0xFF,
    )


def test_motion_burst_message_and_decode():
    sensor = make_sensor()
    recorder = IoctlRecorder(
        burst_payload(motion=True, x=-1234, y=2345, squal=77, shutter=0x1234)
    )
    with mock.patch("pmw3901.fcntl.ioctl", side_effect=recorder):
        result = sensor.read_motion_count()
    if result != (-1234, 2345):
        raise AssertionError(f"signed burst delta mismatch: {result}")
    if sensor.last_squal != 77 or sensor.last_shutter != 0x1234:
        raise AssertionError("burst SQUAL/shutter mismatch")
    if len(recorder.calls) != 1:
        raise AssertionError("motion burst was not exactly one ioctl message")
    call = recorder.calls[0]
    assert_common_message(
        call,
        command=b"\x16",
        data_length=12,
        delay_us=sensor.MOTION_BURST_ADDRESS_DELAY_US,
        read=True,
    )
    if call["segments"][1]["tx"] != bytes(12):
        raise AssertionError("motion burst dummy clocks mismatch")

    sensor = make_sensor()
    recorder = IoctlRecorder(
        burst_payload(motion=False, x=-32768, y=32767, squal=9, shutter=0xABCD)
    )
    with mock.patch("pmw3901.fcntl.ioctl", side_effect=recorder):
        result = sensor.read_motion_count()
    if result != (0, 0):
        raise AssertionError("motion flag clear did not suppress deltas")
    if sensor.last_squal != 9 or sensor.last_shutter != 0xABCD:
        raise AssertionError("diagnostics were lost when motion flag was clear")


def test_transport_failures_propagate():
    sensor = make_sensor()
    bus_error = OSError(errno.EREMOTEIO, "Remote I/O error")
    with mock.patch(
        "pmw3901.fcntl.ioctl",
        side_effect=IoctlRecorder(error=bus_error),
    ):
        try:
            sensor.register_read(0x00)
        except OSError as exc:
            if exc.errno != errno.EREMOTEIO:
                raise AssertionError("ioctl errno was changed") from exc
        else:
            raise AssertionError("ioctl OSError was swallowed")

    sensor = make_sensor()
    with mock.patch(
        "pmw3901.fcntl.ioctl",
        side_effect=IoctlRecorder(b"\x49", result=1),
    ):
        try:
            sensor.register_read(0x00)
        except OSError as exc:
            if exc.errno != errno.EIO or "short SPI message" not in str(exc):
                raise AssertionError("short message error was not classified") from exc
        else:
            raise AssertionError("short SPI message was accepted")


def test_manual_cs_encloses_message_and_recovers():
    cases = (
        (IoctlRecorder(b"\x49"), None),
        (
            IoctlRecorder(error=OSError(errno.EREMOTEIO, "Remote I/O error")),
            errno.EREMOTEIO,
        ),
        (IoctlRecorder(b"\x49", result=1), errno.EIO),
    )
    for recorder, expected_errno in cases:
        sensor = make_sensor()
        events = []
        sensor.cs_device = FakeCs(events=events)

        def recorded_ioctl(*args, **kwargs):
            events.append("IOCTL")
            return recorder(*args, **kwargs)

        with (
            mock.patch("pmw3901.fcntl.ioctl", side_effect=recorded_ioctl),
            mock.patch("pmw3901.time.sleep", return_value=None),
        ):
            try:
                result = sensor.register_read(0x00)
            except OSError as exc:
                if expected_errno is None or exc.errno != expected_errno:
                    raise
            else:
                if expected_errno is not None:
                    raise AssertionError("manual-CS failure case unexpectedly passed")
                if result != 0x49:
                    raise AssertionError("manual-CS read result mismatch")

        if events != ["LOW", "IOCTL", "HIGH"]:
            raise AssertionError(f"manual CS sequence mismatch: {events}")


def test_constructor_cs_configuration():
    hardware_spi = ConfigurableSpi()
    with mock.patch("pmw3901.spidev.SpiDev", return_value=hardware_spi):
        sensor = pmw3901.PMW3901(spi_bus=0, spi_cs=1, speed_hz=2_000_000)
    if hardware_spi.open_calls != [(0, 1)]:
        raise AssertionError("hardware CE1 device selection mismatch")
    if hardware_spi.mode != 0b11 or hardware_spi.max_speed_hz != 2_000_000:
        raise AssertionError("SPI mode/speed configuration mismatch")
    if hardware_spi.no_cs:
        raise AssertionError("hardware-CS path unexpectedly disabled controller CS")
    sensor.close()

    gpiozero = types.ModuleType("gpiozero")
    created_cs = []

    def output_device(*args, **kwargs):
        device = FakeCs(*args, **kwargs)
        created_cs.append(device)
        return device

    gpiozero.OutputDevice = output_device
    manual_spi = ConfigurableSpi()
    with (
        mock.patch.dict(sys.modules, {"gpiozero": gpiozero}),
        mock.patch("pmw3901.spidev.SpiDev", return_value=manual_spi),
    ):
        sensor = pmw3901.PMW3901(cs_gpio=25)
    if manual_spi.open_calls != [(0, 0)] or manual_spi.no_cs is not True:
        raise AssertionError("manual-CS path did not enable spidev no_cs")
    if len(created_cs) != 1 or created_cs[0].args != (25,):
        raise AssertionError("manual CS GPIO was not configured")
    if created_cs[0].kwargs != {"active_high": False, "initial_value": False}:
        raise AssertionError("manual CS GPIO polarity/default mismatch")
    sensor.close()
    if not manual_spi.closed or not created_cs[0].closed:
        raise AssertionError("manual-CS resources were not closed")

    failing_spi = ConfigurableSpi(fail_no_cs=True)
    with (
        mock.patch.dict(sys.modules, {"gpiozero": gpiozero}),
        mock.patch("pmw3901.spidev.SpiDev", return_value=failing_spi),
    ):
        try:
            pmw3901.PMW3901(cs_gpio=25)
        except RuntimeError as exc:
            if "no_cs" not in str(exc):
                raise AssertionError("no_cs setup failure was misclassified") from exc
        else:
            raise AssertionError("manual CS accepted unavailable no_cs")
    if not failing_spi.closed or not created_cs[-1].closed:
        raise AssertionError("failed manual-CS setup leaked resources")


def run_begin_case(chip_id, inverse_id):
    sensor = make_sensor()
    sensor.sensor_type = "PAA5100"
    sensor.register_write = mock.Mock()
    sensor.register_read = mock.Mock(
        side_effect=[chip_id, 0x01, inverse_id, 0, 0, 0, 0, 0]
    )
    sensor._init_paa5100 = mock.Mock()
    with mock.patch("pmw3901.time.sleep", return_value=None):
        result = sensor.begin()
    return result, sensor


def test_begin_product_and_inverse_id():
    for chip_id, inverse_id, expected in (
        (0x49, 0xB6, True),
        (0x49, 0x00, False),
        (0x00, 0xB6, False),
    ):
        result, sensor = run_begin_case(chip_id, inverse_id)
        if result is not expected:
            raise AssertionError(
                f"begin({chip_id:02X}/{inverse_id:02X}) returned {result}"
            )
        if bool(sensor._init_paa5100.called) is not expected:
            raise AssertionError("sensor init was not gated by both ID bytes")
        sensor.register_write.assert_called_once_with(
            sensor.REG_POWER_UP_RESET, 0x5A
        )
        expected_reads = [
            mock.call(sensor.REG_CHIP_ID),
            mock.call(sensor.REG_REVISION),
            mock.call(sensor.REG_CHIP_ID_INV),
        ]
        if expected:
            expected_reads.extend(mock.call(reg) for reg in range(0x02, 0x07))
        if sensor.register_read.call_args_list != expected_reads:
            raise AssertionError(
                f"begin register order mismatch: {sensor.register_read.call_args_list}"
            )


TESTS = (
    ("Linux SPI UAPI layout", test_linux_uapi_layout),
    ("register write segmented message", test_register_write_message),
    ("register read turnaround/RX pointer", test_register_read_message_and_pointer),
    ("motion burst segmented decode", test_motion_burst_message_and_decode),
    ("ioctl and short-transfer failures", test_transport_failures_propagate),
    ("manual CS continuity/error cleanup", test_manual_cs_encloses_message_and_recovers),
    ("constructor hardware/manual CS setup", test_constructor_cs_configuration),
    ("Product ID + inverse ID gate", test_begin_product_and_inverse_id),
)


def main():
    print("=== PAA5100 SPI PROTOCOL MOCK TEST ===")
    failures = []
    for name, test in TESTS:
        try:
            test()
        except Exception as exc:
            failures.append((name, exc))
            print(f"{name}: FAIL ({type(exc).__name__}: {exc})")
        else:
            print(f"{name}: PASS")
    print()
    print(f"passed={len(TESTS) - len(failures)}/{len(TESTS)}")
    print(f"CODE/MOCK RESULT={'PASS' if not failures else 'FAIL'}")
    print("PAA5100 HARDWARE RESULT=UNVERIFIED (sensor not connected)")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
