#!/usr/bin/env python3
"""
PAA5100JE / PMW3901 Optical Flow Sensor Driver for Raspberry Pi 5 & 4
Supports:
- PAA5100JE (15mm ~ 35mm macro lens, native PixArt secret sauce + dynamic trimming)
- PMW3901 (80mm+ lens, Bitcraze sequence)
- Atomic Burst Reading (0x16) for simultaneous X, Y, and SQUAL (Surface Quality) latching
"""

import ctypes
import errno
import fcntl
import math
import struct
import time

import spidev


class _SpiIocTransfer(ctypes.Structure):
    """Linux ``struct spi_ioc_transfer`` from <linux/spi/spidev.h>."""

    _fields_ = (
        ("tx_buf", ctypes.c_uint64),
        ("rx_buf", ctypes.c_uint64),
        ("len", ctypes.c_uint32),
        ("speed_hz", ctypes.c_uint32),
        ("delay_usecs", ctypes.c_uint16),
        ("bits_per_word", ctypes.c_uint8),
        ("cs_change", ctypes.c_uint8),
        ("tx_nbits", ctypes.c_uint8),
        ("rx_nbits", ctypes.c_uint8),
        ("word_delay_usecs", ctypes.c_uint8),
        ("pad", ctypes.c_uint8),
    )


_SPI_IOC_MAGIC = ord("k")
_IOC_WRITE = 1
_IOC_TYPESHIFT = 8
_IOC_SIZESHIFT = 16
_IOC_DIRSHIFT = 30
_IOC_SIZEBITS = 14

_SPI_IOC_TRANSFER_OFFSETS = {
    "tx_buf": 0,
    "rx_buf": 8,
    "len": 16,
    "speed_hz": 20,
    "delay_usecs": 24,
    "bits_per_word": 26,
    "cs_change": 27,
    "tx_nbits": 28,
    "rx_nbits": 29,
    "word_delay_usecs": 30,
    "pad": 31,
}


def _spi_ioc_message(transfer_count):
    """Return the Linux SPI_IOC_MESSAGE(N) request number."""
    message_size = transfer_count * ctypes.sizeof(_SpiIocTransfer)
    if transfer_count <= 0 or message_size >= (1 << _IOC_SIZEBITS):
        raise ValueError("invalid SPI transfer count")
    return (
        (_IOC_WRITE << _IOC_DIRSHIFT)
        | (message_size << _IOC_SIZESHIFT)
        | (_SPI_IOC_MAGIC << _IOC_TYPESHIFT)
    )


if (
    ctypes.sizeof(_SpiIocTransfer) != 32
    or any(
        getattr(_SpiIocTransfer, field).offset != expected
        for field, expected in _SPI_IOC_TRANSFER_OFFSETS.items()
    )
):
    raise RuntimeError("unexpected Linux spi_ioc_transfer layout")


class PMW3901:
    CHIP_ID = 0x49          # 0x49 (PAA5100JE and PMW3901 share chip ID)
    CHIP_ID_INVERSE = 0xB6

    # Registers
    REG_CHIP_ID = 0x00
    REG_REVISION = 0x01
    REG_MOTION = 0x02
    REG_DELTA_X_L = 0x03
    REG_DELTA_X_H = 0x04
    REG_DELTA_Y_L = 0x05
    REG_DELTA_Y_H = 0x06
    REG_MOTION_BURST = 0x16
    REG_POWER_UP_RESET = 0x3A
    REG_ORIENTATION = 0x5B
    REG_CHIP_ID_INV = 0x5F

    DEFAULT_SCALER = 400.8

    REGISTER_WRITE_ADDRESS_DELAY_US = 50
    REGISTER_READ_ADDRESS_DELAY_US = 500
    MOTION_BURST_ADDRESS_DELAY_US = 50

    def __init__(self, spi_bus=0, spi_cs=1, cs_gpio=None, speed_hz=2000000, height_cm=3.5, scaler=400.8, sensor_type="PAA5100"):
        self.spi_bus = spi_bus
        self.spi_cs = spi_cs
        self.cs_gpio = cs_gpio
        self.speed_hz = speed_hz
        self.height_cm = height_cm
        self.scaler = scaler
        self.sensor_type = sensor_type.upper()
        self.cs_device = None

        self.last_squal = 0
        self.last_shutter = 0

        if self.cs_gpio is not None:
            try:
                import gpiozero
                self.cs_device = gpiozero.OutputDevice(self.cs_gpio, active_high=False, initial_value=False)
            except Exception as e:
                raise RuntimeError(f"CS GPIO {self.cs_gpio} 초기화 실패: {e}")

        self.spi = spidev.SpiDev()
        self.spi.open(self.spi_bus, self.spi_cs if self.cs_gpio is None else 0)
        self.spi.max_speed_hz = self.speed_hz
        self.spi.mode = 0b11

        if self.cs_gpio is not None:
            try:
                self.spi.no_cs = True
            except OSError as exc:
                self.spi.close()
                if self.cs_device:
                    self.cs_device.close()
                raise RuntimeError(
                    "manual CS requires spidev no_cs support"
                ) from exc

    def set_height(self, height_cm: float):
        """바닥과의 고정 높이 설정 (cm 단위)"""
        if self.sensor_type == "PAA5100":
            if height_cm < 1.5 or height_cm > 3.5:
                print(f"[경고] PAA5100JE의 초점 거리는 1.5cm ~ 3.5cm (15~35mm)입니다. 설정값({height_cm}cm)에서는 초점 흐림으로 인한 오차가 발생할 수 있습니다.")
        else:
            if height_cm < 8.0:
                print(f"[경고] PMW3901의 최소 초점 거리는 8.0cm입니다. 설정값({height_cm}cm)에서는 오차가 발생할 수 있습니다.")
        self.height_cm = height_cm

    def _cs_low(self):
        if self.cs_device:
            self.cs_device.on()

    def _cs_high(self):
        if self.cs_device:
            self.cs_device.off()

    def _transfer_with_turnaround(self, command, data, delay_us, *, read):
        """Execute address and data phases as one SPI message.

        Both transfers have ``cs_change=0``.  Linux therefore keeps hardware
        CS asserted across the address-phase delay and the following data
        phase.  With an optional manual CS GPIO, ``no_cs=True`` disables the
        controller CS and the same two-transfer message is enclosed by one
        manual assertion.
        """
        command = bytes(command)
        data = bytes(data)
        if not command or not data:
            raise ValueError("SPI command and data phases must not be empty")
        if delay_us < 0 or delay_us > 0xFFFF:
            raise ValueError("SPI turnaround delay must fit uint16")

        command_tx = (ctypes.c_ubyte * len(command)).from_buffer_copy(command)
        data_tx = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
        data_rx = (ctypes.c_ubyte * len(data))() if read else None

        transfers = (_SpiIocTransfer * 2)()
        transfers[0].tx_buf = ctypes.addressof(command_tx)
        transfers[0].len = len(command)
        transfers[0].speed_hz = self.speed_hz
        transfers[0].delay_usecs = delay_us
        transfers[0].bits_per_word = 8
        transfers[0].cs_change = 0

        transfers[1].tx_buf = ctypes.addressof(data_tx)
        if data_rx is not None:
            transfers[1].rx_buf = ctypes.addressof(data_rx)
        transfers[1].len = len(data)
        transfers[1].speed_hz = self.speed_hz
        transfers[1].bits_per_word = 8
        transfers[1].cs_change = 0

        # fcntl.ioctl accepts a mutable buffer and then returns the kernel's
        # integer result.  The transfer descriptors contain pointers to the
        # three ctypes buffers above, which remain alive until ioctl returns.
        message = bytearray(
            ctypes.string_at(ctypes.addressof(transfers), ctypes.sizeof(transfers))
        )
        expected_length = len(command) + len(data)

        if self.cs_device:
            self._cs_low()
        try:
            transferred = fcntl.ioctl(
                self.spi.fileno(), _spi_ioc_message(2), message, True
            )
        finally:
            if self.cs_device:
                self._cs_high()

        if transferred != expected_length:
            raise OSError(
                errno.EIO,
                f"short SPI message: {transferred}/{expected_length} bytes",
            )
        return bytes(data_rx) if data_rx is not None else b""

    def register_write(self, reg, value):
        """레지스터에 1바이트 쓰기 (MSB 1 설정)"""
        reg_addr = reg | 0x80
        self._transfer_with_turnaround(
            bytes((reg_addr,)),
            bytes((value,)),
            self.REGISTER_WRITE_ADDRESS_DELAY_US,
            read=False,
        )
        time.sleep(0.0001)

    def register_read(self, reg):
        """레지스터에서 1바이트 읽기 (MSB 0 설정)"""
        reg_addr = reg & ~0x80
        result = self._transfer_with_turnaround(
            bytes((reg_addr,)),
            b"\x00",
            self.REGISTER_READ_ADDRESS_DELAY_US,
            read=True,
        )
        time.sleep(0.00005)
        return result[0]

    def _bulk_write(self, data):
        for i in range(0, len(data), 2):
            reg = data[i]
            val = data[i + 1]
            if reg == -1:  # Sleep ms
                time.sleep(val / 1000.0)
            else:
                self.register_write(reg, val)

    def begin(self):
        """센서 초기화 및 통신 검증"""
        self._cs_high()
        time.sleep(0.001)
        self._cs_low()
        time.sleep(0.001)
        self._cs_high()
        time.sleep(0.001)

        # 1. Power on reset (0x3A 레지스터에 0x5A)
        self.register_write(self.REG_POWER_UP_RESET, 0x5A)
        time.sleep(0.05)

        # 2. Product ID와 inverse Product ID를 함께 검증
        chip_id = self.register_read(self.REG_CHIP_ID)
        revision = self.register_read(self.REG_REVISION)
        inverse_id = self.register_read(self.REG_CHIP_ID_INV)

        if chip_id != self.CHIP_ID or inverse_id != self.CHIP_ID_INVERSE:
            return False

        # 3. 모션 레지스터 플러시
        for reg in range(0x02, 0x07):
            self.register_read(reg)
        time.sleep(0.001)

        # 4. 센서 종류별 최적화 레지스터 시퀀스 로드
        if self.sensor_type == "PAA5100":
            self._init_paa5100()
        else:
            self._init_pmw3901()

        return True

    def _init_paa5100(self):
        """PAA5100JE 매크로 렌즈 전용 PixArt 시크릿 소스 및 동적 캘리브레이션"""
        self._bulk_write([
            0x7F, 0x00,
            0x55, 0x01,
            0x50, 0x07,
            0x7F, 0x0E,
            0x43, 0x10
        ])
        if self.register_read(0x67) & 0b10000000:
            self.register_write(0x48, 0x04)
        else:
            self.register_write(0x48, 0x02)

        self._bulk_write([
            0x7F, 0x00,
            0x51, 0x7B,
            0x50, 0x00,
            0x55, 0x00,
            0x7F, 0x0E
        ])

        if self.register_read(0x73) == 0x00:
            c1 = self.register_read(0x70)
            c2 = self.register_read(0x71)
            if c1 <= 28:
                c1 += 14
            elif c1 > 28:
                c1 += 11
            c1 = max(0, min(0x3F, c1))
            c2 = (c2 * 45) // 100
            self._bulk_write([
                0x7F, 0x00,
                0x61, 0xAD,
                0x51, 0x70,
                0x7F, 0x0E
            ])
            self.register_write(0x70, c1)
            self.register_write(0x71, c2)

        self._bulk_write([
            0x7F, 0x00,
            0x61, 0xAD,
            0x7F, 0x03,
            0x40, 0x00,
            0x7F, 0x05,
            0x41, 0xB3,
            0x43, 0xF1,
            0x45, 0x14,
            0x5F, 0x34,
            0x7B, 0x08,
            0x5E, 0x34,
            0x5B, 0x11,
            0x6D, 0x11,
            0x45, 0x17,
            0x70, 0xE5,
            0x71, 0xE5,
            0x7F, 0x06,
            0x44, 0x1B,
            0x40, 0xBF,
            0x4E, 0x3F,
            0x7F, 0x08,
            0x66, 0x44,
            0x65, 0x20,
            0x6A, 0x3A,
            0x61, 0x05,
            0x62, 0x05,
            0x7F, 0x09,
            0x4F, 0xAF,
            0x5F, 0x40,
            0x48, 0x80,
            0x49, 0x80,
            0x57, 0x77,
            0x60, 0x78,
            0x61, 0x78,
            0x62, 0x08,
            0x63, 0x50,
            0x7F, 0x0A,
            0x45, 0x60,
            0x7F, 0x00,
            0x4D, 0x11,
            0x55, 0x80,
            0x74, 0x21,
            0x75, 0x1F,
            0x4A, 0x78,
            0x4B, 0x78,
            0x44, 0x08,
            0x45, 0x50,
            0x64, 0xFF,
            0x65, 0x1F,
            0x7F, 0x14,
            0x65, 0x67,
            0x66, 0x08,
            0x63, 0x70,
            0x6F, 0x1C,
            0x7F, 0x15,
            0x48, 0x48,
            0x7F, 0x07,
            0x41, 0x0D,
            0x43, 0x14,
            0x4B, 0x0E,
            0x45, 0x0F,
            0x44, 0x42,
            0x4C, 0x80,
            0x7F, 0x10,
            0x5B, 0x02,
            0x7F, 0x07,
            0x40, 0x41,
            -1, 10, # Wait 10ms
            0x7F, 0x00,
            0x32, 0x00,
            0x7F, 0x07,
            0x40, 0x40,
            0x7F, 0x06,
            0x68, 0xF0,
            0x69, 0x00,
            0x7F, 0x0D,
            0x48, 0xC0,
            0x6F, 0xD5,
            0x7F, 0x00,
            0x5B, 0xA0,
            0x4E, 0xA8,
            0x5A, 0x90,
            0x40, 0x80,
            0x73, 0x1F,
            -1, 10, # Wait 10ms
            0x73, 0x00
        ])

    def _init_pmw3901(self):
        """Bitcraze PMW3901 표준 최적화 레지스터"""
        init_1 = [
            (0x7F, 0x00), (0x61, 0xAD), (0x7F, 0x03), (0x40, 0x00),
            (0x7F, 0x05), (0x41, 0xB3), (0x43, 0xF1), (0x45, 0x14),
            (0x5B, 0x32), (0x5F, 0x34), (0x7B, 0x08), (0x7F, 0x06),
            (0x44, 0x1B), (0x40, 0xBF), (0x4E, 0x3F), (0x7F, 0x08),
            (0x65, 0x20), (0x6A, 0x18), (0x7F, 0x09), (0x4F, 0xAF),
            (0x5F, 0x40), (0x48, 0x80), (0x49, 0x80), (0x57, 0x77),
            (0x60, 0x78), (0x61, 0x78), (0x62, 0x08), (0x63, 0x50),
            (0x7F, 0x0A), (0x45, 0x60), (0x7F, 0x00), (0x4D, 0x11),
            (0x55, 0x80), (0x74, 0x1F), (0x75, 0x1F), (0x4A, 0x78),
            (0x4B, 0x78), (0x44, 0x08), (0x45, 0x50), (0x64, 0xFF),
            (0x65, 0x1F), (0x7F, 0x14), (0x65, 0x60), (0x66, 0x08),
            (0x63, 0x78), (0x7F, 0x15), (0x48, 0x58), (0x7F, 0x07),
            (0x41, 0x0D), (0x43, 0x14), (0x4B, 0x0E), (0x45, 0x0F),
            (0x44, 0x42), (0x4C, 0x80), (0x7F, 0x10), (0x5B, 0x02),
            (0x7F, 0x07), (0x40, 0x41), (0x70, 0x00)
        ]
        for reg, val in init_1:
            self.register_write(reg, val)
        time.sleep(0.1)
        init_2 = [
            (0x32, 0x44), (0x7F, 0x07), (0x40, 0x40), (0x7F, 0x06),
            (0x62, 0xF0), (0x63, 0x00), (0x7F, 0x0D), (0x48, 0xC0),
            (0x6F, 0xD5), (0x7F, 0x00), (0x5B, 0xA0), (0x4E, 0xA8),
            (0x5A, 0x50), (0x40, 0x80)
        ]
        for reg, val in init_2:
            self.register_write(reg, val)

    def read_motion_count(self):
        """
        Burst Read(0x16)를 통해 12바이트를 단일 SPI 메시지로 원자적(Atomic) 래치하여 읽음.
        프레임 누락, 레지스터 불일치, 방향성 비대칭 왜곡 제거.
        :return: (delta_x, delta_y)
        """
        data = self._transfer_with_turnaround(
            bytes((self.REG_MOTION_BURST,)),
            bytes(12),
            self.MOTION_BURST_ADDRESS_DELAY_US,
            read=True,
        )

        (dr, obs,
         x, y, squal,
         raw_sum, raw_max, raw_min,
         sh_up, sh_lo) = struct.unpack("<BBhhBBBBBB", data)

        self.last_squal = squal
        self.last_shutter = (sh_up << 8) | sh_lo

        # 모션 발생 여부 플래그 (dr bit 7) 확인
        if dr & 0x80:
            return x, y
        return 0, 0

    def read_motion_distance_cm(self, height_cm: float = None):
        """물리적 이동 거리(cm) 계산: distance_cm = (delta_ticks * height_cm) / scaler"""
        h = height_cm if height_cm is not None else self.height_cm
        dx_ticks, dy_ticks = self.read_motion_count()
        dx_cm = (dx_ticks * h) / self.scaler
        dy_cm = (dy_ticks * h) / self.scaler
        return dx_cm, dy_cm

    def read_motion_full(self, height_cm: float = None):
        """틱과 물리적 이동 거리 반환: (dx_ticks, dy_ticks, dx_cm, dy_cm)"""
        h = height_cm if height_cm is not None else self.height_cm
        dx_ticks, dy_ticks = self.read_motion_count()
        dx_cm = (dx_ticks * h) / self.scaler
        dy_cm = (dy_ticks * h) / self.scaler
        return dx_ticks, dy_ticks, dx_cm, dy_cm

    def set_led(self, led_on: bool):
        time.sleep(0.05)
        self.register_write(0x7F, 0x14)
        self.register_write(0x6F, 0x1C if led_on else 0x00)
        self.register_write(0x7F, 0x00)

    def close(self):
        if self.spi:
            self.spi.close()
        if self.cs_device:
            self.cs_device.close()
