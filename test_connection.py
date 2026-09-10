#!/usr/bin/env python3
"""
PMW3901 SPI 연결 및 레지스터 진단 스크립트
기본 CS: 26번 핀 (SPI0 CE1)
"""

import os
import sys
import time
import spidev

def test_spi(spi_cs=1):
    cs_pin_name = "26번 핀 (SPI0 CE1 / GPIO 7)" if spi_cs == 1 else "24번 핀 (SPI0 CE0 / GPIO 8)"
    print("==================================================")
    print(f" PMW3901 센서 SPI 하드웨어 진단 도구")
    print(f" 테스트 대상 CS: {cs_pin_name}")
    print("==================================================")

    spidev_path = f"/dev/spidev0.{spi_cs}"
    if not os.path.exists(spidev_path):
        print(f"[-] 오류: {spidev_path} 디바이스를 찾을 수 없습니다.")
        return False
    print(f"[+] SPI 인터페이스 확인: {spidev_path}")

    try:
        spi = spidev.SpiDev()
        spi.open(0, spi_cs)
        spi.max_speed_hz = 2000000
        spi.mode = 0b11  # SPI Mode 3
        print(f"[+] SPI 디바이스 연결 성공 (Mode: 3, Max Speed: {spi.max_speed_hz} Hz)")
    except Exception as e:
        print(f"[-] SPI 디바이스 열기 실패: {e}")
        return False

    try:
        # Power on reset (0x3A = 0x5A)
        spi.xfer2([0x3A | 0x80, 0x5A])
        time.sleep(0.01)

        # Chip ID (0x00) 및 Inverse Chip ID (0x5F) 읽기
        res_id = spi.xfer2([0x00 & 0x7F, 0x00])
        chip_id = res_id[1]

        res_inv = spi.xfer2([0x5F & 0x7F, 0x00])
        chip_id_inv = res_inv[1]

        print(f"[*] 읽어온 Chip ID        : 0x{chip_id:02X} (정상 기대값: 0x49)")
        print(f"[*] 읽어온 Inverse Chip ID: 0x{chip_id_inv:02X} (정상 기대값: 0xB6)")

        if chip_id == 0x49 and chip_id_inv == 0xB6:
            print("\n[SUCCESS] PMW3901 센서가 정상적으로 인식되었습니다! 하드웨어 연결 완벽.")
            return True
        elif chip_id in (0x00, 0xFF) and chip_id_inv in (0x00, 0xFF):
            print("\n[FAIL] 센서 응답이 0x00 또는 0xFF입니다.")
            print("       - 3.3V (1번 핀) 및 GND (6번 핀) 전원 연결을 확인하세요.")
            print("       - MOSI(Pin 19), MISO(Pin 21), SCLK(Pin 23), CS(Pin 26) 배선이 정확한지 확인하세요.")
            return False
        else:
            print(f"\n[FAIL] 알 수 없는 Chip ID(0x{chip_id:02X})가 반환되었습니다.")
            return False
    finally:
        spi.close()

if __name__ == "__main__":
    # 기본값: 26번 핀 (spi_cs=1)
    target_cs = 1
    if len(sys.argv) > 1 and sys.argv[1] == "0":
        target_cs = 0
    test_spi(target_cs)

