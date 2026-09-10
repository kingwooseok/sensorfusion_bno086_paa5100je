#!/usr/bin/env python3
"""
PMW3901 옵티컬 플로우 스케일러(Scaler) 캘리브레이션 도구
바닥과의 고정 높이 z에서 센서를 자를 대고 일정 거리(예: 10cm)를 이동시켜
정밀한 물리 거리 환산 상수를 자동으로 계산합니다.
"""

import sys
import time
from pmw3901 import PMW3901

def main():
    print("================================================================")
    print(" PMW3901 거리 환산 스케일러(Scaler) 캘리브레이션")
    print("================================================================")

    # 1. 고정 높이 입력
    try:
        raw_h = input("[1] 센서와 바닥 사이의 수직 높이를 입력하세요 (단위 cm, 기본값: 10): ").strip()
        height_cm = float(raw_h) if raw_h else 10.0
    except ValueError:
        height_cm = 10.0

    if height_cm < 8.0:
        print(f"[!] 주의: 센서 초점 거리는 최소 8.0cm 이상이어야 합니다. (현재: {height_cm}cm)")

    # 2. 이동시킬 목표 거리 입력
    try:
        raw_d = input("[2] 테스트로 이동시킬 실제 거리를 입력하세요 (단위 cm, 기본값: 10): ").strip()
        target_dist_cm = float(raw_d) if raw_d else 10.0
    except ValueError:
        target_dist_cm = 10.0

    print(f"\n[*] 설정: 높이 z = {height_cm} cm, 실제 이동 목표 = {target_dist_cm} cm")
    print("[*] CS 핀: 26번 핀 (SPI0 CE1)으로 센서 초기화 중...")

    flow = PMW3901(spi_bus=0, spi_cs=1, height_cm=height_cm)
    if not flow.begin():
        print("[-] 센서 초기화 실패! 배선을 확인해주세요 (26번 핀 CS).")
        flow.close()
        sys.exit(1)

    print("[+] 센서 준비 완료!")
    input("\n>> 센서를 시작 위치에 놓고 [Enter] 키를 누르면 틱 측정을 시작합니다...")

    # 모션 레지스터 초기화 플러시
    flow.read_motion_count()
    accum_x = 0
    accum_y = 0

    print(f">> 이제 센서를 바닥과 평행하게 정확히 {target_dist_cm} cm 이동시킨 후 [Enter]를 누르세요.")
    
    # 비동기로 누적 틱 읽기
    start_time = time.time()
    try:
        import select
        while True:
            dx, dy = flow.read_motion_count()
            accum_x += dx
            accum_y += dy
            print(f"\r현재 누적 틱: X={accum_x:6d}, Y={accum_y:6d}", end="", flush=True)

            # 콘솔 입력 대기 (Non-blocking)
            if sys.stdin in select.select([sys.stdin], [], [], 0.05)[0]:
                sys.stdin.readline()
                break
    except KeyboardInterrupt:
        pass

    import math
    total_ticks = math.sqrt(accum_x**2 + accum_y**2)

    print(f"\n\n[측정 완료]")
    print(f"- 총 누적 틱: {total_ticks:.1f} ticks (X: {accum_x}, Y: {accum_y})")

    if total_ticks == 0:
        print("[-] 움직임이 감지되지 않았습니다. 높이가 너무 낮거나(8cm 미만) 표면에 패턴이 없는지 확인하세요.")
        flow.close()
        return

    # 공식: Scaler = (measured_ticks * height_cm) / actual_distance_cm
    new_scaler = (total_ticks * height_cm) / target_dist_cm

    print(f"================================================================")
    print(f"[*] 추천 보정 Scaler 값: {new_scaler:.2f} (기본값: 355.0)")
    print(f"[*] 앞으로 pmw3901.py나 read_flow.py에서:")
    print(f"    flow = PMW3901(spi_bus=0, spi_cs=1, height_cm={height_cm}, scaler={new_scaler:.2f})")
    print(f"    로 사용하시면 오차 없이 정확한 cm 측정이 가능합니다.")
    print(f"================================================================")

    flow.close()

if __name__ == "__main__":
    main()

