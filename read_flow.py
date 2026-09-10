#!/usr/bin/env python3
"""
PAA5100JE / PMW3901 옵티컬 플로우 + BNO086 IMU 융합 실시간 2D (X, Y) 좌표 추적 시스템
- 시작점을 (0.00 cm, 0.00 cm)으로 설정하고 실시간 전역 2D 위치 좌표 표시
- 실시간 표면 트래킹 품질(Q: SQUAL, 0~255) 모니터링: 초점 흐림 및 바닥 텍스처 이상 즉시 감지
- BNO086 9DoF IMU 융합:
  1) 자력계 보정 Rotation Vector 기반 상대 헤딩 추정
  2) 시작 자세를 기준으로 한 전역 월드 좌표계 2D 회전 변환
  3) 회전 중심과 광류 센서 간 오프셋의 자동 학습/보정
- IMU 미연결 시 단독 PAA5100 모드로 자동 안전 폴백
- 원스톱 캘리브레이션 기능 내장
- 'r' 키 입력 시 원점 (0, 0) 및 Yaw 헤딩 0° 리셋
"""

import os
import sys
import json
import time
import math
import select
import threading
from pmw3901 import PMW3901
from bno086 import (
    BNO086,
    SENSOR_GYROSCOPE_CALIBRATED,
    SENSOR_MAGNETOMETER_CALIBRATED,
    SENSOR_ROTATION_VECTOR,
)

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")
DEFAULT_HEIGHT_CM = 3.5
DEFAULT_SCALER = 400.8

# A PAA5100 sample below this quality is normally texture/focus noise, not a
# trustworthy displacement.  Rejecting it is preferable to integrating a
# permanent position error.
MIN_SQUAL_FOR_POSITION = 25
FLOW_DEADBAND_TICKS = 1
IMU_READY_TIMEOUT_S = 2.5
IMU_STALE_TIMEOUT_S = 0.20
MAX_YAW_RATE_DEG_S = 540.0
MAX_YAW_EXTRAPOLATION_S = 0.030
MIN_FUSION_ACCURACY = 2
MAG_CALIBRATION_INTERVAL_US = 20_000


def wrap_angle_deg(angle):
    """Normalize an angle to [-180, 180), including the 359 -> 0 transition."""
    return (angle + 180.0) % 360.0 - 180.0


def angle_delta_deg(new_angle, old_angle):
    """Signed shortest angular change from old_angle to new_angle."""
    return wrap_angle_deg(new_angle - old_angle)


def quaternion_multiply(lhs, rhs):
    """Hamilton product for quaternions represented as (real, i, j, k)."""
    lw, lx, ly, lz = lhs
    rw, rx, ry, rz = rhs
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def quaternion_conjugate(quaternion):
    real, i, j, k = quaternion
    return (real, -i, -j, -k)


def yaw_from_quaternion(quaternion):
    """Return the Z-axis yaw (degrees) of a normalized (real, i, j, k) quaternion."""
    real, i, j, k = quaternion
    siny_cosp = 2.0 * (real * k + i * j)
    cosy_cosp = 1.0 - 2.0 * (j * j + k * k)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def snapshot_quaternion(snapshot):
    """Extract and normalize the BNO086 quaternion from one state snapshot."""
    quaternion = (
        snapshot["quat_real"],
        snapshot["quat_i"],
        snapshot["quat_j"],
        snapshot["quat_k"],
    )
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm <= 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(component / norm for component in quaternion)


class RelativeHeadingTracker:
    """Heading expressed only as a change from the position at measurement start.

    q_now * conjugate(q_start) removes the BNO086's absolute north heading and
    all fixed installation roll/pitch/yaw offsets.  A PAA5100 axis may therefore
    be mounted at an arbitrary in-plane angle: its start-axis is the world frame.
    """

    def __init__(self, snapshot):
        self.reset(snapshot)

    def reset(self, snapshot):
        self.reference_quaternion = snapshot_quaternion(snapshot)
        self.last_orientation_sequence = snapshot["orientation_sequence"]
        self.last_report_time = snapshot["last_orientation_update"]
        self.last_report_heading = 0.0
        self.yaw_rate_deg_s = 0.0

    def _raw_relative_yaw(self, snapshot):
        current = snapshot_quaternion(snapshot)
        relative = quaternion_multiply(current, quaternion_conjugate(self.reference_quaternion))
        return wrap_angle_deg(yaw_from_quaternion(relative))

    def heading_at(self, snapshot, now):
        """Return heading at `now`, extrapolating at most 30 ms from the IMU report."""
        raw_heading = self._raw_relative_yaw(snapshot)
        sequence = snapshot["orientation_sequence"]
        report_time = snapshot["last_orientation_update"]

        if sequence != self.last_orientation_sequence:
            report_dt = report_time - self.last_report_time
            if 0.001 <= report_dt <= IMU_STALE_TIMEOUT_S:
                measured_rate = angle_delta_deg(raw_heading, self.last_report_heading) / report_dt
                self.yaw_rate_deg_s = max(
                    -MAX_YAW_RATE_DEG_S,
                    min(MAX_YAW_RATE_DEG_S, measured_rate),
                )
            else:
                self.yaw_rate_deg_s = 0.0
            self.last_orientation_sequence = sequence
            self.last_report_time = report_time
            self.last_report_heading = raw_heading

        age = max(0.0, min(now - report_time, MAX_YAW_EXTRAPOLATION_S))
        return wrap_angle_deg(raw_heading + self.yaw_rate_deg_s * age)


class AutoYawFlowCompensator:
    """Calibrate and remove turn-only flow from an offset optical sensor.

    Values are centimeters/radian in the PAA5100 body frame.  Learning is off
    during normal driving and must be explicitly armed for a stationary CW/CCW
    calibration maneuver.
    """

    MIN_TURN_RAD = math.radians(0.35)
    MIN_GYRO_RAD_S = 0.35
    MAX_LEVER_ARM_CM = 20.0
    MAX_TRANSLATION_MARGIN_CM = 0.25
    MIN_SAMPLES = 8
    MIN_DIRECTION_SAMPLES = 2

    def __init__(self):
        self.x_per_rad = 0.0
        self.y_per_rad = 0.0
        self.sample_count = 0
        self.positive_sample_count = 0
        self.negative_sample_count = 0
        self.learning = False
        self.calibrated = False

    def start_calibration(self):
        """Clear the estimate and arm an explicit in-place learning phase."""
        self.x_per_rad = 0.0
        self.y_per_rad = 0.0
        self.sample_count = 0
        self.positive_sample_count = 0
        self.negative_sample_count = 0
        self.learning = True
        self.calibrated = False

    def finish_calibration(self):
        """Freeze the estimate after enough CW and CCW observations."""
        ready = (
            self.sample_count >= self.MIN_SAMPLES
            and self.positive_sample_count >= self.MIN_DIRECTION_SAMPLES
            and self.negative_sample_count >= self.MIN_DIRECTION_SAMPLES
        )
        if ready:
            self.learning = False
            self.calibrated = True
        return ready

    def compensate(self, dx_cm, dy_cm, delta_yaw_rad, gyro_magnitude):
        """Learn only while armed; otherwise apply the frozen estimate."""
        if not self.learning:
            if not self.calibrated:
                return dx_cm, dy_cm
            return (
                dx_cm - self.x_per_rad * delta_yaw_rad,
                dy_cm - self.y_per_rad * delta_yaw_rad,
            )

        # Explicit calibration samples never enter the position estimate,
        # including samples that fail the learning guards below.
        if (
            abs(delta_yaw_rad) < self.MIN_TURN_RAD
            or gyro_magnitude < self.MIN_GYRO_RAD_S
        ):
            return 0.0, 0.0

        max_turn_distance = (
            self.MAX_LEVER_ARM_CM * abs(delta_yaw_rad)
            + self.MAX_TRANSLATION_MARGIN_CM
        )
        raw_distance = math.hypot(dx_cm, dy_cm)
        if raw_distance <= max_turn_distance:
            candidate_x = dx_cm / delta_yaw_rad
            candidate_y = dy_cm / delta_yaw_rad
            candidate_distance = math.hypot(candidate_x, candidate_y)
            if candidate_distance <= self.MAX_LEVER_ARM_CM:
                if self.sample_count == 0:
                    self.x_per_rad = candidate_x
                    self.y_per_rad = candidate_y
                else:
                    alpha = 0.20
                    self.x_per_rad += alpha * (candidate_x - self.x_per_rad)
                    self.y_per_rad += alpha * (candidate_y - self.y_per_rad)
                self.sample_count += 1
                if delta_yaw_rad > 0.0:
                    self.positive_sample_count += 1
                else:
                    self.negative_sample_count += 1

        return 0.0, 0.0


def fusion_reports_are_fresh(snapshot, now):
    """Return whether all three reports required for fusion are current."""
    return (
        snapshot["startup_ready"]
        and snapshot["orientation_sequence"] > 0
        and snapshot["gyro_sequence"] > 0
        and snapshot["mag_sequence"] > 0
        and now - snapshot["last_orientation_update"] < IMU_STALE_TIMEOUT_S
        and now - snapshot["last_gyro_update"] < IMU_STALE_TIMEOUT_S
        and now - snapshot["last_mag_update"] < 0.50
    )


def fusion_accuracy_is_ready(snapshot):
    """Apply the accuracy gate before, but not during, fused measurement."""
    return (
        snapshot["rotation_accuracy"] >= MIN_FUSION_ACCURACY
        and snapshot["mag_accuracy"] >= MIN_FUSION_ACCURACY
    )


def wait_for_fusion_reports(imu, timeout_s=IMU_READY_TIMEOUT_S):
    """Wait until orientation, gyro, and calibrated magnetic reports are live."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snapshot = imu.snapshot()
        if not snapshot["startup_ready"]:
            return None
        if fusion_reports_are_fresh(snapshot, time.monotonic()):
            return snapshot
        time.sleep(0.01)
    return None


def wait_for_calibration_ready(imu):
    """Wait for fresh RV/Mag 2/2 calibration, with an explicit safe fallback."""
    print("[*] BNO086 보정 대기: 센서를 3초간 정지시킨 뒤 Roll/Pitch/Yaw 축으로")
    print("    약 180° 왕복 회전하십시오. RV/Mag가 모두 2 이상이어야 융합을 시작합니다.")
    print("    보정을 중단하고 PAA 단독 모드로 전환하려면 's' 후 Enter를 누르세요.")
    next_status_at = 0.0
    while True:
        now = time.monotonic()
        snapshot = imu.snapshot()
        if not snapshot["startup_ready"]:
            print("\n[!] 보정 중 BNO086 startup_ready가 해제되었습니다.")
            return None
        fresh = fusion_reports_are_fresh(snapshot, now)
        if fresh and fusion_accuracy_is_ready(snapshot):
            print(
                f"\n[+] BNO086 보정 준비 완료: RV/Mag="
                f"{snapshot['rotation_accuracy']}/{snapshot['mag_accuracy']}"
            )
            return snapshot
        if now >= next_status_at:
            print(
                f"\r    RV/Mag={snapshot['rotation_accuracy']}/"
                f"{snapshot['mag_accuracy']}  reports={'fresh' if fresh else 'stale'}",
                end="",
                flush=True,
            )
            next_status_at = now + 0.25
        if sys.stdin in select.select([sys.stdin], [], [], 0.02)[0]:
            if sys.stdin.readline().strip().lower() == "s":
                print("\n[*] 사용자가 IMU 융합 보정을 건너뛰었습니다.")
                return None
        time.sleep(0.005)


def wait_for_new_orientation_report(imu, previous_sequence, timeout_s=0.75):
    """Return a new RV report while every startup fusion input remains valid."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snapshot = imu.snapshot()
        now = time.monotonic()
        if not snapshot["startup_ready"]:
            return None
        if (
            snapshot["orientation_sequence"] > previous_sequence
            and fusion_reports_are_fresh(snapshot, now)
            and fusion_accuracy_is_ready(snapshot)
        ):
            return snapshot
        time.sleep(0.002)
    return None

def load_config():
    """저장된 캘리브레이션 및 높이 설정 불러오기"""
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                data = json.load(f)
                return float(data.get("height_cm", DEFAULT_HEIGHT_CM)), float(data.get("scaler", DEFAULT_SCALER))
        except Exception:
            pass
    return DEFAULT_HEIGHT_CM, DEFAULT_SCALER

def save_config(height_cm, scaler):
    """설정을 config.json에 저장"""
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump({"height_cm": height_cm, "scaler": scaler}, f, indent=4)
        print(f"[+] 보정 설정이 저장되었습니다: 높이={height_cm}cm, Scaler={scaler:.2f}")
    except Exception as e:
        print(f"[!] 설정 저장 실패: {e}")

def run_calibration(flow, height_cm):
    """원스톱 10cm 간편 캘리브레이션"""
    print("\n----------------------------------------------------------------")
    print(" [간편 캘리브레이션 모드]")
    print(f" - 현재 센서 높이 z: {height_cm} cm (PAA5100JE 권장 초점 높이: 2.0 ~ 3.0cm)")
    try:
        dist_input = input(" - 테스트로 이동시킬 실제 거리(cm) 입력 [기본값: 10]: ").strip()
        target_dist_cm = float(dist_input) if dist_input else 10.0
    except ValueError:
        target_dist_cm = 10.0

    print(f"[*] 목표: 센서를 바닥과 수평을 유지하며 직선으로 정확히 {target_dist_cm} cm 이동합니다.")
    print("    (팁: 센서가 기울어지지 않도록 평평한 블록이나 자에 밀착해 밀어주세요)")
    input(">> 센서를 시작 위치에 놓고 [Enter] 키를 누르세요...")

    flow.read_motion_count()
    accum_x = 0
    accum_y = 0

    print(f">> 센서를 자를 대고 {target_dist_cm} cm 이동시킨 뒤 [Enter]를 누르세요.")

    while True:
        dx, dy = flow.read_motion_count()
        accum_x += dx
        accum_y += dy
        q = flow.last_squal
        print(f"\r   측정 중... 누적 틱: X={accum_x:6d}, Y={accum_y:6d} | 표면품질(Q)={q:3d}", end="", flush=True)

        if sys.stdin in select.select([sys.stdin], [], [], 0.05)[0]:
            sys.stdin.readline()
            break

    total_ticks = math.sqrt(accum_x**2 + accum_y**2)
    print(f"\n   -> 측정 완료: 총 누적 틱 = {total_ticks:.1f} ticks (최종 품질 Q={flow.last_squal})")

    if total_ticks > 0:
        new_scaler = (total_ticks * height_cm) / target_dist_cm
        flow.scaler = new_scaler
        save_config(height_cm, new_scaler)
        print(f"[+] 보정 완료! 새 Scaler: {new_scaler:.2f}")
    else:
        print("[!] 틱이 감지되지 않았습니다. 기본 Scaler를 유지합니다.")

    print("----------------------------------------------------------------\n")
    time.sleep(1)

def run_measurement(flow, imu=None):
    """Track position in the PAA5100 start frame, with guarded IMU fusion."""
    has_imu = imu is not None
    imu_stop_event = threading.Event()
    imu_thread = None
    fusion_enabled = False
    heading_tracker = None
    turn_compensator = AutoYawFlowCompensator()
    reference_snapshot = None

    pos_x_cm = 0.0
    pos_y_cm = 0.0
    ticks_x = 0
    ticks_y = 0
    last_flow_heading = 0.0
    imu_was_stale = False
    accuracy_was_low = False
    accuracy_low_events = 0
    accuracy_low_started = None
    accuracy_low_total_s = 0.0

    if has_imu:
        def imu_worker():
            while not imu_stop_event.is_set():
                try:
                    imu.update()
                except Exception:
                    # The foreground loop will detect stale reports and refuse
                    # to mix body-frame data into a world-frame trajectory.
                    pass
                time.sleep(0.002)

        imu_thread = threading.Thread(target=imu_worker, daemon=True)
        imu_thread.start()

    try:
        if has_imu:
            print("[*] BNO086 Rotation Vector / Gyro / Magnetometer 리포트 대기 중...")
            reference_snapshot = wait_for_fusion_reports(imu)
            if reference_snapshot is None:
                print(
                    "[!] IMU 리포트 3종이 모두 수신되지 않았습니다: "
                    f"{imu.snapshot()['boot_failure_reason'] or 'fresh input timeout'}"
                )
                print(f"    진단: {imu.status_summary()}")
                print("    이번 측정은 PAA5100 단독 좌표계로 실행합니다.")
            else:
                calibration_snapshot = wait_for_calibration_ready(imu)
                if calibration_snapshot is None:
                    print("    이번 측정은 PAA5100 단독 좌표계로 실행합니다.")
                else:
                    print(
                        "[*] 차량을 출발 위치에서 정지시킨 뒤 [Enter]를 누르세요. "
                        "이 자세가 X/Y 및 헤딩의 기준입니다."
                    )
                    input()
                    sequence_at_enter = imu.snapshot()["orientation_sequence"]
                    latest_snapshot = wait_for_new_orientation_report(
                        imu, sequence_at_enter
                    )
                    if latest_snapshot is None:
                        print(
                            "[!] 기준 입력 이후의 새 Rotation Vector를 확인하지 못해 "
                            "PAA5100 단독 모드로 실행합니다."
                        )
                    else:
                        # A software reference is sufficient and deterministic;
                        # hardware all-axis tare would only introduce an
                        # asynchronous frame-change race here.
                        reference_snapshot = latest_snapshot
                        fusion_enabled = True

                if fusion_enabled:
                    heading_tracker = RelativeHeadingTracker(reference_snapshot)
                    print("[*] Enter 이후의 새 quaternion을 상대 헤딩 기준으로 설정했습니다.")

        print("================================================================")
        if fusion_enabled:
            print(f" [PAA5100 + BNO086 자력계 보정 융합 2D 좌표 추적] (높이 z = {flow.height_cm} cm)")
            print("  - 시작 quaternion 대비 상대 헤딩을 사용하므로 절대 북쪽/장착 yaw 오프셋을 제거합니다.")
            print("  - 회전 오프셋 학습은 일반 주행 중 꺼져 있습니다.")
            print("  - 'c' 입력 후 제자리 CW/CCW 회전으로 학습하고, 다시 'c'를 입력하면 계수를 고정합니다.")
            print("  - IMU가 stale이면 잘못된 바디 좌표 누적을 막기 위해 해당 구간의 광류를 버립니다.")
        else:
            print(f" [PAA5100 단독 2D 좌표 추적 모드] (높이 z = {flow.height_cm} cm)")
            print("  - 센서 자체 바디 좌표계(Local Body Frame) 기준입니다.")
        print(" - 시작 기준점: (X, Y) = (0.00 cm, 0.00 cm)")
        print(f" - 표면 품질 Q가 {MIN_SQUAL_FOR_POSITION} 미만인 샘플은 누적하지 않습니다.")
        print(" - 'r' 입력 후 [Enter]를 누르면 현재 위치와 상대 헤딩 기준을 (0, 0)으로 리셋합니다.")
        print(" - Ctrl + C 를 누르면 측정을 종료합니다.")
        print("================================================================\n")

        # Clear any delta latched while the operator was setting the reference.
        flow.read_motion_count()

        while True:
            current_time = time.monotonic()

            # Burst Read: acquire one atomically latched optical-flow sample.
            raw_dx_ticks, raw_dy_ticks = flow.read_motion_count()
            ticks_x += raw_dx_ticks
            ticks_y += raw_dy_ticks
            q = flow.last_squal
            q_flag = "!" if q < MIN_SQUAL_FOR_POSITION else " "
            snapshot = imu.snapshot() if fusion_enabled else None

            # Accuracy is diagnostic after startup.  A transient 2 -> 1 does
            # not make a fresh quaternion stale and must not create a position
            # gap.  Only report freshness/reset state gates runtime integration.
            if fusion_enabled:
                accuracy_low = not fusion_accuracy_is_ready(snapshot)
                if accuracy_low and not accuracy_was_low:
                    accuracy_low_events += 1
                    accuracy_low_started = current_time
                elif not accuracy_low and accuracy_was_low:
                    accuracy_low_total_s += current_time - accuracy_low_started
                    accuracy_low_started = None
                accuracy_was_low = accuracy_low

            # Suppress stationary one-tick noise before it can become a slowly
            # growing X error.  Raw totals above remain available for diagnosis.
            dx_ticks = 0 if abs(raw_dx_ticks) <= FLOW_DEADBAND_TICKS else raw_dx_ticks
            dy_ticks = 0 if abs(raw_dy_ticks) <= FLOW_DEADBAND_TICKS else raw_dy_ticks

            if q < MIN_SQUAL_FOR_POSITION:
                if fusion_enabled:
                    # Keep heading synchronized even while rejecting a bad
                    # image sample; otherwise the next good sample would span
                    # the whole turn and receive an incorrect yaw correction.
                    heading_fresh = (
                        snapshot["startup_ready"]
                        and current_time - snapshot["last_orientation_update"] < IMU_STALE_TIMEOUT_S
                    )
                    if heading_fresh:
                        last_flow_heading = heading_tracker.heading_at(snapshot, current_time)
                        # A discarded low-quality sample is a safe boundary at
                        # which to re-anchor after an IMU outage.
                        imu_was_stale = False
                    else:
                        imu_was_stale = True
                    status_str = (
                        f"\r[POS] X:{pos_x_cm:+6.1f} Y:{pos_y_cm:+6.1f}cm | "
                        f"Q:{q:3d}{q_flag}| A:{snapshot['rotation_accuracy']}/"
                        f"{snapshot['mag_accuracy']} | "
                        f"[{'저품질 샘플 무시' if heading_fresh else '저품질 + IMU stale: 샘플 무시'}]"
                    )
                else:
                    status_str = (
                        f"\r[POS] X:{pos_x_cm:+6.1f} Y:{pos_y_cm:+6.1f}cm | "
                        f"Q:{q:3d}{q_flag}| [저품질 샘플 무시]"
                    )
            elif fusion_enabled:
                imu_active = (
                    snapshot["startup_ready"]
                    and
                    current_time - snapshot["last_orientation_update"] < IMU_STALE_TIMEOUT_S
                    and current_time - snapshot["last_gyro_update"] < IMU_STALE_TIMEOUT_S
                )
                if not imu_active:
                    # Do not fall back mid-run to unrotated body deltas.  That
                    # is the direct cause of a large, irreversible X/Y jump
                    # when the vehicle happens to be turning during IMU loss.
                    imu_was_stale = True
                    status_str = (
                        f"\r[POS] X:{pos_x_cm:+6.1f} Y:{pos_y_cm:+6.1f}cm | "
                        f"Q:{q:3d}{q_flag}| [IMU stale: 샘플 무시]"
                    )
                else:
                    current_heading = heading_tracker.heading_at(snapshot, current_time)
                    if imu_was_stale:
                        # Re-anchor heading after a data outage so its missing
                        # turn is not incorrectly assigned to one flow sample.
                        last_flow_heading = current_heading
                        imu_was_stale = False

                    delta_heading_deg = angle_delta_deg(current_heading, last_flow_heading)
                    heading_mid_deg = wrap_angle_deg(last_flow_heading + delta_heading_deg * 0.5)
                    last_flow_heading = current_heading

                    dx_body_cm = (dx_ticks * flow.height_cm) / flow.scaler
                    dy_body_cm = (dy_ticks * flow.height_cm) / flow.scaler
                    gyro_magnitude = math.sqrt(
                        snapshot["gyro_x"] ** 2
                        + snapshot["gyro_y"] ** 2
                        + snapshot["gyro_z"] ** 2
                    )

                    # Do not apply the former fixed gyro_x/gyro_y tilt formula:
                    # it assumes an exact BNO-to-PAA axis installation and can
                    # create X drift when the modules are mounted differently.
                    # The learned turn term below is axis-free because it is
                    # estimated directly in the PAA5100 frame.
                    dx_body_cm, dy_body_cm = turn_compensator.compensate(
                        dx_body_cm,
                        dy_body_cm,
                        math.radians(delta_heading_deg),
                        gyro_magnitude,
                    )

                    yaw_rad = math.radians(heading_mid_deg)
                    cos_y = math.cos(yaw_rad)
                    sin_y = math.sin(yaw_rad)
                    dx_world_cm = dx_body_cm * cos_y - dy_body_cm * sin_y
                    dy_world_cm = dx_body_cm * sin_y + dy_body_cm * cos_y
                    pos_x_cm += dx_world_cm
                    pos_y_cm += dy_world_cm

                    status_str = (
                        f"\r[POS] X:{pos_x_cm:+6.1f} Y:{pos_y_cm:+6.1f}cm | "
                        f"Q:{q:3d}{q_flag}| H:{current_heading:+6.1f}° "
                        f"A:{snapshot['rotation_accuracy']}/{snapshot['mag_accuracy']}"
                        f"{'!' if not fusion_accuracy_is_ready(snapshot) else ' '} "
                        f"Off:{'CAL' if turn_compensator.learning else 'LOCK' if turn_compensator.calibrated else 'OFF'}"
                        f"/{turn_compensator.sample_count:02d} | "
                        f"T:{ticks_x:+6d},{ticks_y:+6d}"
                    )
            else:
                # Standalone mode is deliberately kept in its local PAA frame.
                # It never silently claims that an unrotated value is world X/Y.
                pos_x_cm += (dx_ticks * flow.height_cm) / flow.scaler
                pos_y_cm += (dy_ticks * flow.height_cm) / flow.scaler
                status_str = (
                    f"\r[POS] X:{pos_x_cm:+6.1f} Y:{pos_y_cm:+6.1f}cm | "
                    f"Q:{q:3d}{q_flag}| [PAA local] | T:{ticks_x:+6d},{ticks_y:+6d}"
                )

            print(status_str.ljust(118), end="", flush=True)

            if sys.stdin in select.select([sys.stdin], [], [], 0.005)[0]:
                cmd = sys.stdin.readline().strip().lower()
                if cmd == 'r':
                    pos_x_cm = 0.0
                    pos_y_cm = 0.0
                    ticks_x = 0
                    ticks_y = 0
                    if fusion_enabled:
                        snapshot = imu.snapshot()
                        if (
                            snapshot["startup_ready"]
                            and current_time - snapshot["last_orientation_update"] < IMU_STALE_TIMEOUT_S
                        ):
                            heading_tracker.reset(snapshot)
                            last_flow_heading = 0.0
                            imu_was_stale = False
                            print("\n[!] 좌표와 상대 헤딩 기준점을 (0.00 cm, 0.00 cm)으로 리셋했습니다!\n")
                        else:
                            print("\n[!] IMU가 stale 상태라 헤딩 기준은 유지하고 좌표만 리셋했습니다.\n")
                    else:
                        print("\n[!] 좌표 기준점을 (0.00 cm, 0.00 cm)으로 리셋했습니다!\n")
                elif cmd == 'c':
                    if not fusion_enabled:
                        print("\n[!] IMU 융합 모드에서만 회전 오프셋을 보정할 수 있습니다.\n")
                    elif not turn_compensator.learning:
                        turn_compensator.start_calibration()
                        current_snapshot = imu.snapshot()
                        if (
                            current_snapshot["startup_ready"]
                            and current_time - current_snapshot["last_orientation_update"]
                            < IMU_STALE_TIMEOUT_S
                        ):
                            last_flow_heading = heading_tracker.heading_at(
                                current_snapshot, current_time
                            )
                            imu_was_stale = False
                        print(
                            "\n[!] 회전 오프셋 보정 시작: 차량 위치를 고정하고 "
                            "CW/CCW 양방향으로 천천히 회전하세요."
                        )
                        print("    보정 중 광류는 위치에 누적되지 않습니다. 완료 후 다시 'c'를 입력하세요.\n")
                    elif turn_compensator.finish_calibration():
                        current_snapshot = imu.snapshot()
                        if (
                            current_snapshot["startup_ready"]
                            and current_time - current_snapshot["last_orientation_update"]
                            < IMU_STALE_TIMEOUT_S
                        ):
                            last_flow_heading = heading_tracker.heading_at(
                                current_snapshot, current_time
                            )
                            imu_was_stale = False
                        else:
                            imu_was_stale = True
                        print(
                            "\n[+] 회전 오프셋 보정 고정: "
                            f"samples={turn_compensator.sample_count}, "
                            f"CW/CCW={turn_compensator.positive_sample_count}/"
                            f"{turn_compensator.negative_sample_count}, "
                            f"offset=({turn_compensator.x_per_rad:+.3f}, "
                            f"{turn_compensator.y_per_rad:+.3f}) cm/rad\n"
                        )
                    else:
                        print(
                            "\n[!] 표본 부족: "
                            f"total={turn_compensator.sample_count}/"
                            f"{turn_compensator.MIN_SAMPLES}, "
                            f"CW/CCW={turn_compensator.positive_sample_count}/"
                            f"{turn_compensator.negative_sample_count}. "
                            "양방향 제자리 회전을 계속하세요.\n"
                        )

    except KeyboardInterrupt:
        print("\n\n================================================================")
        print(" [최종 정지 좌표 요약]")
        print(f" - 최종 좌표 (X, Y) : ( {pos_x_cm:+.2f} cm,  {pos_y_cm:+.2f} cm )")
        print(f" - 원점으로부터의 거리: {math.sqrt(pos_x_cm**2 + pos_y_cm**2):.2f} cm")
        print(f" - 총 원시 틱       : X = {ticks_x}, Y = {ticks_y}")
        print(f" - 최종 표면 품질 Q : {flow.last_squal}")
        if fusion_enabled:
            snapshot = imu.snapshot()
            summary_time = time.monotonic()
            heading = heading_tracker.heading_at(snapshot, summary_time)
            final_accuracy_low_s = accuracy_low_total_s
            if accuracy_was_low and accuracy_low_started is not None:
                final_accuracy_low_s += summary_time - accuracy_low_started
            print(
                f" - 최종 상대 헤딩   : {heading:+.1f}° "
                f"(RV/Mag 정확도 {snapshot['rotation_accuracy']}/{snapshot['mag_accuracy']})"
            )
            print(
                f" - 회전 오프셋 학습 : {turn_compensator.sample_count} samples, "
                f"({turn_compensator.x_per_rad:+.2f}, {turn_compensator.y_per_rad:+.2f}) cm/rad, "
                f"state={'learning' if turn_compensator.learning else 'locked' if turn_compensator.calibrated else 'disabled'}"
            )
            print(
                f" - 운행 중 accuracy 저하: {accuracy_low_events} events, "
                f"{final_accuracy_low_s:.3f} s (적분은 계속)"
            )
        print("================================================================")
    finally:
        if has_imu:
            imu_stop_event.set()
            if imu_thread is not None:
                imu_thread.join(timeout=0.25)

def main():
    print("================================================================")
    print(" PAA5100JE 옵티컬 플로우 + BNO086 IMU 좌표 추적 시스템")
    print("================================================================")

    # 1. 이전 설정 불러오기
    height_cm, scaler = load_config()

    # 2. PAA5100 센서 초기화 (CS: 26번 핀 / SPI0 CE1)
    flow = PMW3901(spi_bus=0, spi_cs=1, height_cm=height_cm, scaler=scaler, sensor_type="PAA5100")
    print("[*] PAA5100JE 센서 연결 확인 중 (CS: 26번 핀 / CE1)...")
    if not flow.begin():
        print("[-] PAA5100JE 센서 초기화 실패!")
        print("    [배선 점검: Pin 1(3.3V), Pin 14(GND), Pin 19(MOSI), Pin 21(MISO), Pin 23(SCLK), Pin 26(CS)]")
        flow.close()
        sys.exit(1)
    print(f"[+] PAA5100JE 인식 성공! (현재 설정: 높이 {height_cm}cm, Scaler {scaler:.1f})")

    print("----------------------------------------------------------------")
    print(" [1] PAA5100 단독 모드 (평행이동 전용, 지연 0ms, 최고 반응속도, [Enter])")
    print(" [2] PAA5100 + BNO086 IMU 융합 모드 (비동기 스레드)")
    print(" [3] 10cm 이동으로 캘리브레이션")
    print(" [4] 센서 높이 z 변경 (현재: {:.1f}cm)".format(height_cm))
    print("----------------------------------------------------------------")

    choice = input("선택 번호 입력 [1/2/3/4, 기본 1]: ").strip()

    if choice == "3":
        run_calibration(flow, height_cm)
        choice = input("측정을 바로 시작하시겠습니까? [1:단독(기본) / 2:IMU융합]: ").strip()
    elif choice == "4":
        try:
            new_h = float(input(f"새로운 높이 z (cm) 입력 [현재 {height_cm}]: ").strip())
            flow.set_height(new_h)
            height_cm = new_h
            save_config(height_cm, flow.scaler)
        except ValueError:
            print("[!] 입력 오류, 기존 높이를 유지합니다.")
        
        cal_ask = input("새 높이에 맞춰 캘리브레이션도 진행하시겠습니까? (y/N): ").strip().lower()
        if cal_ask == "y":
            run_calibration(flow, height_cm)
        choice = input("측정을 바로 시작하시겠습니까? [1:단독(기본) / 2:IMU융합]: ").strip()

    # 3. IMU 사용 여부 결정
    imu = None
    if choice == "2":
        print("[*] BNO086 IMU 센서 연결 시도 중 (I2C-1, INT: Pin 12)...")
        imu = BNO086(i2c_bus=1, i2c_addr=0x4B, int_pin=18, rst_pin=23)
        if imu.begin():
            # Configure one report at a time.  A valid FC reply *and* a real
            # input sample are required before the next command is sent, so a
            # reset/bus fault is identified at its first triggering command
            # rather than being hidden by a later generic timeout.
            report_plan = (
                ("Rotation Vector", SENSOR_ROTATION_VECTOR,
                 imu.enable_rotation_vector, 10000, 0.75),
                ("Gyro", SENSOR_GYROSCOPE_CALIBRATED,
                 imu.enable_gyro, 10000, 0.75),
                ("Magnetometer", SENSOR_MAGNETOMETER_CALIBRATED,
                 imu.enable_magnetometer, MAG_CALIBRATION_INTERVAL_US, 1.00),
            )
            report_setup_ok = True
            failed_step = None
            for report_name, sensor_id, enable_report, interval_us, timeout_s in report_plan:
                if not enable_report(interval_us):
                    report_setup_ok = False
                    failed_step = f"{report_name} FC 설정 응답"
                    break
                if not imu.wait_for_sensor_report(sensor_id, timeout_s):
                    report_setup_ok = False
                    failed_step = f"{report_name} 입력 리포트"
                    break

            calibration_started = report_setup_ok and imu.start_dynamic_calibration()
            if report_setup_ok and calibration_started:
                print("[+] BNO086 리포트 활성화: Rotation Vector(100Hz), Gyro(100Hz), Magnetometer(50Hz)")
                print("[*] 자이로/자력계 동적 보정을 시작했습니다. 실제 accuracy 확인은 측정 시작 전에 수행합니다.")
            else:
                if failed_step is None:
                    failed_step = "동적 보정 명령 응답"
                print(
                    f"[!] BNO086 {failed_step} 확인 실패: "
                    f"{imu.boot_failure_reason or 'unknown failure'}"
                )
                print(f"    진단: {imu.status_summary()}")
                print("    IMU 융합을 비활성화하고 PAA5100 단독 좌표계로 안전 전환합니다.")
                imu.close()
                imu = None
        else:
            print(
                "[!] BNO086 융합 시작 실패: "
                f"{imu.boot_failure_reason or 'unknown failure'}"
            )
            print(f"    진단: {imu.status_summary()}")
            print("    PAA5100 단독 모드로 안전 실행합니다.")
            imu.close()
            imu = None
    else:
        print("[+] PAA5100 단독 초고속 모드로 시작합니다 (I2C 간섭 0%).")

    # 4. 좌표 추적 실행
    try:
        run_measurement(flow, imu)
    finally:
        flow.close()
        if imu:
            imu.close()

if __name__ == "__main__":
    main()
