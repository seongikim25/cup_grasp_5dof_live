import math
from dataclasses import dataclass
from typing import Tuple

from config import CFG


@dataclass
class ServoPose6:
    base: int
    shoulder: int
    elbow: int
    wrist: int
    left_gripper: int
    right_gripper: int


class EmpiricalManipulatorKinematics:
    """
    반경 기반 선형 보간 모델.

    입력:
        패널 좌표 x_mm, y_mm

    출력:
        AX-12A position 값 6개
        [Base, Shoulder, Elbow, Wrist, Left_Gripper, Right_Gripper]

    현재 로봇 설치:
        로봇팔 = 패널 상단 중앙부
        로봇 정면 = 패널 아래쪽 방향

    핵심:
        - ID1 Base는 로봇 기준 방향각으로 계산
        - ID2/ID3/ID4는 Wizard 실제 절대각을 반경 기준으로 보간
        - 보간된 절대각을 AX-12A tick으로 변환
    """

    def __init__(self):
        self.robot_x = float(CFG.ROBOT_ORIGIN_X_MM)
        self.robot_y = float(CFG.ROBOT_ORIGIN_Y_MM)

        self.min_r = float(CFG.MIN_RADIUS_MM)
        self.mid_r = float(CFG.MID_RADIUS_MM)
        self.max_r = float(CFG.MAX_RADIUS_MM)

        self.tick_per_deg = CFG.AX_TICK_RANGE / CFG.AX_DEG_RANGE

        self.validate_calibration()

    def panel_to_robot(self, x_mm: float, y_mm: float) -> Tuple[float, float]:
        """
        패널 LT 원점 좌표를 로봇 중심 좌표로 변환.

        패널 좌표:
            x: 오른쪽 +
            y: 아래쪽 +

        로봇 좌표:
            dx: 오른쪽 +
            dy: 로봇 전방 +
        """
        dx = x_mm - self.robot_x
        dy = y_mm - self.robot_y
        return dx, dy

    def radius(self, x_mm: float, y_mm: float) -> float:
        dx, dy = self.panel_to_robot(x_mm, y_mm)
        return math.sqrt(dx * dx + dy * dy)

    def base_angle_deg(self, x_mm: float, y_mm: float) -> float:
        """
        로봇 정면을 0도로 봄.
        오른쪽 회전 +
        왼쪽 회전 -
        """
        dx, dy = self.panel_to_robot(x_mm, y_mm)
        return math.degrees(math.atan2(dx, dy))

    def base_angle_to_tick(self, angle_deg: float) -> int:
        """
        Base 전용 상대각 -> tick 변환.
        Base는 512를 정면 0도로 사용.
        """
        value = CFG.BASE_ZERO + CFG.BASE_DIR * angle_deg * self.tick_per_deg
        return int(round(value))

    def angle_to_ax_tick_absolute(self, angle_deg: float) -> int:
        """
        Wizard에 표시되는 실제 각도(deg)를 AX-12A tick으로 변환.
        AX-12A: 0~300도 = 0~1023 tick
        """
        tick = round((angle_deg / CFG.AX_DEG_RANGE) * CFG.AX_TICK_RANGE)
        return int(tick)

    def clamp_with_warning(self, name: str, value: int, low: int, high: int) -> int:
        if value < low:
            print(f"[LIMIT] {name}: {value} < {low}, clamped to {low}")
            return low

        if value > high:
            print(f"[LIMIT] {name}: {value} > {high}, clamped to {high}")
            return high

        return value

    def check_reachable(self, x_mm: float, y_mm: float):
        r = self.radius(x_mm, y_mm)

        if r < self.min_r:
            return False, f"WARNING! TOO CLOSE r={r / 10.0:.1f}cm"

        if r > self.max_r:
            return False, f"WARNING! OUT OF RANGE r={r / 10.0:.1f}cm"

        return True, f"OK r={r / 10.0:.1f}cm"

    def interpolate_joint_angles_by_radius(self, r_mm: float) -> Tuple[float, float, float]:
        """
        반경-관절각 다점 테이블 기반 선형 보간.
        중간 구간이 단순 MIN-MAX 직선으로 맞지 않을 때 사용.
        """
        table = getattr(CFG, "RADIUS_JOINT_TABLE", None)

        if getattr(CFG, "USE_MULTI_RADIUS_TABLE", False) and table is not None:
            table = sorted(table, key=lambda row: row[0])

            r = max(table[0][0], min(table[-1][0], r_mm))

            for i in range(len(table) - 1):
                r0, s0, e0, w0 = table[i]
                r1, s1, e1, w1 = table[i + 1]

                if r0 <= r <= r1:
                    if r1 <= r0:
                        return s0, e0, w0

                    t = (r - r0) / (r1 - r0)

                    shoulder_deg = s0 + (s1 - s0) * t
                    elbow_deg = e0 + (e1 - e0) * t
                    wrist_deg = w0 + (w1 - w0) * t

                    print(
                        "[IK MultiTable] "
                        f"r={r:.1f}mm | segment {r0:.1f}-{r1:.1f}, t={t:.3f} | "
                        f"deg=({shoulder_deg:.2f}, {elbow_deg:.2f}, {wrist_deg:.2f})"
                    )

                    return shoulder_deg, elbow_deg, wrist_deg

            _, s, e, w = table[-1]
            return s, e, w

        # fallback: MIN-MID-MAX piecewise
        r = max(self.min_r, min(self.max_r, r_mm))

        min_pose = CFG.MIN_RADIUS_JOINT_DEG
        mid_pose = CFG.MID_RADIUS_JOINT_DEG
        max_pose = CFG.MAX_RADIUS_JOINT_DEG

        if r <= self.mid_r:
            t = (r - self.min_r) / (self.mid_r - self.min_r)
            return (
                min_pose[0] + (mid_pose[0] - min_pose[0]) * t,
                min_pose[1] + (mid_pose[1] - min_pose[1]) * t,
                min_pose[2] + (mid_pose[2] - min_pose[2]) * t,
            )

        t = (r - self.mid_r) / (self.max_r - self.mid_r)
        return (
            mid_pose[0] + (max_pose[0] - mid_pose[0]) * t,
            mid_pose[1] + (max_pose[1] - mid_pose[1]) * t,
            mid_pose[2] + (max_pose[2] - mid_pose[2]) * t,
        )

    def make_servo_pose(
        self,
        x_mm: float,
        y_mm: float,
        left_gripper: int,
        right_gripper: int,
    ) -> ServoPose6:
        r = self.radius(x_mm, y_mm)

        # Base
        base_deg = self.base_angle_deg(x_mm, y_mm)
        base = self.base_angle_to_tick(base_deg)

        # ID2, ID3, ID4: Wizard 절대각 보간
        shoulder_deg, elbow_deg, wrist_deg = self.interpolate_joint_angles_by_radius(r)

        shoulder = self.angle_to_ax_tick_absolute(shoulder_deg)
        elbow = self.angle_to_ax_tick_absolute(elbow_deg)
        wrist = self.angle_to_ax_tick_absolute(wrist_deg)

        # Clamp
        base = self.clamp_with_warning("Base", base, CFG.BASE_MIN, CFG.BASE_MAX)
        shoulder = self.clamp_with_warning("Shoulder", shoulder, CFG.SHOULDER_MIN, CFG.SHOULDER_MAX)
        elbow = self.clamp_with_warning("Elbow", elbow, CFG.ELBOW_MIN, CFG.ELBOW_MAX)
        wrist = self.clamp_with_warning("Wrist", wrist, CFG.WRIST_MIN, CFG.WRIST_MAX)

        left_gripper = self.clamp_with_warning(
            "Left Gripper",
            int(left_gripper),
            CFG.LEFT_GRIPPER_MIN,
            CFG.LEFT_GRIPPER_MAX,
        )

        right_gripper = self.clamp_with_warning(
            "Right Gripper",
            int(right_gripper),
            CFG.RIGHT_GRIPPER_MIN,
            CFG.RIGHT_GRIPPER_MAX,
        )

        print(
            "[IK] "
            f"x={x_mm:.1f}, y={y_mm:.1f}, r={r:.1f}mm | "
            f"base={base_deg:.1f}deg | "
            f"deg: shoulder={shoulder_deg:.2f}, elbow={elbow_deg:.2f}, wrist={wrist_deg:.2f} | "
            f"tick: {base}, {shoulder}, {elbow}, {wrist}, "
            f"{left_gripper}, {right_gripper}"
        )

        return ServoPose6(
            base=base,
            shoulder=shoulder,
            elbow=elbow,
            wrist=wrist,
            left_gripper=left_gripper,
            right_gripper=right_gripper,
        )

    def cup_level_lift_pose_keep_base(
        self,
        base_tick: int,
        gripper_closed: bool = True,
    ) -> ServoPose6:
        """
        컵을 잡은 상태에서 물이 쏟아지지 않도록 드는 자세.
        
        목표:
            ID2 = 512
            ID3 = 512
            ID4 = 820
            gripper = close 유지
        """
        if gripper_closed:
            left_gripper = CFG.LEFT_GRIPPER_CLOSE
            right_gripper = CFG.RIGHT_GRIPPER_CLOSE
        else:
            left_gripper = CFG.LEFT_GRIPPER_OPEN
            right_gripper = CFG.RIGHT_GRIPPER_OPEN

        shoulder = 512
        elbow = 512
        wrist = 820

        shoulder = self.clamp_with_warning(
            "Cup Level Shoulder",
            shoulder,
            CFG.SHOULDER_MIN,
            CFG.SHOULDER_MAX,
        )

        elbow = self.clamp_with_warning(
            "Cup Level Elbow",
            elbow,
            CFG.ELBOW_MIN,
            CFG.ELBOW_MAX,
        )

        wrist = self.clamp_with_warning(
            "Cup Level Wrist",
            wrist,
            CFG.WRIST_MIN,
            CFG.WRIST_MAX,
        )

        print(
            "[Cup Level Lift] "
            f"base={base_tick}, shoulder={shoulder}, elbow={elbow}, wrist={wrist}, "
            f"gripper=closed"
        )

        return ServoPose6(
            base=base_tick,
            shoulder=shoulder,
            elbow=elbow,
            wrist=wrist,
            left_gripper=left_gripper,
            right_gripper=right_gripper,
        )

    def make_level_lift_pose_keep_base_by_tick(
        self,
        base_tick: int,
        gripper_closed: bool = True,
    ) -> ServoPose6:
        """
        물컵을 든 상태에서 컵 자세를 유지하는 lift pose.

        기준:
            ID3 Elbow tick + ID4 Wrist tick = CUP_LEVEL_ELBOW_WRIST_SUM_TICK

        예:
            ID3 = 512
            ID4 = 820
            sum = 1332
        """
        if gripper_closed:
            left_gripper = CFG.LEFT_GRIPPER_CLOSE
            right_gripper = CFG.RIGHT_GRIPPER_CLOSE
        else:
            left_gripper = CFG.LEFT_GRIPPER_OPEN
            right_gripper = CFG.RIGHT_GRIPPER_OPEN

        elbow = int(CFG.CUP_LEVEL_LIFT_ELBOW_TICK)
        wrist = int(CFG.CUP_LEVEL_ELBOW_WRIST_SUM_TICK - elbow)

        elbow = self.clamp_with_warning(
            "Cup Level Elbow",
            elbow,
            CFG.ELBOW_MIN,
            CFG.ELBOW_MAX,
        )

        wrist = self.clamp_with_warning(
            "Cup Level Wrist",
            wrist,
            CFG.WRIST_MIN,
            CFG.WRIST_MAX,
        )

        print(
            "[Cup Level Lift Tick] "
            f"elbow={elbow}, wrist={wrist}, "
            f"sum={elbow + wrist}"
        )

        return ServoPose6(
            base=base_tick,
            shoulder=CFG.HOME_SHOULDER,
            elbow=elbow,
            wrist=wrist,
            left_gripper=left_gripper,
            right_gripper=right_gripper,
        )

    def lift_pose_keep_base(
        self,
        base_tick: int,
        gripper_closed: bool = True,
    ) -> ServoPose6:
        """
        Base 방향은 유지하고, 팔만 위로 들어올리는 자세.
        """
        if gripper_closed:
            left_gripper = CFG.LEFT_GRIPPER_CLOSE
            right_gripper = CFG.RIGHT_GRIPPER_CLOSE
        else:
            left_gripper = CFG.LEFT_GRIPPER_OPEN
            right_gripper = CFG.RIGHT_GRIPPER_OPEN

        return ServoPose6(
            base=base_tick,
            shoulder=CFG.HOME_SHOULDER,
            elbow=CFG.HOME_ELBOW,
            wrist=CFG.HOME_WRIST,
            left_gripper=left_gripper,
            right_gripper=right_gripper,
        )

    def pre_grasp_pose_from_target(
        self,
        x_mm: float,
        y_mm: float,
        approach_mm: float = None,
    ) -> ServoPose6:
        """
        컵을 잡기 전에, 최종 컵 위치보다 로봇 쪽으로 조금 안쪽에 있는 사전 접근 자세.

        현재는 config에서 PRE_GRASP_APPROACH_MM = 0.0 이므로
        target과 같은 반경 자세를 사용한다.
        """
        if approach_mm is None:
            approach_mm = CFG.PRE_GRASP_APPROACH_MM

        r_target = self.radius(x_mm, y_mm)
        base_deg = self.base_angle_deg(x_mm, y_mm)

        dx, dy = self.panel_to_robot(x_mm, y_mm)

        if r_target <= 1.0:
            r_pre = self.min_r
            pre_x = x_mm
            pre_y = y_mm
        else:
            r_pre = r_target - approach_mm
            if r_pre < self.min_r:
                r_pre = self.min_r

            scale = r_pre / r_target
            pre_dx = dx * scale
            pre_dy = dy * scale

            pre_x = self.robot_x + pre_dx
            pre_y = self.robot_y + pre_dy

        base = self.base_angle_to_tick(base_deg)
        base = self.clamp_with_warning("Base", base, CFG.BASE_MIN, CFG.BASE_MAX)

        shoulder_deg, elbow_deg, wrist_deg = self.interpolate_joint_angles_by_radius(r_pre)

        shoulder = self.angle_to_ax_tick_absolute(shoulder_deg)
        elbow = self.angle_to_ax_tick_absolute(elbow_deg)
        wrist = self.angle_to_ax_tick_absolute(wrist_deg)

        shoulder = self.clamp_with_warning("Shoulder", shoulder, CFG.SHOULDER_MIN, CFG.SHOULDER_MAX)
        elbow = self.clamp_with_warning("Elbow", elbow, CFG.ELBOW_MIN, CFG.ELBOW_MAX)
        wrist = self.clamp_with_warning("Wrist", wrist, CFG.WRIST_MIN, CFG.WRIST_MAX)

        print(
            "[PreGrasp] "
            f"target_r={r_target:.1f}mm -> pre_r={r_pre:.1f}mm | "
            f"target=({x_mm:.1f},{y_mm:.1f}), pre=({pre_x:.1f},{pre_y:.1f}) | "
            f"base={base_deg:.1f}deg | "
            f"deg: shoulder={shoulder_deg:.2f}, elbow={elbow_deg:.2f}, wrist={wrist_deg:.2f} | "
            f"tick: {base}, {shoulder}, {elbow}, {wrist}, "
            f"{CFG.LEFT_GRIPPER_OPEN}, {CFG.RIGHT_GRIPPER_OPEN}"
        )

        return ServoPose6(
            base=base,
            shoulder=shoulder,
            elbow=elbow,
            wrist=wrist,
            left_gripper=CFG.LEFT_GRIPPER_OPEN,
            right_gripper=CFG.RIGHT_GRIPPER_OPEN,
        )

    def retract_pose_from_current_place(
        self,
        x_mm: float,
        y_mm: float,
        retract_mm: float = None,
    ) -> ServoPose6:
        """
        컵을 내려놓은 뒤, 현재 place 위치에서 로봇 쪽으로 뒤로 빠지는 후퇴 자세.
        """
        if retract_mm is None:
            retract_mm = CFG.RETRACT_MM

        r_place = self.radius(x_mm, y_mm)
        base_deg = self.base_angle_deg(x_mm, y_mm)

        r_retract = r_place - retract_mm
        if r_retract < self.min_r:
            r_retract = self.min_r

        base = self.base_angle_to_tick(base_deg)
        base = self.clamp_with_warning("Base", base, CFG.BASE_MIN, CFG.BASE_MAX)

        shoulder_deg, elbow_deg, wrist_deg = self.interpolate_joint_angles_by_radius(r_retract)

        shoulder = self.angle_to_ax_tick_absolute(shoulder_deg)
        elbow = self.angle_to_ax_tick_absolute(elbow_deg)
        wrist = self.angle_to_ax_tick_absolute(wrist_deg)

        shoulder = self.clamp_with_warning("Shoulder", shoulder, CFG.SHOULDER_MIN, CFG.SHOULDER_MAX)
        elbow = self.clamp_with_warning("Elbow", elbow, CFG.ELBOW_MIN, CFG.ELBOW_MAX)
        wrist = self.clamp_with_warning("Wrist", wrist, CFG.WRIST_MIN, CFG.WRIST_MAX)

        print(
            "[Retract] "
            f"place_r={r_place:.1f}mm -> retract_r={r_retract:.1f}mm | "
            f"base={base_deg:.1f}deg | "
            f"deg: shoulder={shoulder_deg:.2f}, elbow={elbow_deg:.2f}, wrist={wrist_deg:.2f} | "
            f"tick: {base}, {shoulder}, {elbow}, {wrist}, "
            f"{CFG.LEFT_GRIPPER_OPEN}, {CFG.RIGHT_GRIPPER_OPEN}"
        )

        return ServoPose6(
            base=base,
            shoulder=shoulder,
            elbow=elbow,
            wrist=wrist,
            left_gripper=CFG.LEFT_GRIPPER_OPEN,
            right_gripper=CFG.RIGHT_GRIPPER_OPEN,
        )

    def vertical_lift_pose_keep_base(self, base_tick: int) -> ServoPose6:
        """
        컵을 내려놓고 뒤로 빠진 뒤,
        Base 방향은 유지한 채 팔만 위로 들어올리는 안전 자세.
        """
        return ServoPose6(
            base=base_tick,
            shoulder=CFG.HOME_SHOULDER,
            elbow=CFG.HOME_ELBOW,
            wrist=CFG.HOME_WRIST,
            left_gripper=CFG.LEFT_GRIPPER_OPEN,
            right_gripper=CFG.RIGHT_GRIPPER_OPEN,
        )

    def make_pose_from_joint_ticks(
        self,
        x_mm: float,
        y_mm: float,
        joint_ticks,
        left_gripper: int,
        right_gripper: int,
    ) -> ServoPose6:
        """
        Base는 목표 x,y 방향으로 계산하고,
        ID2/ID3/ID4는 직접 지정한 tick을 사용한다.

        가까운 컵 READY/SET 단계 전용.
        """
        base_deg = self.base_angle_deg(x_mm, y_mm)
        base = self.base_angle_to_tick(base_deg)

        shoulder, elbow, wrist = joint_ticks

        base = self.clamp_with_warning("Base", base, CFG.BASE_MIN, CFG.BASE_MAX)
        shoulder = self.clamp_with_warning("Shoulder", int(shoulder), CFG.SHOULDER_MIN, CFG.SHOULDER_MAX)
        elbow = self.clamp_with_warning("Elbow", int(elbow), CFG.ELBOW_MIN, CFG.ELBOW_MAX)
        wrist = self.clamp_with_warning("Wrist", int(wrist), CFG.WRIST_MIN, CFG.WRIST_MAX)

        left_gripper = self.clamp_with_warning(
            "Left Gripper",
            int(left_gripper),
            CFG.LEFT_GRIPPER_MIN,
            CFG.LEFT_GRIPPER_MAX,
        )

        right_gripper = self.clamp_with_warning(
            "Right Gripper",
            int(right_gripper),
            CFG.RIGHT_GRIPPER_MIN,
            CFG.RIGHT_GRIPPER_MAX,
        )

        print(
            "[Near Pose] "
            f"x={x_mm:.1f}, y={y_mm:.1f} | "
            f"base={base_deg:.1f}deg | "
            f"tick: {base}, {shoulder}, {elbow}, {wrist}, "
            f"{left_gripper}, {right_gripper}"
        )

        return ServoPose6(
            base=base,
            shoulder=shoulder,
            elbow=elbow,
            wrist=wrist,
            left_gripper=left_gripper,
            right_gripper=right_gripper,
        )

    def near_ready_pose(self, x_mm: float, y_mm: float) -> ServoPose6:
        return self.make_pose_from_joint_ticks(
            x_mm=x_mm,
            y_mm=y_mm,
            joint_ticks=CFG.NEAR_READY_JOINT_TICKS,
            left_gripper=CFG.LEFT_GRIPPER_OPEN,
            right_gripper=CFG.RIGHT_GRIPPER_OPEN,
        )

    def near_set_pose(self, x_mm: float, y_mm: float) -> ServoPose6:
        return self.make_pose_from_joint_ticks(
            x_mm=x_mm,
            y_mm=y_mm,
            joint_ticks=CFG.NEAR_SET_JOINT_TICKS,
            left_gripper=CFG.LEFT_GRIPPER_OPEN,
            right_gripper=CFG.RIGHT_GRIPPER_OPEN,
        )

    def home_pose(self) -> ServoPose6:
        return ServoPose6(
            base=CFG.HOME_BASE,
            shoulder=CFG.HOME_SHOULDER,
            elbow=CFG.HOME_ELBOW,
            wrist=CFG.HOME_WRIST,
            left_gripper=CFG.HOME_LEFT_GRIPPER,
            right_gripper=CFG.HOME_RIGHT_GRIPPER,
        )

    def validate_calibration(self):
        """
        시작할 때 각도 테이블과 리미트 확인.
        """
        print("[Calibration Check] AX-12A tick per degree =", round(self.tick_per_deg, 3))
        print(
            "[Calibration Check] Robot origin = "
            f"({CFG.ROBOT_ORIGIN_X_MM:.1f}, {CFG.ROBOT_ORIGIN_Y_MM:.1f}) mm"
        )
        print(
            "[Calibration Check] Radius = "
            f"min {CFG.MIN_RADIUS_MM:.1f}, mid {CFG.MID_RADIUS_MM:.1f}, max {CFG.MAX_RADIUS_MM:.1f} mm"
        )

        samples = [
            ("MIN", CFG.MIN_RADIUS_JOINT_DEG),
            ("MID", CFG.MID_RADIUS_JOINT_DEG),
            ("MAX", CFG.MAX_RADIUS_JOINT_DEG),
        ]

        for name, pose in samples:
            shoulder_deg, elbow_deg, wrist_deg = pose

            shoulder = self.angle_to_ax_tick_absolute(shoulder_deg)
            elbow = self.angle_to_ax_tick_absolute(elbow_deg)
            wrist = self.angle_to_ax_tick_absolute(wrist_deg)

            print(
                f"[Calibration Check] {name}: "
                f"deg=({shoulder_deg:.2f}, {elbow_deg:.2f}, {wrist_deg:.2f}) "
                f"tick=({shoulder}, {elbow}, {wrist})"
            )

            if not (CFG.SHOULDER_MIN <= shoulder <= CFG.SHOULDER_MAX):
                print(
                    f"[WARNING] {name} Shoulder tick {shoulder} "
                    f"is outside limit {CFG.SHOULDER_MIN}~{CFG.SHOULDER_MAX}"
                )

            if not (CFG.ELBOW_MIN <= elbow <= CFG.ELBOW_MAX):
                print(
                    f"[WARNING] {name} Elbow tick {elbow} "
                    f"is outside limit {CFG.ELBOW_MIN}~{CFG.ELBOW_MAX}"
                )

            if not (CFG.WRIST_MIN <= wrist <= CFG.WRIST_MAX):
                print(
                    f"[WARNING] {name} Wrist tick {wrist} "
                    f"is outside limit {CFG.WRIST_MIN}~{CFG.WRIST_MAX}"
                )