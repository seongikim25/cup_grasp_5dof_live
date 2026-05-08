import time
from typing import Optional, List

from dynamixel_sdk import PortHandler, PacketHandler

from config import CFG
from manipulator_kinematics import EmpiricalManipulatorKinematics, ServoPose6


class RobotController:
    """
    U2D2 + AX-12A 직접 제어용 컨트롤러.

    AX-12A:
        Protocol 1.0
        Baudrate: 1000000

    모터 ID:
        1: Base
        2: Shoulder
        3: Elbow
        4: Wrist
        5: Left Gripper
        6: Right Gripper
    """

    ADDR_TORQUE_ENABLE = 24
    ADDR_GOAL_POSITION = 30
    ADDR_MOVING_SPEED = 32

    PROTOCOL_VERSION = 1.0

    DXL_IDS = [1, 2, 3, 4, 5, 6]

    def __init__(self):
        self.ik = EmpiricalManipulatorKinematics()
        self.current_pose: Optional[ServoPose6] = None
        self.estopped = False

        self.port_handler = None
        self.packet_handler = None
        self.connected = False

        print("[Robot] U2D2 + AX-12A controller initialized")

        if CFG.RUN_MODE:
            self._connect_u2d2()
            self._torque_on_all()
            print("[Robot] RUN mode: U2D2 hardware command enabled")
        else:
            print("[Robot] DEV mode: no hardware command will be sent")

        self.home()

    def _connect_u2d2(self):
        self.port_handler = PortHandler(CFG.SERIAL_PORT)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            print(f"[Robot] failed to open port: {CFG.SERIAL_PORT}")
            self.connected = False
            return

        if not self.port_handler.setBaudRate(CFG.SERIAL_BAUD):
            print(f"[Robot] failed to set baudrate: {CFG.SERIAL_BAUD}")
            self.connected = False
            return

        self.connected = True
        print(f"[Robot] connected: {CFG.SERIAL_PORT}, baud={CFG.SERIAL_BAUD}")

    def pose_values(self, pose: ServoPose6) -> List[int]:
        return [
            pose.base,
            pose.shoulder,
            pose.elbow,
            pose.wrist,
            pose.left_gripper,
            pose.right_gripper,
        ]

    def compute_synchronized_speeds(
        self,
        current_pose: ServoPose6,
        target_pose: ServoPose6,
        move_time_sec: float,
    ) -> List[int]:
        """
        각 모터가 같은 시간에 목표 위치에 도착하도록 Moving Speed 계산.
        """
        current_values = self.pose_values(current_pose)
        target_values = self.pose_values(target_pose)

        speeds = []

        motor_min_speeds = getattr(
            CFG,
            "MOTOR_MIN_MOVING_SPEEDS",
            (CFG.MIN_MOVING_SPEED,) * len(self.DXL_IDS),
        )

        for i, (cur, tgt) in enumerate(zip(current_values, target_values)):
            delta = abs(tgt - cur)

            if i < len(motor_min_speeds):
                min_speed = int(motor_min_speeds[i])
            else:
                min_speed = int(CFG.MIN_MOVING_SPEED)

            if delta == 0:
                speed = min_speed
            else:
                speed = int(round(delta / (move_time_sec * CFG.AX_SPEED_UNIT_TICK_PER_SEC)))

            speed = max(min_speed, speed)
            speed = min(int(CFG.MAX_MOVING_SPEED), speed)

            speeds.append(speed)

        return speeds

    def _write_word(self, dxl_id: int, address: int, value: int):
        if not CFG.RUN_MODE or not self.connected:
            return True

        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler,
            dxl_id,
            address,
            int(value),
        )

        if dxl_comm_result != 0:
            print(
                f"[DXL ERROR] ID {dxl_id}: "
                f"{self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )
            return False

        if dxl_error != 0:
            print(
                f"[DXL ERROR] ID {dxl_id}: "
                f"{self.packet_handler.getRxPacketError(dxl_error)}"
            )
            return False

        return True

    def _write_byte(self, dxl_id: int, address: int, value: int):
        if not CFG.RUN_MODE or not self.connected:
            return True

        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            dxl_id,
            address,
            int(value),
        )

        if dxl_comm_result != 0:
            print(
                f"[DXL ERROR] ID {dxl_id}: "
                f"{self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )
            return False

        if dxl_error != 0:
            print(
                f"[DXL ERROR] ID {dxl_id}: "
                f"{self.packet_handler.getRxPacketError(dxl_error)}"
            )
            return False

        return True

    def _torque_on_all(self):
        print("[Robot] torque ON all")

        for dxl_id in self.DXL_IDS:
            ok = self._write_byte(dxl_id, self.ADDR_TORQUE_ENABLE, 1)
            if ok:
                print(f"[Robot] torque ON OK: ID {dxl_id}")
            else:
                print(f"[Robot] torque ON failed: ID {dxl_id}")

    def _torque_off_all(self):
        print("[Robot] torque OFF all")

        for dxl_id in self.DXL_IDS:
            ok = self._write_byte(dxl_id, self.ADDR_TORQUE_ENABLE, 0)
            if ok:
                print(f"[Robot] torque OFF OK: ID {dxl_id}")
            else:
                print(f"[Robot] torque OFF failed: ID {dxl_id}")

    def _write_speeds(self, speeds: List[int]):
        ok_all = True

        for dxl_id, speed in zip(self.DXL_IDS, speeds):
            ok = self._write_word(
                dxl_id,
                self.ADDR_MOVING_SPEED,
                int(speed),
            )

            if not ok:
                print(f"[Robot] speed write failed: ID {dxl_id}, speed={speed}")
                ok_all = False
            else:
                print(f"[Robot] speed write OK: ID {dxl_id}, speed={speed}")

        return ok_all

    def _write_positions(self, pose: ServoPose6):
        values = self.pose_values(pose)
        ok_all = True

        for dxl_id, pos in zip(self.DXL_IDS, values):
            ok = self._write_word(
                dxl_id,
                self.ADDR_GOAL_POSITION,
                int(pos),
            )

            if not ok:
                print(f"[Robot] position write failed: ID {dxl_id}, pos={pos}")
                ok_all = False
            else:
                print(f"[Robot] position write OK: ID {dxl_id}, pos={pos}")

        return ok_all

    def _send_servo_pose_with_speed(self, pose: ServoPose6, speeds: List[int]):
        values = self.pose_values(pose)

        if CFG.RUN_MODE and self.connected:
            print("[Robot TX START]")

            speed_ok = self._write_speeds(speeds)

            time.sleep(0.05)

            pos_ok = self._write_positions(pose)

            print(
                "[Robot TX] "
                f"pos={values}, speed={speeds}, "
                f"speed_ok={speed_ok}, pos_ok={pos_ok}"
            )

        else:
            print(
                "[Robot SIM] "
                f"pos={values}, speed={speeds}"
            )

        self.current_pose = pose

    def move_pose(
        self,
        pose: ServoPose6,
        delay: Optional[float] = None,
        move_time_sec: Optional[float] = None,
    ):
        if self.estopped:
            print("[Robot] move rejected: emergency stop active")
            return False

        if pose is None:
            print("[Robot] move_pose failed: pose is None")
            return False

        if self.current_pose is None:
            self.current_pose = self.ik.home_pose()

        if move_time_sec is None:
            move_time_sec = CFG.MOVE_TIME_SEC

        speeds = self.compute_synchronized_speeds(
            current_pose=self.current_pose,
            target_pose=pose,
            move_time_sec=move_time_sec,
        )

        self._send_servo_pose_with_speed(pose, speeds)

        if delay is None:
            delay = move_time_sec

        time.sleep(delay)
        return True

    def home(self):
        if self.estopped:
            print("[Robot] home rejected: emergency stop active")
            return False

        print("[Robot] moving home")
        pose = self.ik.home_pose()
        return self.move_pose(
            pose,
            delay=CFG.MOVE_TIME_SEC,
            move_time_sec=CFG.MOVE_TIME_SEC,
        )

    def open_gripper(self):
        if self.estopped:
            print("[Robot] open gripper rejected: emergency stop active")
            return False

        print("[Robot] gripper open")

        if self.current_pose is None:
            base_pose = self.ik.home_pose()
        else:
            base_pose = self.current_pose

        pose = ServoPose6(
            base=base_pose.base,
            shoulder=base_pose.shoulder,
            elbow=base_pose.elbow,
            wrist=base_pose.wrist,
            left_gripper=CFG.LEFT_GRIPPER_OPEN,
            right_gripper=CFG.RIGHT_GRIPPER_OPEN,
        )

        return self.move_pose(
            pose,
            delay=CFG.GRIPPER_TIME_SEC,
            move_time_sec=CFG.GRIPPER_TIME_SEC,
        )

    def close_gripper(self):
        if self.estopped:
            print("[Robot] close gripper rejected: emergency stop active")
            return False

        print("[Robot] gripper close")

        if self.current_pose is None:
            base_pose = self.ik.home_pose()
        else:
            base_pose = self.current_pose

        pose = ServoPose6(
            base=base_pose.base,
            shoulder=base_pose.shoulder,
            elbow=base_pose.elbow,
            wrist=base_pose.wrist,
            left_gripper=CFG.LEFT_GRIPPER_CLOSE,
            right_gripper=CFG.RIGHT_GRIPPER_CLOSE,
        )

        return self.move_pose(
            pose,
            delay=CFG.GRIPPER_TIME_SEC,
            move_time_sec=CFG.GRIPPER_TIME_SEC,
        )

    def align_base_to_xy_holding(self, x_mm: float, y_mm: float):
        """
        컵을 잡은 상태에서 Base만 place 방향으로 회전.
        팔은 들어올린 자세 유지.
        """
        if self.estopped:
            print("[Robot] base align holding rejected: emergency stop active")
            return False

        if self.current_pose is None:
            self.current_pose = self.ik.home_pose()

        target_pose = self.ik.make_servo_pose(
            x_mm=x_mm,
            y_mm=y_mm,
            left_gripper=CFG.LEFT_GRIPPER_CLOSE,
            right_gripper=CFG.RIGHT_GRIPPER_CLOSE,
        )

        align_pose = ServoPose6(
            base=target_pose.base,
            shoulder=self.current_pose.shoulder,
            elbow=self.current_pose.elbow,
            wrist=self.current_pose.wrist,
            left_gripper=CFG.LEFT_GRIPPER_CLOSE,
            right_gripper=CFG.RIGHT_GRIPPER_CLOSE,
        )

        print("[Robot] base align while holding cup")

        return self.move_pose(
            align_pose,
            delay=2.0,
            move_time_sec=2.0,
        )

    def align_base_to_xy(self, x_mm: float, y_mm: float):
        """
        물체를 잡기 전에 Base만 먼저 목표 방향으로 회전.
        """
        if self.estopped:
            print("[Robot] base align rejected: emergency stop active")
            return False

        if self.current_pose is None:
            self.current_pose = self.ik.home_pose()

        target_pose = self.ik.make_servo_pose(
            x_mm=x_mm,
            y_mm=y_mm,
            left_gripper=CFG.LEFT_GRIPPER_OPEN,
            right_gripper=CFG.RIGHT_GRIPPER_OPEN,
        )

        align_pose = ServoPose6(
            base=target_pose.base,
            shoulder=self.current_pose.shoulder,
            elbow=self.current_pose.elbow,
            wrist=self.current_pose.wrist,
            left_gripper=CFG.LEFT_GRIPPER_OPEN,
            right_gripper=CFG.RIGHT_GRIPPER_OPEN,
        )

        print("[Robot] base align before approach")

        return self.move_pose(
            align_pose,
            delay=2.0,
            move_time_sec=2.0,
        )

    def move_to_xy(self, x_mm: float, y_mm: float, gripper_closed: bool = False):
        if self.estopped:
            print("[Robot] move_to_xy rejected: emergency stop active")
            return False

        left = CFG.LEFT_GRIPPER_CLOSE if gripper_closed else CFG.LEFT_GRIPPER_OPEN
        right = CFG.RIGHT_GRIPPER_CLOSE if gripper_closed else CFG.RIGHT_GRIPPER_OPEN

        pose = self.ik.make_servo_pose(
            x_mm=x_mm,
            y_mm=y_mm,
            left_gripper=left,
            right_gripper=right,
        )

        return self.move_pose(
            pose,
            delay=CFG.MOVE_TIME_SEC,
            move_time_sec=CFG.MOVE_TIME_SEC,
        )

    def grasp_cup(self, x_mm: float, y_mm: float):
        if self.estopped:
            print("[Robot] grasp rejected: emergency stop active")
            return False

        ok, msg = self.ik.check_reachable(x_mm, y_mm)

        if not ok:
            print(f"[Robot] {msg}")
            return False

        r_mm = self.ik.radius(x_mm, y_mm)

        print(
            f"[Robot] grasp sequence start: "
            f"x={x_mm:.1f}, y={y_mm:.1f}, r={r_mm:.1f}mm"
        )

        if r_mm < CFG.MID_RADIUS_MM:
            return self.grasp_near_cup(x_mm, y_mm, r_mm)

        return self.grasp_normal_cup(x_mm, y_mm)

        if not self.move_pose(
            pre_grasp_pose,
            delay=CFG.MOVE_TIME_SEC,
            move_time_sec=CFG.MOVE_TIME_SEC,
        ):
            return False

        # 4. pre-grasp 자세에서 컵 쪽으로 접근
        print("[Robot] approach cup from pre-grasp pose")

        if not self.move_to_xy(
            x_mm,
            y_mm,
            gripper_closed=False,
        ):
            return False

        # 5. 그리퍼 닫기
        if not self.close_gripper():
            return False

        # 6. 컵을 잡은 상태로 들어올리기
        if self.current_pose is None:
            print("[Robot] lift after grasp failed: current pose is None")
            return False

        if CFG.KEEP_CUP_LEVEL:
            lift_pose = self.ik.make_level_lift_pose_keep_base_by_tick(
                base_tick=self.current_pose.base,
                gripper_closed=True,
            )
        else:
            lift_pose = self.ik.lift_pose_keep_base(
                base_tick=self.current_pose.base,
                gripper_closed=True,
            )

        print("[Robot] lift cup after grasp")

        if not self.move_pose(
            lift_pose,
            delay=CFG.MOVE_TIME_SEC,
            move_time_sec=CFG.MOVE_TIME_SEC,
        ):
            return False

        print("[Robot] grasp sequence complete")
        return True

    def grasp_normal_cup(self, x_mm: float, y_mm: float):
        """
        MID_RADIUS_MM 이상에서 사용하는 기존 보간 기반 기본 파지.
        기존 보간식은 그대로 유지한다.
        """
        print("[Robot] normal grasp sequence")

        # 1. 그리퍼 열기
        if not self.open_gripper():
            return False

        # 2. Base만 먼저 컵 방향으로 정렬
        if not self.align_base_to_xy(x_mm, y_mm):
            return False

        # 3. pre-grasp 자세
        pre_grasp_pose = self.ik.pre_grasp_pose_from_target(
            x_mm=x_mm,
            y_mm=y_mm,
            approach_mm=CFG.PRE_GRASP_APPROACH_MM,
        )

        print("[Robot] move to pre-grasp pose")

        if not self.move_pose(
            pre_grasp_pose,
            delay=CFG.MOVE_TIME_SEC,
            move_time_sec=CFG.MOVE_TIME_SEC,
        ):
            return False

        # 4. 목표 위치 접근
        print("[Robot] approach cup from pre-grasp pose")

        if not self.move_to_xy(
            x_mm,
            y_mm,
            gripper_closed=False,
        ):
            return False

        # 5. 그리퍼 닫기
        if not self.close_gripper():
            return False

        # 6. 들어올리기
        if self.current_pose is None:
            print("[Robot] lift after grasp failed: current pose is None")
            return False

        lift_pose = ServoPose6(
            base=self.current_pose.base,
            shoulder=512,
            elbow=512,
            wrist=820,
            left_gripper=CFG.LEFT_GRIPPER_CLOSE,
            right_gripper=CFG.RIGHT_GRIPPER_CLOSE,
        )

        print("[Robot] lift cup after grasp")

        if not self.move_pose(
            lift_pose,
            delay=CFG.MOVE_TIME_SEC,
            move_time_sec=CFG.MOVE_TIME_SEC,
        ):
            return False

        print("[Robot] normal grasp sequence complete")
        return True

    def grasp_near_cup(self, x_mm: float, y_mm: float, r_mm: float):
        """
        MID_RADIUS_MM보다 가까운 컵 전용 파지 루틴.

        MIN 쪽 가까운 경우:
            open → base align → READY → SET → gripper close → lift

        MID 쪽 가까운 경우:
            open → base align → READY → SET → 기존 보간 target pose → gripper close → lift

        기존 보간식은 침범하지 않고, 가까운 컵에서만 예외 동작을 수행한다.
        """
        print("[Robot] near grasp sequence")

        near_split_r = getattr(
            CFG,
            "NEAR_SPLIT_RADIUS_MM",
            (CFG.MIN_RADIUS_MM + CFG.MID_RADIUS_MM) / 2.0,
        )

        is_min_side = r_mm < near_split_r

        if is_min_side:
            print(
                f"[Near Grasp] r={r_mm:.1f}mm < split={near_split_r:.1f}mm "
                f"-> MIN-side: READY -> SET -> close"
            )
        else:
            print(
                f"[Near Grasp] r={r_mm:.1f}mm >= split={near_split_r:.1f}mm "
                f"-> MID-side: READY -> SET -> target -> close"
            )

        # 1. 그리퍼 열기
        if not self.open_gripper():
            return False

        # 2. Base만 먼저 컵 방향 정렬
        if not self.align_base_to_xy(x_mm, y_mm):
            return False

        # 3. READY 자세
        ready_pose = self.ik.near_ready_pose(x_mm, y_mm)

        print("[Robot] near READY pose")

        if not self.move_pose(
            ready_pose,
            delay=CFG.MOVE_TIME_SEC,
            move_time_sec=CFG.MOVE_TIME_SEC,
        ):
            return False

        # 4. SET 자세
        set_pose = self.ik.near_set_pose(x_mm, y_mm)

        print("[Robot] near SET pose")

        if not self.move_pose(
            set_pose,
            delay=CFG.MOVE_TIME_SEC,
            move_time_sec=CFG.MOVE_TIME_SEC,
        ):
            return False

        # 5. MIN 쪽이면 SET 위치에서 바로 그리퍼만 닫기
        if is_min_side:
            print("[Robot] MIN-side near grasp: close gripper at SET pose")

            if not self.close_gripper():
                return False

        # 6. MID 쪽이면 기존 보간 target pose로 한 번 더 전진 후 닫기
        else:
            print("[Robot] MID-side near grasp: move to interpolated target pose")

            if not self.move_to_xy(
                x_mm,
                y_mm,
                gripper_closed=False,
            ):
                return False

            if not self.close_gripper():
                return False

        # 7. 들어올리기
        if self.current_pose is None:
            print("[Robot] near lift failed: current pose is None")
            return False

        lift_pose = ServoPose6(
            base=self.current_pose.base,
            shoulder=512,
            elbow=512,
            wrist=820,
            left_gripper=CFG.LEFT_GRIPPER_CLOSE,
            right_gripper=CFG.RIGHT_GRIPPER_CLOSE,
        )

        print("[Robot] lift cup after near grasp")

        if not self.move_pose(
            lift_pose,
            delay=CFG.MOVE_TIME_SEC,
            move_time_sec=CFG.MOVE_TIME_SEC,
        ):
            return False

        print("[Robot] near grasp sequence complete")
        return True

    def place_cup(self, x_mm: float, y_mm: float):
        if self.estopped:
            print("[Robot] place rejected: emergency stop active")
            return False

        ok, msg = self.ik.check_reachable(x_mm, y_mm)

        if not ok:
            print(f"[Robot] {msg}")
            return False

        print(f"[Robot] place sequence start: x={x_mm:.1f}, y={y_mm:.1f}")

        # 1. 컵을 든 상태에서 Base만 place 방향으로 먼저 회전
        if not self.align_base_to_xy_holding(x_mm, y_mm):
            return False

        # 2. Base 정렬 후 팔 관절이 내려감
        print("[Robot] lower cup after base align")

        if not self.move_to_xy(x_mm, y_mm, gripper_closed=True):
            return False

        # 3. 컵 내려놓기: 그리퍼 열기
        if not self.open_gripper():
            return False

        # 4. 컵을 건드리지 않도록 READY 자세로 먼저 후퇴
        #    기존 retract_pose는 근거리에서 컵을 칠 수 있으므로 사용하지 않음
        ready_pose = self.ik.near_ready_pose(x_mm, y_mm)

        print("[Robot] move to READY pose after release")

        if not self.move_pose(
            ready_pose,
            delay=CFG.MOVE_TIME_SEC,
            move_time_sec=CFG.MOVE_TIME_SEC,
        ):
            return False

        # 5. READY 자세까지 빠진 뒤 home 복귀
        if not self.home():
            return False

        print("[Robot] place sequence complete")
        return True

    def emergency_stop(self):
        print("[EMERGENCY STOP] torque off all motors")
        self.estopped = True

        if CFG.RUN_MODE and self.connected:
            self._torque_off_all()
        else:
            print("[Robot SIM] torque OFF all")

    def clear_emergency_stop(self):
        print("[EMERGENCY STOP] cleared")
        self.estopped = False

        if CFG.RUN_MODE and self.connected:
            self._torque_on_all()
        else:
            print("[Robot SIM] torque ON all")

    def close(self):
        if CFG.RUN_MODE and self.connected:
            try:
                self._torque_off_all()
                self.port_handler.closePort()
                print("[Robot] U2D2 port closed")
            except Exception:
                pass

        print("[Robot] controller closed")