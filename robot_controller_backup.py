import time
from config import CFG
from ik_5dof import IK5DOF
from serial_comm import SerialComm

class RobotController:
    def __init__(self):
        self.ik = IK5DOF()
        self.serial = SerialComm()

    def move_encoders(self, encoders, delay_s=0.7):
        self.serial.send_positions(encoders)
        time.sleep(delay_s)

    def home(self):
        self.move_encoders(CFG.HOME_POS, delay_s=1.0)

    def grasp_cup(self, x_mm: float, y_mm: float):
        """
        요청한 파지 흐름:
        1) 그리퍼를 위로 치켜든 상태로 컵 위 접근
        2) 해당 위치에서 그리퍼를 90도 숙임
        3) 하강 후 파지
        4) 상승
        """
        open_g = CFG.GRIPPER_OPEN
        close_g = CFG.GRIPPER_CLOSE

        # 1. 컵 위쪽 접근: gripper up
        approach_up = self.ik.solve(x_mm, y_mm, CFG.APPROACH_Z_MM, wrist_pitch_deg=0.0, gripper_encoder=open_g)
        self.move_encoders(approach_up, 1.0)

        # 2. 같은 xy 위치에서 그리퍼를 90도 숙임
        approach_down = self.ik.solve(x_mm, y_mm, CFG.APPROACH_Z_MM, wrist_pitch_deg=-90.0, gripper_encoder=open_g)
        self.move_encoders(approach_down, 0.8)

        # 3. 하강
        grasp_pose = self.ik.solve(x_mm, y_mm, CFG.GRASP_Z_MM, wrist_pitch_deg=-90.0, gripper_encoder=open_g)
        self.move_encoders(grasp_pose, 0.8)

        # 4. 그리퍼 닫기
        close_pose = self.ik.solve(x_mm, y_mm, CFG.GRASP_Z_MM, wrist_pitch_deg=-90.0, gripper_encoder=close_g)
        self.move_encoders(close_pose, 0.6)

        # 5. 상승
        lift_pose = self.ik.solve(x_mm, y_mm, CFG.LIFT_Z_MM, wrist_pitch_deg=-90.0, gripper_encoder=close_g)
        self.move_encoders(lift_pose, 1.0)

    def close(self):
        self.serial.close()
