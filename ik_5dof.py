import math
from typing import Tuple
from config import CFG

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def deg_to_ax12(deg: float, center: int = CFG.AX_CENTER, invert: bool = False) -> int:
    # AX-12A는 일반적으로 0~1023이 약 300도 범위.
    # center를 150도 기준으로 보고 상대각을 encoder로 변환한다.
    sign = -1.0 if invert else 1.0
    enc = center + sign * (deg / CFG.AX_DEG_RANGE) * 1023.0
    return int(clamp(round(enc), CFG.AX_MIN, CFG.AX_MAX))

class IK5DOF:
    """
    5DOF 간단 IK.
    - Base: x,y 방향 회전
    - Shoulder/Elbow: 2링크 평면 IK
    - Wrist: 그리퍼 자세 보정
    - Gripper: open/close encoder
    실제 조립 방향에 따라 invert와 offset은 반드시 수정해야 한다.
    """

    def solve(self, x_mm: float, y_mm: float, z_mm: float, wrist_pitch_deg: float, gripper_encoder: int) -> Tuple[int, int, int, int, int]:
        base_deg = math.degrees(math.atan2(y_mm, x_mm))

        r = math.sqrt(x_mm * x_mm + y_mm * y_mm)
        z = z_mm
        L1 = CFG.LINK1_MM
        L2 = CFG.LINK2_MM

        d = math.sqrt(r * r + z * z)
        d = clamp(d, 1.0, L1 + L2 - 1.0)

        cos_elbow = clamp((d*d - L1*L1 - L2*L2) / (2.0 * L1 * L2), -1.0, 1.0)
        elbow_rad = math.acos(cos_elbow)

        shoulder_rad = math.atan2(z, r) - math.atan2(L2 * math.sin(elbow_rad), L1 + L2 * math.cos(elbow_rad))

        shoulder_deg = math.degrees(shoulder_rad)
        elbow_deg = math.degrees(elbow_rad)

        # wrist_pitch_deg: 그리퍼를 위로 들거나 90도 숙이는 명령값
        wrist_deg = wrist_pitch_deg - shoulder_deg - elbow_deg

        # 아래 encoder 변환은 임시값. 실제 로봇에서 축 방향 맞춰야 한다.
        base = deg_to_ax12(base_deg, center=512, invert=False)
        shoulder = deg_to_ax12(shoulder_deg, center=512, invert=True)
        elbow = deg_to_ax12(elbow_deg - 90.0, center=512, invert=False)
        wrist = deg_to_ax12(wrist_deg, center=512, invert=False)
        gripper = int(clamp(gripper_encoder, CFG.AX_MIN, CFG.AX_MAX))

        return base, shoulder, elbow, wrist, gripper
