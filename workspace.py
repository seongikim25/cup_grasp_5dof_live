from dataclasses import dataclass
import math

from config import CFG


@dataclass
class WorkspaceResult:
    ok: bool
    message: str
    r_mm: float
    theta_deg: float


def check_workspace(x_mm: float, y_mm: float) -> WorkspaceResult:
    """
    로봇 중심 기준 원형 작업 반경 검사.

    입력:
        x_mm, y_mm:
            패널 LT 원점 기준 cup bottom point 좌표(mm)

    패널 좌표계:
        LT = (0, 0)
        RT = (600, 0)
        RB = (600, 450)
        LB = (0, 450)

        x: 오른쪽 +
        y: 아래쪽 +

    현재 로봇 설치:
        로봇팔 = 패널 상단 중앙부
        로봇 정면 = 패널 아래쪽 방향

    로봇 기준:
        dx = 오른쪽 +
        dy = 로봇 전방 +
    """

    robot_x_mm = CFG.ROBOT_ORIGIN_X_MM
    robot_y_mm = CFG.ROBOT_ORIGIN_Y_MM

    dx = x_mm - robot_x_mm
    dy = y_mm - robot_y_mm

    r_mm = math.sqrt(dx * dx + dy * dy)

    # 로봇 정면을 0도로 봄
    # 오른쪽 +, 왼쪽 -
    theta_deg = math.degrees(math.atan2(dx, dy))

    if r_mm < CFG.MIN_RADIUS_MM:
        return WorkspaceResult(
            ok=False,
            message=f"WARNING: too close r={r_mm / 10.0:.1f}cm",
            r_mm=r_mm,
            theta_deg=theta_deg,
        )

    if r_mm > CFG.MAX_RADIUS_MM:
        return WorkspaceResult(
            ok=False,
            message=f"WARNING: too far r={r_mm / 10.0:.1f}cm",
            r_mm=r_mm,
            theta_deg=theta_deg,
        )

    return WorkspaceResult(
        ok=True,
        message=f"OK r={r_mm / 10.0:.1f}cm theta={theta_deg:.1f}deg",
        r_mm=r_mm,
        theta_deg=theta_deg,
    )