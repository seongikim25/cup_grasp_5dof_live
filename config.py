from dataclasses import dataclass
from typing import Tuple


@dataclass
class AppConfig:
    # True: 실제 AX-12A로 명령 전송 / False: 화면과 콘솔만 확인
    RUN_MODE: bool = False

    # U2D2
    SERIAL_PORT: str = "/dev/ttyUSB0"
    SERIAL_BAUD: int = 1000000

    # ============================================================
    # Camera / YOLO
    # ============================================================
    CAMERA_INDEX: int = 0
    CAMERA_FLIP_HORIZONTAL: bool = False

    YOLO_MODEL_PATH: str = "best.pt"
    CUP_CLASS_NAMES: Tuple[str, ...] = ("cup", "Cup", "컵")

    CONF_THRES: float = 0.45
    YOLO_CONF_THRESHOLD: float = 0.45
    YOLO_IMGSZ: int = 640

    STABLE_FRAME_COUNT: int = 8
    STABLE_PIXEL_TOLERANCE: float = 8.0
    STABLE_WINDOW: int = 8
    STABLE_PIXEL_THRESHOLD: float = 8.0

    # ============================================================
    # Panel / Homography
    #
    # LT = (0, 0)
    # RT = (600, 0)
    # RB = (600, 450)
    # LB = (0, 450)
    #
    # x: 오른쪽 +
    # y: 아래쪽 +
    # ============================================================
    PANEL_WIDTH_MM: float = 600.0
    PANEL_HEIGHT_MM: float = 450.0

    PANEL_THRESHOLD_V: int = 90
    PANEL_MIN_AREA_RATIO: float = 0.00003
    PANEL_MAX_AREA_RATIO: float = 0.20

    INPUT_IS_ROBOT_XY: bool = False

    IMAGE_POINTS: Tuple[Tuple[float, float], ...] = (
        (160, 120),
        (520, 120),
        (520, 420),
        (160, 420),
    )

    WORLD_POINTS_MM: Tuple[Tuple[float, float], ...] = (
        (-150, 250),
        (150, 250),
        (150, 50),
        (-150, 50),
    )

    # ============================================================
    # Measured Robot Origin / Workspace Radius
    #
    # 현재 하드웨어 배치:
    # 로봇팔 = 패널 상단 중앙부
    # 로봇 정면 = 패널 아래쪽 방향, 즉 y 증가 방향
    # ============================================================
    ROBOT_ORIGIN_X_MM: float = 300.0
    ROBOT_ORIGIN_Y_MM: float = 23.0

    MIN_RADIUS_MM: float = 145.0
    MAX_RADIUS_MM: float = 310.0
    MID_RADIUS_MM: float = 227.5

    BOTTOM_OFFSET_X_MM: float = 0.0
    BOTTOM_OFFSET_Y_MM: float = 0.0

    # ============================================================
    # Measured Manipulator Geometry
    # ============================================================
    SHOULDER_HEIGHT_MM: float = 70.0

    LINK1_MM: float = 83.0
    LINK2_MM: float = 99.0
    TOOL_OFFSET_MM: float = 47.0

    APPROACH_Z_MM: float = 70.0
    GRASP_Z_MM: float = 35.0
    LIFT_Z_MM: float = 100.0
    PLACE_Z_MM: float = 35.0

    # ============================================================
    # Empirical Joint Angle Table
    # Wizard에 표시되는 실제 절대각 기준
    #
    # 순서: ID2 Shoulder, ID3 Elbow, ID4 Wrist
    #
    # MIN 반경:
    #   ID2 = 148.83 deg
    #   ID3 = 296.00 deg
    #   ID4 = 87.60 deg
    #
    # MAX 반경:
    #   ID2 원래 65 deg지만 현재 리미트상 69.14 deg 사용
    #   ID3 = 151.00 deg
    #   ID4 원래 154 deg지만 현재 리미트상 150.00 deg 사용
    #
    # ID2: 각도 증가 -> 팔이 위로 올라감
    # ID3: 각도 증가 -> 팔이 내려감
    # ID4: 각도 증가 -> 손목/그리퍼가 내려감
    # ============================================================
    MIN_RADIUS_JOINT_DEG: Tuple[float, float, float] = (148.83, 288.0, 90.0)
    MID_RADIUS_JOINT_DEG: Tuple[float, float, float] = (111.04, 241.41, 102.54)
    MAX_RADIUS_JOINT_DEG: Tuple[float, float, float] = (69.14, 151.0, 150.0)

    # 컵 접근 전 pre-grasp 위치를 목표보다 안쪽으로 얼마나 당길지
    # 동작이 불안정하면 0.0으로 두는 것이 안전
    PRE_GRASP_APPROACH_MM: float = 6.0

    # 컵을 내려놓은 뒤 후퇴 거리
    RETRACT_MM: float = 70.0

    # ============================================================
    # Near Cup Special Grasp Sequence
    # r < MID_RADIUS_MM 일 때만 사용하는 가까운 컵 전용 자세
    #
    # 순서:
    #   ID2 Shoulder tick, ID3 Elbow tick, ID4 Wrist tick
    #
    # 값은 임시 안전값.
    # 네가 직접 측정한 ready/set tick으로 나중에 교체하면 됨.
    # ============================================================
    NEAR_SPLIT_RADIUS_MM: float = (MIN_RADIUS_MM + MID_RADIUS_MM) / 2.0

    NEAR_READY_JOINT_TICKS: Tuple[int, int, int] = ( # 각도를 tick으로 변환할시 3.41을 곱해주면 된다.
        819, 1012, 511
        # 240.23, 296.19, 150
    )

    NEAR_SET_JOINT_TICKS: Tuple[int, int, int] = ( # 각도를 tick으로 변환할시 3.41을 곱해주면 된다. 
        501, 1012, 298
        # 147.07, 296.19, 87.60
    )

    # ============================================================
    # Multi-point radius joint table
    # 중간 구간이 선형으로 안 맞을 때 사용하는 다점 보간 테이블
    # 순서: radius_mm, ID2_deg, ID3_deg, ID4_deg
    # ============================================================
    USE_MULTI_RADIUS_TABLE: bool = True

    RADIUS_JOINT_TABLE = (
    (145.0, 148.83, 288.00,  90.00),   # MIN
    (186.25, 130.00, 255.00, 105.00),
    (227.5, 111.04, 241.41, 102.54),   # MID
    (239.0, 104.59, 215.00, 105.00),   # 문제 구간 보정점
    (268.75, 88.00, 185.00, 135.00),
    (310.0, 69.14, 151.00, 150.00),    # MAX
)

    # ============================================================
    # AX-12A
    # ============================================================
    AX_CENTER: int = 512
    AX_MIN: int = 0
    AX_MAX: int = 1023
    AX_DEG_RANGE: float = 300.0
    AX_TICK_RANGE: float = 1023.0

    MOTOR_NAMES: Tuple[str, ...] = (
        "Base",
        "Shoulder",
        "Elbow",
        "Wrist",
        "Left_Gripper",
        "Right_Gripper",
    )

    # ============================================================
    # Servo Zero / Direction
    # Base만 상대각 방식 사용
    # ID2/ID3/ID4는 Wizard 절대각 -> tick 변환 사용
    # ============================================================
    BASE_ZERO: int = 512

    # ID1 Base가 컵 반대 방향으로 돌면 BASE_DIR만 -1로 변경
    BASE_DIR: int = 1

    # 아래 값들은 호환용으로 남김
    SHOULDER_ZERO: int = 512
    ELBOW_ZERO: int = 512
    WRIST_ZERO: int = 512

    SHOULDER_DIR: int = -1
    ELBOW_DIR: int = 1
    WRIST_DIR: int = -1

    # ============================================================
    # Wizard에서 직접 설정한 절대 리미트
    # 현재 리미트 스크린샷 기준
    # ============================================================
    BASE_MIN: int = 204
    BASE_MAX: int = 820

    SHOULDER_MIN: int = 236
    SHOULDER_MAX: int = 820

    ELBOW_MIN: int = 512
    ELBOW_MAX: int = 1013

    WRIST_MIN: int = 298
    WRIST_MAX: int = 820

    LEFT_GRIPPER_MIN: int = 371
    LEFT_GRIPPER_MAX: int = 608

    RIGHT_GRIPPER_MIN: int = 415
    RIGHT_GRIPPER_MAX: int = 697

    # ============================================================
    # Gripper
    #
    # 오른쪽 그리퍼가 닫히지 않고 열리면
    # RIGHT_GRIPPER_CLOSE를 697로 바꿔서 테스트
    # ============================================================
    LEFT_GRIPPER_OPEN: int = 371
    LEFT_GRIPPER_CLOSE: int = 608

    RIGHT_GRIPPER_OPEN: int = 697
    RIGHT_GRIPPER_CLOSE: int = 415

    # ============================================================
    # Cup Level Constraint - tick sum version
    # 물컵을 들 때 ID3 + ID4 tick 합을 일정하게 유지
    # 기준 자세: ID3=512, ID4=820 → sum=1332
    # ============================================================
    KEEP_CUP_LEVEL: bool = True

    CUP_LEVEL_ELBOW_WRIST_SUM_TICK: int = 1332

    # lift 중 사용할 ID3 기준값
    # ID4는 1332 - ID3 으로 자동 계산
    CUP_LEVEL_LIFT_ELBOW_TICK: int = 512

    # ============================================================
    # Home Pose
    # ============================================================
    HOME_BASE: int = 512
    HOME_SHOULDER: int = 512
    HOME_ELBOW: int = 512
    HOME_WRIST: int = 512
    HOME_LEFT_GRIPPER: int = LEFT_GRIPPER_OPEN
    HOME_RIGHT_GRIPPER: int = RIGHT_GRIPPER_OPEN

    HOME_POS: Tuple[int, ...] = (
        HOME_BASE,
        HOME_SHOULDER,
        HOME_ELBOW,
        HOME_WRIST,
        HOME_LEFT_GRIPPER,
        HOME_RIGHT_GRIPPER,
    )

    # ============================================================
    # Motion Timing
    # 안전 테스트용 저속 설정
    # ============================================================
    MOVE_TIME_SEC: float = 2.5
    GRIPPER_TIME_SEC: float = 1.5

    MOVE_DELAY_SEC: float = 1.0
    GRIPPER_DELAY_SEC: float = 1.2

    # AX-12A Moving Speed 1 unit ≈ 1.89 tick/s 근사
    AX_SPEED_UNIT_TICK_PER_SEC: float = 1.89

    # 순서: ID1, ID2, ID3, ID4, ID5, ID6
    MOTOR_MIN_MOVING_SPEEDS: Tuple[int, ...] = (30, 35, 35, 30, 30, 30)

    MIN_MOVING_SPEED: int = 40
    MAX_MOVING_SPEED: int = 120

    # ============================================================
    # Emergency Stop
    # ============================================================
    ALLOW_HOME_AFTER_ESTOP: bool = True


CFG = AppConfig()
