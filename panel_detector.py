import cv2
import numpy as np
from typing import Optional, Tuple

from config import CFG


class PanelDetector:
    """
    수동 4점 클릭 기반 패널 캘리브레이션.
    LT, RT, RB, LB 순서로 자동 정렬한 뒤 homography를 계산한다.

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
    """

    def __init__(
        self,
        panel_width_mm: float,
        panel_height_mm: float,
        threshold_v=None,
        min_area_ratio=None,
        max_area_ratio=None,
        debug: bool = False,
    ):
        self.panel_width_mm = float(panel_width_mm)
        self.panel_height_mm = float(panel_height_mm)
        self.debug = debug

        self.clicked_points = []
        self.ordered_points = None
        self.H = None

    def reset(self):
        self.clicked_points = []
        self.ordered_points = None
        self.H = None
        print("[Panel] calibration reset")

    def add_corner_point(self, x: int, y: int):
        """
        마우스로 클릭한 패널 코너 픽셀 좌표 추가.
        4개가 모이면 자동으로 homography 계산.
        """
        if len(self.clicked_points) >= 4:
            print("[Panel] already calibrated. Press R to reset.")
            return

        self.clicked_points.append((int(x), int(y)))
        print(f"[Panel] corner {len(self.clicked_points)}/4 = ({x}, {y})")

        if len(self.clicked_points) == 4:
            self._compute_homography()

    def _order_points(self, pts: np.ndarray) -> np.ndarray:
        """
        4개의 점을 LT, RT, RB, LB 순서로 정렬.
        """
        rect = np.zeros((4, 2), dtype=np.float32)

        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]      # LT
        rect[2] = pts[np.argmax(s)]      # RB

        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]   # RT
        rect[3] = pts[np.argmax(diff)]   # LB

        return rect

    def _compute_homography(self):
        pts_src = np.array(self.clicked_points, dtype=np.float32)
        pts_src = self._order_points(pts_src)

        pts_dst = np.float32([
            [0.0, 0.0],
            [self.panel_width_mm, 0.0],
            [self.panel_width_mm, self.panel_height_mm],
            [0.0, self.panel_height_mm],
        ])

        self.H = cv2.getPerspectiveTransform(pts_src, pts_dst)
        self.ordered_points = pts_src

        print("[Panel] manual calibration complete")
        print(
            f"[Panel] LT={pts_src[0]}, RT={pts_src[1]}, "
            f"RB={pts_src[2]}, LB={pts_src[3]}"
        )

    def is_calibrated(self) -> bool:
        return self.H is not None

    def detect_panel(self, frame):
        """
        기존 main.py 호환용 함수.
        빨간색 검출은 하지 않고, 현재 수동 캘리브레이션 상태만 반환한다.
        """
        panel_ok = self.is_calibrated()
        panel_box = self.ordered_points
        panel_mask = None
        return panel_ok, panel_box, panel_mask

    def pixel_to_world(self, u: float, v: float) -> Optional[Tuple[float, float]]:
        """
        픽셀 좌표를 패널 기준 mm 좌표로 변환.
        LT = (0, 0)
        RT = (600, 0)
        RB = (600, 450)
        LB = (0, 450)
        """
        if self.H is None:
            return None

        src = np.array([[[u, v]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(src, self.H)

        x_mm = float(dst[0, 0, 0])
        y_mm = float(dst[0, 0, 1])

        return x_mm, y_mm

    def pixel_to_cm(self, u: float, v: float) -> Optional[Tuple[float, float]]:
        """
        픽셀 좌표를 패널 기준 cm 좌표로 변환.
        homography는 mm 기준이므로 0.1을 곱한다.
        """
        world = self.pixel_to_world(u, v)
        if world is None:
            return None

        x_mm, y_mm = world
        return x_mm * 0.1, y_mm * 0.1

    def _world_to_pixel(self, x_mm: float, y_mm: float):
        """
        패널 mm 좌표를 이미지 픽셀 좌표로 역변환.
        """
        if self.H is None:
            return None

        _, inv_H = cv2.invert(self.H)

        src = np.array([[[x_mm, y_mm]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(src, inv_H)

        u = int(dst[0, 0, 0])
        v = int(dst[0, 0, 1])

        return u, v

    def robot_vector_from_world(self, x_mm: float, y_mm: float):
        """
        패널 좌표를 로봇 기준 벡터로 변환.

        현재 로봇 설치:
            로봇 정면 = 패널 아래쪽 방향

        반환:
            dx: 오른쪽 +
            dy: 로봇 전방 +
            r_mm
            theta_deg: 로봇 정면 기준 각도, 오른쪽 +, 왼쪽 -
        """
        robot_x_mm = getattr(CFG, "ROBOT_ORIGIN_X_MM", self.panel_width_mm / 2.0)
        robot_y_mm = getattr(CFG, "ROBOT_ORIGIN_Y_MM", 0.0)

        dx = x_mm - robot_x_mm
        dy = y_mm - robot_y_mm

        r_mm = float(np.sqrt(dx * dx + dy * dy))
        theta_deg = float(np.degrees(np.arctan2(dx, dy)))

        return dx, dy, r_mm, theta_deg

    def check_target_radius(self, u: float, v: float):
        """
        YOLO bottom point 또는 UI 클릭 point가 로봇 작업 반경 안에 있는지 검사.
        반환: ok, message, r_mm
        """
        world = self.pixel_to_world(u, v)

        if world is None:
            return False, "WARNING: panel not calibrated", None

        x_mm, y_mm = world
        _, _, r_mm, _ = self.robot_vector_from_world(x_mm, y_mm)

        min_r_mm = getattr(CFG, "MIN_RADIUS_MM", 80.0)
        max_r_mm = getattr(CFG, "MAX_RADIUS_MM", 260.0)

        if r_mm < min_r_mm:
            return False, f"WARNING! TOO CLOSE r={r_mm / 10.0:.1f}cm", r_mm

        if r_mm > max_r_mm:
            return False, f"WARNING! OUT OF RANGE r={r_mm / 10.0:.1f}cm", r_mm

        return True, f"OK r={r_mm / 10.0:.1f}cm", r_mm

    def draw_workspace_radius(self, frame):
        """
        로봇 중심 기준 전방 부채꼴 작업 반경 표시.
        MIN R / MAX R을 전체 원이 아니라 로봇 정면 기준 ±72.5도 범위로 표시한다.
        """
        if frame is None or self.H is None:
            return frame

        robot_x_mm = getattr(CFG, "ROBOT_ORIGIN_X_MM", self.panel_width_mm / 2.0)
        robot_y_mm = getattr(CFG, "ROBOT_ORIGIN_Y_MM", 0.0)

        min_r_mm = getattr(CFG, "MIN_RADIUS_MM", 80.0)
        max_r_mm = getattr(CFG, "MAX_RADIUS_MM", 260.0)

        robot_px = self._world_to_pixel(robot_x_mm, robot_y_mm)
        if robot_px is None:
            return frame

        cv2.circle(frame, robot_px, 8, (0, 0, 255), -1)
        cv2.drawMarker(
            frame,
            robot_px,
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=30,
            thickness=2,
        )
        cv2.putText(
            frame,
            "ROBOT ORIGIN",
            (robot_px[0] + 10, robot_px[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        def draw_projected_arc(radius_mm: float, color, label: str):
            pts = []

            # 로봇 정면 = 패널 아래쪽 방향
            # 좌우 작업 각도 = ±72.5도
            for deg in np.arange(-72.5, 72.5 + 0.1, 2.0):
                rad = np.deg2rad(deg)

                x_mm = robot_x_mm + radius_mm * np.sin(rad)
                y_mm = robot_y_mm + radius_mm * np.cos(rad)

                if 0.0 <= x_mm <= self.panel_width_mm and 0.0 <= y_mm <= self.panel_height_mm:
                    px = self._world_to_pixel(x_mm, y_mm)
                    if px is not None:
                        pts.append(px)

            if len(pts) >= 2:
                pts_np = np.array(pts, dtype=np.int32)
                cv2.polylines(
                    frame,
                    [pts_np],
                    isClosed=False,
                    color=color,
                    thickness=2,
                )

                label_px = pts[len(pts) // 2]
                cv2.putText(
                    frame,
                    label,
                    (label_px[0] + 5, label_px[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                    cv2.LINE_AA,
                )

        draw_projected_arc(min_r_mm, (0, 165, 255), "MIN R")
        draw_projected_arc(max_r_mm, (0, 255, 0), "MAX R")

        return frame

    def draw_target_radius(self, frame, u: float, v: float):
        """
        현재 YOLO target과 로봇 중심 사이의 거리 선 표시.
        """
        if frame is None or self.H is None:
            return frame

        target_world = self.pixel_to_world(u, v)
        if target_world is None:
            return frame

        x_mm, y_mm = target_world

        robot_x_mm = getattr(CFG, "ROBOT_ORIGIN_X_MM", self.panel_width_mm / 2.0)
        robot_y_mm = getattr(CFG, "ROBOT_ORIGIN_Y_MM", 0.0)

        robot_px = self._world_to_pixel(robot_x_mm, robot_y_mm)
        target_px = self._world_to_pixel(x_mm, y_mm)

        if robot_px is None or target_px is None:
            return frame

        _, _, r_mm, theta_deg = self.robot_vector_from_world(x_mm, y_mm)

        cv2.line(frame, robot_px, target_px, (0, 255, 255), 2)
        cv2.putText(
            frame,
            f"r={r_mm / 10.0:.1f}cm, th={theta_deg:.1f}deg",
            (
                int((robot_px[0] + target_px[0]) / 2),
                int((robot_px[1] + target_px[1]) / 2),
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        return frame

    def draw_panel(self, frame):
        """
        클릭한 패널 외곽선 + 로봇 중심 기준 작업 반경 표시.
        기존 10cm grid는 표시하지 않는다.
        """
        if frame is None:
            return frame

        for i, pt in enumerate(self.clicked_points):
            cv2.circle(frame, pt, 7, (0, 255, 0), -1)
            cv2.putText(
                frame,
                str(i + 1),
                (pt[0] + 8, pt[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        if self.H is None:
            cv2.putText(
                frame,
                f"Click table corners: {len(self.clicked_points)}/4",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            return frame

        pts = self.ordered_points.astype(np.int32)
        cv2.polylines(
            frame,
            [pts],
            isClosed=True,
            color=(0, 255, 255),
            thickness=2,
        )

        labels = [
            "LT (0,0)",
            "RT (60,0)",
            "RB (60,45)",
            "LB (0,45)",
        ]

        for label, pt in zip(labels, pts):
            cv2.circle(frame, tuple(pt), 7, (0, 0, 255), -1)
            cv2.putText(
                frame,
                label,
                (pt[0] + 8, pt[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        frame = self.draw_workspace_radius(frame)

        return frame