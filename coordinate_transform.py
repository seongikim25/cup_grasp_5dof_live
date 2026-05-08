from typing import Optional, Tuple
from config import CFG


class CoordinateTransformer:
    def __init__(self):
        self.panel_detector = None

    def set_panel_detector(self, panel_detector):
        """
        main.py 또는 tracker에서 자동 패널 검출기를 연결.
        """
        self.panel_detector = panel_detector

    def pixel_to_robot_xy(self, u: float, v: float) -> Tuple[float, float]:
        """
        YOLO keypoint 픽셀 좌표를 로봇 기준 xy(mm)로 변환.

        우선순위:
        1. 자동 패널 검출 Homography 사용
        2. 실패 시 기존 수동 Homography 또는 임시 선형 변환 사용
        """
        if self.panel_detector is not None:
            result = self.panel_detector.pixel_to_world(u, v)

            if result is not None:
                x_mm, y_mm = result

                # 필요하면 바닥 중심점 오프셋 적용
                x_mm += CFG.BOTTOM_OFFSET_X_MM
                y_mm += CFG.BOTTOM_OFFSET_Y_MM

                return x_mm, y_mm

        # 자동 검출 실패 시 임시 fallback
        # 기존 config의 대략적인 기준 사용
        # 이 부분은 panel_detector가 정상 작동하면 거의 사용되지 않음.
        x_mm = (u - 320.0) * 0.8
        y_mm = (480.0 - v) * 0.8

        x_mm += CFG.BOTTOM_OFFSET_X_MM
        y_mm += CFG.BOTTOM_OFFSET_Y_MM

        return x_mm, y_mm
