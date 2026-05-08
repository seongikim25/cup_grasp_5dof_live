from dataclasses import dataclass
from typing import Optional, Tuple, List
import time
import subprocess

import cv2
import numpy as np
from ultralytics import YOLO

from config import CFG


@dataclass
class Detection:
    u: float
    v: float
    conf: float
    bbox: Tuple[int, int, int, int]
    class_name: str
    timestamp: float


class LiveYOLOTracker:
    def __init__(self):
        self.camera_index = getattr(CFG, "CAMERA_INDEX", 0)
        self.model_path = getattr(CFG, "YOLO_MODEL_PATH", "best.pt")

        self.conf_thres = getattr(CFG, "YOLO_CONF_THRESHOLD", 0.25)
        self.imgsz = getattr(CFG, "YOLO_IMGSZ", 640)
        self.flip_horizontal = getattr(CFG, "CAMERA_FLIP_HORIZONTAL", False)

        self.stable_window = getattr(CFG, "STABLE_WINDOW", 5)
        self.stable_pixel_thresh = getattr(CFG, "STABLE_PIXEL_THRESHOLD", 12.0)

        self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            raise RuntimeError(f"Camera open failed: index={self.camera_index}")

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.set_manual_exposure(2046)

        self.model = YOLO(self.model_path)

        self.history: List[Detection] = []
        self.latest_detection: Optional[Detection] = None

        print(f"[Camera] index={self.camera_index}")
        print(f"[YOLO Pose] model={self.model_path}")
        print(f"[YOLO Pose] imgsz={self.imgsz}, conf={self.conf_thres}")
        print("[Device] CPU mode")

    def _device_path(self):
        return f"/dev/video{self.camera_index}"

    def set_manual_exposure(self, exposure_value: int = 250):
        device = self._device_path()

        try:
            subprocess.run(
                ["v4l2-ctl", "-d", device, "-c", "auto_exposure=1"],
                check=False
            )

            subprocess.run(
                ["v4l2-ctl", "-d", device, "-c", f"exposure_time_absolute={exposure_value}"],
                check=False
            )

            print(f"[Exposure] manual exposure = {exposure_value}")

        except Exception as e:
            print(f"[Exposure] manual exposure failed: {e}")

    def adjust_exposure(self, delta: int, step_scale: int = 1):
        device = self._device_path()

        BASE_STEP = 5
        step = BASE_STEP * step_scale

        try:
            current = subprocess.check_output(
                ["v4l2-ctl", "-d", device, "-C", "exposure_time_absolute"],
                text=True
            )

            current_value = int(current.strip().split(":")[-1].strip())
            new_value = max(3, min(2047, current_value + delta * step))

            subprocess.run(
                ["v4l2-ctl", "-d", device, "-c", "auto_exposure=1"],
                check=False
            )

            subprocess.run(
                ["v4l2-ctl", "-d", device, "-c", f"exposure_time_absolute={new_value}"],
                check=False
            )

            print(f"[Exposure] {current_value} -> {new_value} step={step}")

        except Exception as e:
            print(f"[Exposure] v4l2 adjust failed: {e}")

    def toggle_auto_exposure(self):
        device = self._device_path()

        try:
            current = subprocess.check_output(
                ["v4l2-ctl", "-d", device, "-C", "auto_exposure"],
                text=True
            )

            current_value = int(current.strip().split(":")[-1].strip())

            if current_value == 1:
                subprocess.run(
                    ["v4l2-ctl", "-d", device, "-c", "auto_exposure=3"],
                    check=False
                )
                print("[Exposure] auto ON")
            else:
                subprocess.run(
                    ["v4l2-ctl", "-d", device, "-c", "auto_exposure=1"],
                    check=False
                )
                print("[Exposure] auto OFF / manual ON")

        except Exception as e:
            print(f"[Exposure] toggle failed: {e}")

    def _detect_pose(self, frame):
        results = self.model(
            frame,
            imgsz=self.imgsz,
            conf=self.conf_thres,
            device="cpu",
            verbose=False
        )

        if len(results) == 0:
            return None

        result = results[0]

        if result.boxes is None or len(result.boxes) == 0:
            return None

        best_idx = None
        best_conf = -1.0

        for i, box in enumerate(result.boxes):
            conf = float(box.conf[0].item())
            if conf > best_conf:
                best_conf = conf
                best_idx = i

        if best_idx is None or best_conf < self.conf_thres:
            return None

        box = result.boxes[best_idx]
        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]

        cls_id = int(box.cls[0].item()) if box.cls is not None else 0
        class_name = self.model.names.get(cls_id, str(cls_id))

        if result.keypoints is not None and result.keypoints.xy is not None:
            kpt_xy = result.keypoints.xy[best_idx]

            if len(kpt_xy) > 0:
                u = float(kpt_xy[0][0].item())
                v = float(kpt_xy[0][1].item())
            else:
                u, v = self._fallback_point((x1, y1, x2, y2))
        else:
            u, v = self._fallback_point((x1, y1, x2, y2))

        return Detection(
            u=u,
            v=v,
            conf=best_conf,
            bbox=(x1, y1, x2, y2),
            class_name=class_name,
            timestamp=time.time()
        )

    def _fallback_point(self, bbox):
        x1, y1, x2, y2 = bbox
        u = (x1 + x2) / 2.0
        v = y1 + 0.65 * (y2 - y1)
        return u, v

    def read(self):
        ok, frame = self.cap.read()

        if not ok or frame is None:
            return None, frame, False

        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        det = self._detect_pose(frame)

        if det is None:
            self.latest_detection = None
            self.history.clear()
            return None, frame, True

        self.latest_detection = det
        self.history.append(det)

        if len(self.history) > self.stable_window:
            self.history.pop(0)

        return det, frame, True

    def stable_detection(self):
        if len(self.history) < self.stable_window:
            return None

        us = np.array([d.u for d in self.history], dtype=np.float32)
        vs = np.array([d.v for d in self.history], dtype=np.float32)

        du = us.max() - us.min()
        dv = vs.max() - vs.min()

        if du <= self.stable_pixel_thresh and dv <= self.stable_pixel_thresh:
            latest = self.history[-1]

            return Detection(
                u=float(us.mean()),
                v=float(vs.mean()),
                conf=latest.conf,
                bbox=latest.bbox,
                class_name=latest.class_name,
                timestamp=latest.timestamp
            )

        return None

    def draw(self, frame, det: Optional[Detection], stable: Optional[Detection], status: str):
        if frame is None:
            return None

        drawn = frame.copy()

        if det is not None:
            x1, y1, x2, y2 = det.bbox

            cv2.rectangle(drawn, (x1, y1), (x2, y2), (255, 0, 0), 2)

            label = f"{det.class_name} {det.conf:.2f}"
            cv2.putText(
                drawn,
                label,
                (x1, max(25, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 0, 0),
                2,
                cv2.LINE_AA
            )

            cv2.circle(drawn, (int(det.u), int(det.v)), 7, (0, 0, 255), -1)
            cv2.circle(drawn, (int(det.u), int(det.v)), 12, (0, 0, 255), 2)

            cv2.putText(
                drawn,
                f"bottom ({det.u:.0f},{det.v:.0f})",
                (int(det.u) + 10, int(det.v) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2,
                cv2.LINE_AA
            )

        if stable is not None:
            cv2.circle(drawn, (int(stable.u), int(stable.v)), 14, (255, 0, 0), 2)

            cv2.putText(
                drawn,
                "STABLE TARGET",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 0, 0),
                2,
                cv2.LINE_AA
            )

        cv2.putText(
            drawn,
            status[:80],
            (20, drawn.shape[0] - 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            drawn,
            "YOLOv8 Pose | bbox + bottom-center keypoint | CPU",
            (20, drawn.shape[0] - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        return drawn

    def release(self):
        if self.cap is not None:
            self.cap.release()