# 색깔을 찾아내는 pick_color.py 코드
import cv2
import numpy as np

CAM_INDEX = 0  # 필요하면 1, 2로 변경

clicked_hsv_values = []

def mouse_callback(event, x, y, flags, param):
    global frame, hsv

    if event == cv2.EVENT_LBUTTONDOWN:
        b, g, r = frame[y, x]
        h, s, v = hsv[y, x]

        clicked_hsv_values.append((int(h), int(s), int(v)))

        print("\n[CLICK]")
        print(f"Pixel: x={x}, y={y}")
        print(f"BGR: B={int(b)}, G={int(g)}, R={int(r)}")
        print(f"HSV: H={int(h)}, S={int(s)}, V={int(v)}")

        if len(clicked_hsv_values) >= 2:
            arr = np.array(clicked_hsv_values)
            print("\n[HSV clicked range so far]")
            print("H min/max:", arr[:, 0].min(), arr[:, 0].max())
            print("S min/max:", arr[:, 1].min(), arr[:, 1].max())
            print("V min/max:", arr[:, 2].min(), arr[:, 2].max())

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)

if not cap.isOpened():
    print(f"카메라를 열 수 없습니다: index={CAM_INDEX}")
    exit()

cv2.namedWindow("click red tape")
cv2.setMouseCallback("click red tape", mouse_callback)

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    frame = cv2.resize(frame, (640, 480))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    show = frame.copy()
    cv2.putText(
        show,
        "Left click red tape. Press Q to quit.",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA
    )

    cv2.imshow("click red tape", show)

    if cv2.waitKey(1) & 0xFF in [ord("q"), ord("Q")]:
        break

cap.release()
cv2.destroyAllWindows()
