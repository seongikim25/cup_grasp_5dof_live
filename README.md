# 5DOF AX-12A 실시간 YOLO 컵 파지 코드

이 버전은 컵 좌표를 랜덤으로 만들지 않는다.  
카메라 프레임에서 YOLO가 실시간으로 컵을 검출하고, 안정된 중심점이 확보되면 그 좌표를 로봇 좌표로 변환해 파지한다.

## 설치

```bash
pip install pygame opencv-python numpy pyserial ultralytics
```

## 실행 - 개발 모드

하드웨어 명령 없이 카메라/YOLO/좌표/ROI만 확인한다.

```bash
python3 main.py --model best.pt --cam 0
## 윈도우에서는 python3 우분투 환경에서는 python 을 넣으면 된다
```

## 실행 - 실제 로봇 제어

```bash
python3 main.py --run --model best.pt --cam 0
```

## 동작

- 실시간 카메라 영상에서 컵 검출
- 최근 N프레임 동안 중심점 흔들림이 작으면 stable target으로 판단
- SPACE를 누르면 해당 안정 좌표로 파지 시퀀스 실행
- ROI 반경 밖이면 파지하지 않고 경고 출력

## 파지 시퀀스 

1. 컵 중간 높이로 가까이 접근하면서 그리퍼가 수평이 된 상태로 된 자세 유지
2. 같은 위치에서 ID: (5번, 6번) 그리퍼를 닫기
3. 상승

## 커스텀 수정가능 값

`config.py`

- YOLO_MODEL_PATH
- CAMERA_INDEX
- IMAGE_POINTS
- WORLD_POINTS_MM
- MIN_RADIUS_MM
- MAX_RADIUS_MM
- LINK1_MM
- LINK2_MM
- HOME_POS
- GRIPPER_OPEN
- GRIPPER_CLOSE
- 각 축 encoder 방향은 `ik_5dof.py`의 invert/center에서 보정

## Arduino/OpenRB

`robot_5dof_ax12a.ino`는 openRB에서만 사용한다.

- ID1: Base
- ID2: Shoulder
- ID3: Elbow
- ID4: Wrist
- ID5: Gripper 1
- ID6: Gripper 2

Python 명령 형식:

```text
0,base,shoulder,elbow,wrist,gripper*
```

## 가상환경 실행
cd ~/Desktop/cup_grasp_5dof_live
source -venv/bin/activate
python main.py

## 가상환경 비활성화
deactivate

