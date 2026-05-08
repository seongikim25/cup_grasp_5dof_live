import argparse
import pygame
import cv2
import numpy as np

from config import CFG
from coordinate_transform import CoordinateTransformer
from live_yolo_tracker import LiveYOLOTracker
from workspace import check_workspace
from robot_controller import RobotController
from panel_detector import PanelDetector


VIEW_X = 20
VIEW_Y = 20
VIEW_W = 800
VIEW_H = 600

SCREEN_W = 1100
SCREEN_H = 760

BG_COLOR = (245, 247, 250)
TEXT_COLOR = (30, 41, 59)
BLUE = (37, 99, 235)
RED = (220, 38, 38)


def cv_to_pygame(frame):
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    rgb = np.transpose(rgb, (1, 0, 2))
    return pygame.surfarray.make_surface(rgb)


def draw_big_warning(frame, warning_text: str):
    if frame is None or warning_text is None:
        return frame

    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w, 95), (0, 0, 255), -1)
    cv2.putText(
        frame,
        warning_text,
        (25, 62),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.35,
        (255, 255, 255),
        4,
        cv2.LINE_AA,
    )
    return frame


def draw_target_marker(frame, target_px):
    """
    클릭한 place target을 50% 반투명 원으로 표시.
    """
    if frame is None or target_px is None:
        return frame

    x, y = target_px
    x = int(x)
    y = int(y)

    overlay = frame.copy()
    radius = 38
    alpha = 0.5

    cv2.circle(overlay, (x, y), radius, (255, 0, 255), -1, cv2.LINE_AA)
    cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0, frame)

    cv2.circle(frame, (x, y), radius, (255, 0, 255), 3, cv2.LINE_AA)
    cv2.circle(frame, (x, y), 5, (255, 255, 255), -1, cv2.LINE_AA)

    cv2.putText(
        frame,
        f"CLICKED)",
        (x + radius + 8, y - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 0, 255),
        2,
        cv2.LINE_AA,
    )
    return frame


def screen_to_frame(mouse_x, mouse_y, frame):
    """
    Pygame 화면 좌표를 원본 frame 픽셀 좌표로 변환.
    카메라 영상은 화면의 (20,20)에 800x600으로 표시됨.
    """
    if frame is None:
        return None

    if not (VIEW_X <= mouse_x <= VIEW_X + VIEW_W):
        return None

    if not (VIEW_Y <= mouse_y <= VIEW_Y + VIEW_H):
        return None

    frame_h, frame_w = frame.shape[:2]

    u = (mouse_x - VIEW_X) * frame_w / VIEW_W
    v = (mouse_y - VIEW_Y) * frame_h / VIEW_H

    return int(u), int(v)


def make_panel_detector():
    return PanelDetector(
        panel_width_mm=CFG.PANEL_WIDTH_MM,
        panel_height_mm=CFG.PANEL_HEIGHT_MM,
        threshold_v=CFG.PANEL_THRESHOLD_V,
        min_area_ratio=CFG.PANEL_MIN_AREA_RATIO,
        max_area_ratio=CFG.PANEL_MAX_AREA_RATIO,
        debug=True,
    )


def build_drawn_frame(
    tracker,
    frame,
    det,
    stable,
    status,
    panel_detector,
    place_target_px,
    emergency_stopped,
    waiting_place_target,
    radius_ok,
    radius_warning,
):
    drawn = tracker.draw(frame, det, stable, status)

    if drawn is None:
        return None

    drawn = panel_detector.draw_panel(drawn)

    if stable is not None and panel_detector.is_calibrated():
        drawn = panel_detector.draw_target_radius(drawn, stable.u, stable.v)

    if place_target_px is not None:
        drawn = draw_target_marker(drawn, place_target_px)

    if emergency_stopped:
        drawn = draw_big_warning(drawn, "EMERGENCY STOP")
    elif not radius_ok and radius_warning is not None and not waiting_place_target:
        drawn = draw_big_warning(drawn, radius_warning)
    elif waiting_place_target:
        drawn = draw_big_warning(drawn, "CLICK PLACE TARGET")

    return drawn


def update_detection_state(stable, transformer, panel_detector, waiting_place_target, emergency_stopped, current_status):
    """
    stable YOLO detection을 패널 좌표/작업반경으로 변환.
    반환:
        latest_xy, latest_workspace, radius_ok, radius_warning, latest_radius_mm, status
    """
    latest_xy = None
    latest_workspace = None
    radius_ok = True
    radius_warning = None
    latest_radius_mm = None
    status = current_status

    if stable is None:
        return latest_xy, latest_workspace, radius_ok, radius_warning, latest_radius_mm, status

    x_mm, y_mm = transformer.pixel_to_robot_xy(stable.u, stable.v)
    latest_xy = (x_mm, y_mm)
    latest_workspace = check_workspace(x_mm, y_mm)

    cm_result = panel_detector.pixel_to_cm(stable.u, stable.v)

    if panel_detector.is_calibrated():
        radius_ok, radius_warning, latest_radius_mm = panel_detector.check_target_radius(stable.u, stable.v)

    if cm_result is not None:
        x_cm, y_cm = cm_result
        radius_text = ""

        if latest_radius_mm is not None:
            radius_text = f" r={latest_radius_mm / 10.0:.1f}cm"

        if not waiting_place_target and not emergency_stopped:
            status = (
                f"stable cup: pixel=({stable.u:.0f},{stable.v:.0f}) "
                f"cm=({x_cm:.1f},{y_cm:.1f}) "
                f"mm=({x_mm:.1f},{y_mm:.1f})"
                f"{radius_text} "
                f"{latest_workspace.message}"
            )
    else:
        if not waiting_place_target and not emergency_stopped:
            status = (
                f"stable cup: pixel=({stable.u:.0f},{stable.v:.0f}) "
                f"robot=({x_mm:.1f},{y_mm:.1f}) "
                f"{latest_workspace.message}"
            )

    if not radius_ok and radius_warning is not None and not waiting_place_target and not emergency_stopped:
        status = radius_warning

    return latest_xy, latest_workspace, radius_ok, radius_warning, latest_radius_mm, status


def preview_place_target(
    screen,
    font,
    small,
    frame,
    panel_detector,
    place_target_px,
    u,
    v,
    target_x_mm,
    target_y_mm,
    target_r_mm,
):
    """
    robot.place_cup() 실행 전에 클릭 위치를 화면에 먼저 보여줌.
    """
    preview = frame.copy()
    preview = panel_detector.draw_panel(preview)
    preview = draw_target_marker(preview, place_target_px)
    preview = draw_big_warning(preview, "PLACE TARGET SELECTED")

    preview = cv2.resize(preview, (VIEW_W, VIEW_H))
    surf = cv_to_pygame(preview)

    screen.fill(BG_COLOR)
    screen.blit(surf, (VIEW_X, VIEW_Y))

    screen.blit(font.render("PLACE TARGET SELECTED", True, RED), (840, 30))
    screen.blit(small.render(f"pixel=({u},{v})", True, BLUE), (840, 65))
    screen.blit(
        small.render(f"mm=({target_x_mm:.1f},{target_y_mm:.1f})", True, BLUE),
        (840, 90),
    )
    screen.blit(
        small.render(f"r={target_r_mm / 10.0:.1f}cm", True, BLUE),
        (840, 115),
    )

    pygame.display.flip()
    pygame.time.wait(800)


def handle_place_click(
    u,
    v,
    frame,
    screen,
    font,
    small,
    panel_detector,
    robot,
):
    """
    place target 클릭 처리.
    반환:
        status, success, place_target_px, place_target_world
    """
    if not panel_detector.is_calibrated():
        return "WARNING: panel is not calibrated", False, None, None

    target_world = panel_detector.pixel_to_world(u, v)
    if target_world is None:
        return "WARNING: invalid place target", False, None, None

    target_x_mm, target_y_mm = target_world
    ok_target, target_warning, target_r_mm = panel_detector.check_target_radius(u, v)

    if not ok_target:
        return target_warning, False, None, None

    place_target_px = (u, v)
    place_target_world = (target_x_mm, target_y_mm)

    status = (
        f"place target selected: "
        f"pixel=({u},{v}) "
        f"x={target_x_mm:.1f}, y={target_y_mm:.1f}, "
        f"r={target_r_mm / 10.0:.1f}cm"
    )

    print(
        f"[Place Click] pixel=({u},{v}) "
        f"world=({target_x_mm:.1f},{target_y_mm:.1f})mm "
        f"r={target_r_mm:.1f}mm"
    )

    preview_place_target(
        screen=screen,
        font=font,
        small=small,
        frame=frame,
        panel_detector=panel_detector,
        place_target_px=place_target_px,
        u=u,
        v=v,
        target_x_mm=target_x_mm,
        target_y_mm=target_y_mm,
        target_r_mm=target_r_mm,
    )

    success = robot.place_cup(target_x_mm, target_y_mm)
    return status, success, place_target_px, place_target_world


def render_side_panel(
    screen,
    font,
    small,
    status,
    emergency_stopped,
    waiting_place_target,
    holding_cup,
    latest_radius_mm,
):
    panel_x = 840
    y = 30

    robot_x = getattr(CFG, "ROBOT_ORIGIN_X_MM", 0.0)
    robot_y = getattr(CFG, "ROBOT_ORIGIN_Y_MM", 0.0)

    radius_line = "Target R: --"
    if latest_radius_mm is not None:
        radius_line = f"Target R: {latest_radius_mm:.1f} mm"

    state_line = "State: IDLE"
    if emergency_stopped:
        state_line = "State: E-STOP"
    elif waiting_place_target:
        state_line = "State: WAIT PLACE"
    elif holding_cup:
        state_line = "State: HOLDING CUP"

    lines = [
        "5DOF AX-12A Cup Grasp",
        f"Mode: {'RUN' if CFG.RUN_MODE else 'DEV'}",
        state_line,
        "",
        "Panel Calibration",
        "Mouse : click 4 table corners",
        "R     : reset panel",
        "[ / ] : exposure down / up",
        "Hold 1.5s: fast exposure",
        "T     : toggle auto exposure",
        "",
        "Workspace Radius",
        f"Robot X: {robot_x:.1f} mm",
        f"Robot Y: {robot_y:.1f} mm",
        f"Min R  : {CFG.MIN_RADIUS_MM:.1f} mm",
        f"Max R  : {CFG.MAX_RADIUS_MM:.1f} mm",
        radius_line,
        "",
        "Pick & Place",
        "SPACE : grasp stable cup",
        "Click : select place target",
        "H     : home / recover",
        "E     : EMERGENCY STOP",
        "ESC   : quit",
    ]

    for line in lines:
        color = RED if "E-STOP" in line or "EMERGENCY" in line else TEXT_COLOR
        txt = font.render(line, True, color)
        screen.blit(txt, (panel_x, y))
        y += 26

    y += 8
    status_lines = [status[i:i + 34] for i in range(0, len(status), 34)]

    for line in status_lines:
        color = RED if "WARNING" in line or "EMERGENCY" in line or "STOP" in line else BLUE
        txt = small.render(line, True, color)
        screen.blit(txt, (panel_x, y))
        y += 24


def render_screen(
    screen,
    drawn,
    font,
    small,
    status,
    emergency_stopped,
    waiting_place_target,
    holding_cup,
    latest_radius_mm,
):
    screen.fill(BG_COLOR)

    if drawn is not None:
        drawn = cv2.resize(drawn, (VIEW_W, VIEW_H))
        surf = cv_to_pygame(drawn)
        screen.blit(surf, (VIEW_X, VIEW_Y))

    render_side_panel(
        screen=screen,
        font=font,
        small=small,
        status=status,
        emergency_stopped=emergency_stopped,
        waiting_place_target=waiting_place_target,
        holding_cup=holding_cup,
        latest_radius_mm=latest_radius_mm,
    )

    pygame.display.flip()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="store_true", help="실제 로봇으로 시리얼 명령 전송")
    parser.add_argument("--model", default=None, help="YOLO model path, e.g. best.pt")
    parser.add_argument("--cam", type=int, default=None, help="camera index")
    return parser.parse_args()


def apply_args(args):
    if args.run:
        CFG.RUN_MODE = True

    if args.model:
        CFG.YOLO_MODEL_PATH = args.model

    if args.cam is not None:
        CFG.CAMERA_INDEX = args.cam


def main():
    args = parse_args()
    apply_args(args)

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("5DOF Cup Grasp - Live YOLO Tracking")

    font = pygame.font.Font(None, 28)
    small = pygame.font.Font(None, 22)

    tracker = LiveYOLOTracker()
    transformer = CoordinateTransformer()
    robot = RobotController()
    panel_detector = make_panel_detector()

    transformer.set_panel_detector(panel_detector)

    clock = pygame.time.Clock()
    running = True

    status = "Mouse: click 4 table corners | SPACE: grasp | H: home | E: stop"

    latest_xy = None
    latest_workspace = None
    latest_radius_mm = None

    radius_ok = True
    radius_warning = None

    holding_cup = False
    waiting_place_target = False
    place_target_px = None
    place_target_world = None
    emergency_stopped = False

    exposure_hold_key = None
    exposure_hold_start = 0
    last_exposure_change = 0

    while running:
        det, frame, ok = tracker.read()
        stable = tracker.stable_detection()

        if frame is not None:
            panel_detector.detect_panel(frame)

        (
            new_xy,
            new_workspace,
            radius_ok,
            radius_warning,
            latest_radius_mm,
            status,
        ) = update_detection_state(
            stable=stable,
            transformer=transformer,
            panel_detector=panel_detector,
            waiting_place_target=waiting_place_target,
            emergency_stopped=emergency_stopped,
            current_status=status,
        )

        if new_xy is not None:
            latest_xy = new_xy

        if new_workspace is not None:
            latest_workspace = new_workspace

        drawn = build_drawn_frame(
            tracker=tracker,
            frame=frame,
            det=det,
            stable=stable,
            status=status,
            panel_detector=panel_detector,
            place_target_px=place_target_px,
            emergency_stopped=emergency_stopped,
            waiting_place_target=waiting_place_target,
            radius_ok=radius_ok,
            radius_warning=radius_warning,
        )

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                continue

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button != 1 or frame is None:
                    continue

                frame_pt = screen_to_frame(event.pos[0], event.pos[1], frame)
                if frame_pt is None:
                    continue

                u, v = frame_pt

                if emergency_stopped:
                    status = "EMERGENCY STOP active. Press H to recover."
                    continue

                if waiting_place_target:
                    (
                        status,
                        success,
                        place_target_px,
                        place_target_world,
                    ) = handle_place_click(
                        u=u,
                        v=v,
                        frame=frame,
                        screen=screen,
                        font=font,
                        small=small,
                        panel_detector=panel_detector,
                        robot=robot,
                    )

                    if success:
                        status = "place finished. robot moved home"
                        holding_cup = False
                        waiting_place_target = False
                        place_target_px = None
                        place_target_world = None
                    else:
                        if "WARNING" not in status:
                            status = "WARNING: place failed"
                    continue

                if not panel_detector.is_calibrated():
                    panel_detector.add_corner_point(int(u), int(v))
                    status = f"panel corner clicked: {len(panel_detector.clicked_points)}/4"

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_e:
                    robot.emergency_stop()
                    emergency_stopped = True
                    holding_cup = False
                    waiting_place_target = False
                    place_target_px = None
                    place_target_world = None
                    status = "EMERGENCY STOP! Torque OFF. Press H to recover."

                elif event.key == pygame.K_h:
                    if emergency_stopped:
                        robot.clear_emergency_stop()
                        emergency_stopped = False
                        status = "E-STOP cleared. moving home"
                    else:
                        status = "moving home"

                    robot.home()
                    holding_cup = False
                    waiting_place_target = False
                    place_target_px = None
                    place_target_world = None

                elif event.key == pygame.K_SPACE:
                    if emergency_stopped:
                        status = "EMERGENCY STOP active. Press H to recover."
                        continue

                    if stable is None or latest_xy is None or latest_workspace is None:
                        status = "WARNING: stable cup target is not ready"
                        continue

                    if not panel_detector.is_calibrated():
                        status = "WARNING: panel is not calibrated"
                        continue

                    radius_ok_now, radius_warning_now, _ = panel_detector.check_target_radius(stable.u, stable.v)

                    if not radius_ok_now:
                        status = radius_warning_now
                        continue

                    if not latest_workspace.ok:
                        status = latest_workspace.message
                        continue

                    x_mm, y_mm = latest_xy
                    place_target_px = None
                    place_target_world = None

                    status = f"grasp start: x={x_mm:.1f}, y={y_mm:.1f}"
                    success = robot.grasp_cup(x_mm, y_mm)

                    if success:
                        holding_cup = True
                        waiting_place_target = True
                        status = "grasp finished. click place target on UI"
                    else:
                        status = "WARNING: grasp failed"

                elif event.key == pygame.K_r:
                    panel_detector.reset()
                    latest_xy = None
                    latest_workspace = None
                    latest_radius_mm = None
                    radius_ok = True
                    radius_warning = None
                    holding_cup = False
                    waiting_place_target = False
                    place_target_px = None
                    place_target_world = None
                    status = "panel calibration reset. click 4 table corners"

                elif event.key == pygame.K_t:
                    tracker.toggle_auto_exposure()

                elif event.key in [pygame.K_LEFTBRACKET, pygame.K_RIGHTBRACKET]:
                    exposure_hold_key = event.key
                    exposure_hold_start = pygame.time.get_ticks()
                    last_exposure_change = 0

            if event.type == pygame.KEYUP:
                if event.key in [pygame.K_LEFTBRACKET, pygame.K_RIGHTBRACKET]:
                    exposure_hold_key = None

        if exposure_hold_key is not None:
            now = pygame.time.get_ticks()
            held_sec = (now - exposure_hold_start) / 1000.0

            if now - last_exposure_change >= 100:
                direction = -1 if exposure_hold_key == pygame.K_LEFTBRACKET else 1
                step_scale = 5 if held_sec >= 1.5 else 1
                tracker.adjust_exposure(direction, step_scale)
                last_exposure_change = now

        render_screen(
            screen=screen,
            drawn=drawn,
            font=font,
            small=small,
            status=status,
            emergency_stopped=emergency_stopped,
            waiting_place_target=waiting_place_target,
            holding_cup=holding_cup,
            latest_radius_mm=latest_radius_mm,
        )

        clock.tick(30)

    tracker.release()
    robot.close()
    pygame.quit()


if __name__ == "__main__":
    main()