"""
camera_setup.py - camera setup window with sliders and a live marker-size readout.

Use it to set resolution, zoom, focus and exposure for the overhead camera, and to
see how many pixels wide the ArUco marker is at the camera's real mounting height.

Run (the number is the camera index; 0 = laptop camera, 1 = usually the mounted one):
    python camera_setup.py 1

Keys:
    1 / 2 / 3 / 4   switch resolution: 640x480, 1280x720, 1920x1080, 3840x2160
    S               save the current settings to camera_settings.json (next to this script;
                    vision_pipeline_tool.py and live_tracker.py then use them automatically)
    P               save the current full-resolution frame as a PNG (camera_frame_*.png)
    R               show / hide squares the detector found but could not decode (magenta)
    ESC             quit

Sliders (only the ones your camera supports will have any effect):
    Zoom            camera zoom value (the range depends on the camera; try it)
    AutoFocus       1 = camera focuses by itself, 0 = fixed focus (use 0 once it is sharp)
    Focus           focus value, used when AutoFocus is 0
    AutoExposure    1 = automatic, 0 = manual
    Exposure        manual exposure; the slider value n is sent as -n (Windows webcams
                    usually use negative values, larger n = shorter exposure)

The fps number in the second line is how many pictures per second the camera really gives.
Aim for 20 or more. A low number (about 5) means the exposure is too long: raise the
Exposure slider.

The marker readout:
    green   20 px or more across  - good
    yellow  12 to 20 px           - marginal, may drop out when the drone tilts
    red     under 12 px           - too small

Important: if you change zoom or focus after calibrating the camera, the calibration
is no longer valid. Finish this setup first, then calibrate.
"""

import json
import os
import sys
import time

import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_4X4_50
RESOLUTIONS = {
    ord("1"): (640, 480),
    ord("2"): (1280, 720),
    ord("3"): (1920, 1080),
    ord("4"): (3840, 2160),
}
GOOD_PX = 20
MARGINAL_PX = 12
WIN = "Camera setup  (S save, ESC quit, 1-4 resolution)"
DISPLAY_MAX_WIDTH = 1280


def make_detector():
    params = cv2.aruco.DetectorParameters()
    # The default (0.03) rejects markers smaller than about 18 px at 1080p.
    params.minMarkerPerimeterRate = 0.01
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    # Read each cell of a small marker more finely; fixes missed detections at 1920x1080.
    params.perspectiveRemovePixelPerCell = 8
    params.perspectiveRemoveIgnoredMarginPerCell = 0.25
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    return cv2.aruco.ArucoDetector(dictionary, params)


def marker_side_px(corner_set):
    pts = np.array(corner_set, dtype=np.float32).reshape(4, 2)
    sides = [np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]
    return float(np.mean(sides))


def sharpness(gray, corner_set=None):
    if corner_set is not None:
        pts = np.array(corner_set, dtype=np.int32).reshape(4, 2)
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        crop = gray[max(y0, 0):y1 + 1, max(x0, 0):x1 + 1]
    else:
        h, w = gray.shape
        crop = gray[h // 3: 2 * h // 3, w // 3: 2 * w // 3]
    if crop.size == 0:
        return 0.0
    return float(cv2.Laplacian(crop, cv2.CV_64F).var())


def set_resolution(cap, width, height):
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    for _ in range(5):
        cap.read()
    return int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))


def clamp(value, lo, hi):
    return max(lo, min(hi, int(value)))


def main():
    index = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        print(f"Camera {index} did not open.")
        return

    width, height = set_resolution(cap, 1920, 1080)
    print(f"Camera {index}: running at {width}x{height}")

    detector = make_detector()
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)

    zoom0 = clamp(cap.get(cv2.CAP_PROP_ZOOM), 0, 500)
    focus0 = clamp(cap.get(cv2.CAP_PROP_FOCUS), 0, 255)
    cv2.createTrackbar("Zoom", WIN, zoom0, 500, lambda v: None)
    cv2.createTrackbar("AutoFocus", WIN, 1, 1, lambda v: None)
    cv2.createTrackbar("Focus", WIN, focus0, 255, lambda v: None)
    cv2.createTrackbar("AutoExposure", WIN, 1, 1, lambda v: None)
    cv2.createTrackbar("Exposure", WIN, 6, 13, lambda v: None)

    applied = {"Zoom": zoom0, "AutoFocus": None, "Focus": None,
               "AutoExposure": None, "Exposure": None}
    show_rejected = False
    snap_count = 0
    last_t = time.time()
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        now = time.time()
        if now > last_t:
            fps = 0.9 * fps + 0.1 / (now - last_t)
        last_t = now

        # Apply a slider only when it changes, so we do not fight the camera.
        pos = {name: cv2.getTrackbarPos(name, WIN)
               for name in ("Zoom", "AutoFocus", "Focus", "AutoExposure", "Exposure")}
        if pos["Zoom"] != applied["Zoom"]:
            cap.set(cv2.CAP_PROP_ZOOM, pos["Zoom"])
        if pos["AutoFocus"] != applied["AutoFocus"]:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, pos["AutoFocus"])
        if pos["Focus"] != applied["Focus"] and pos["AutoFocus"] == 0:
            cap.set(cv2.CAP_PROP_FOCUS, pos["Focus"])
        if pos["AutoExposure"] != applied["AutoExposure"]:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if pos["AutoExposure"] else 0.25)
        if pos["Exposure"] != applied["Exposure"] and pos["AutoExposure"] == 0:
            cap.set(cv2.CAP_PROP_EXPOSURE, -pos["Exposure"])
        applied.update(pos)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        scale = min(1.0, DISPLAY_MAX_WIDTH / frame.shape[1])
        view = cv2.resize(frame, None, fx=scale, fy=scale) if scale < 1.0 else frame.copy()

        # Squares the detector found but could not decode (blur, glare, bad margin).
        if show_rejected and rejected is not None:
            for cand in rejected:
                pts = (np.array(cand).reshape(4, 2) * scale).astype(np.int32)
                cv2.polylines(view, [pts], True, (255, 0, 255), 1)

        if ids is not None and len(ids) > 0:
            sizes = []
            for corner_set in corners:
                side = marker_side_px(corner_set)
                sizes.append(side)
                c = (0, 200, 0) if side >= GOOD_PX else (0, 200, 255) if side >= MARGINAL_PX else (0, 0, 255)
                pts = (np.array(corner_set).reshape(4, 2) * scale).astype(np.int32)
                cv2.polylines(view, [pts], True, c, 2)
                cv2.putText(view, f"{side:.0f}", (int(pts[0][0]), int(pts[0][1]) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
            smallest = min(sizes)
            color = (0, 200, 0) if smallest >= GOOD_PX else (0, 200, 255) if smallest >= MARGINAL_PX else (0, 0, 255)
            label = (f"{len(sizes)} marker(s) found, smallest {smallest:.0f} px, "
                     f"sharpness {sharpness(gray, corners[0]):.0f}")
        else:
            color = (0, 0, 255)
            label = f"no marker found, sharpness {sharpness(gray):.0f}"

        n_rej = 0 if rejected is None else len(rejected)
        actual_zoom = cap.get(cv2.CAP_PROP_ZOOM)
        hud = (f"{frame.shape[1]}x{frame.shape[0]}  {fps:.0f} fps  zoom(read back) {actual_zoom:.0f}  "
               f"undecoded squares {n_rej} (R shows them)")
        # black outline first, so the text stays readable on a white picture
        cv2.putText(view, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 5)
        cv2.putText(view, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(view, hud, (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 5)
        cv2.putText(view, hud, (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow(WIN, view)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key in (ord("r"), ord("R")):
            show_rejected = not show_rejected
        if key in (ord("p"), ord("P")):
            name = f"camera_frame_{frame.shape[1]}x{frame.shape[0]}_{snap_count}.png"
            cv2.imwrite(name, frame)
            snap_count += 1
            print(f"Saved full-resolution frame: {name}")
        if key in RESOLUTIONS:
            w, h = RESOLUTIONS[key]
            aw, ah = set_resolution(cap, w, h)
            print(f"Asked {w}x{h}, camera gave {aw}x{ah}")
        if key in (ord("s"), ord("S")):
            # Save the values that were SET (the sliders), not what the driver reads back,
            # so that vision_pipeline_tool.py can apply exactly the same thing later.
            settings = {
                "camera_index": index,
                "width": int(frame.shape[1]),
                "height": int(frame.shape[0]),
                "autofocus": pos["AutoFocus"],
                "focus": pos["Focus"],
                "auto_exposure": pos["AutoExposure"],
                "exposure": -pos["Exposure"],
            }
            settings_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_settings.json")
            with open(settings_path, "w") as f:
                json.dump(settings, f, indent=2)
            print(f"Saved {settings_path}:", settings)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
