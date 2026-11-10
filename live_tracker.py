"""
live_tracker.py - show the marker's floor position (metres) live, and log it to a CSV.

Needs both files from vision_pipeline_tool.py first:
    cf_vision_data/camera_calib.npz        (option 1 + 2: camera calibration)
    cf_vision_data/floor_homography.npy    (option 5: floor homography)

Put this file in the same folder as vision_pipeline_tool.py, then run:
    python live_tracker.py

What you see:
    x, y   marker centre on the floor, in metres, in the frame you defined in option 5
    yaw    marker heading in degrees, 0 = along +X, counter-clockwise positive
    A log file tracker_log_<date>_<time>.csv is written next to this script.

Keys:
    ESC   quit
    L     start / stop writing the log (the log is on from the start)
"""

import csv
import os
import time

import cv2
import numpy as np

import vision_pipeline_tool as vp

MARKER_ID = 0


def make_detector():
    params = cv2.aruco.DetectorParameters()
    params.minMarkerPerimeterRate = 0.01
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    # Read each cell of a small marker more finely; fixes missed detections at 1920x1080.
    params.perspectiveRemovePixelPerCell = 8
    params.perspectiveRemoveIgnoredMarginPerCell = 0.25
    dictionary = cv2.aruco.getPredefinedDictionary(vp.ARUCO_DICT_NAME)
    return cv2.aruco.ArucoDetector(dictionary, params)


def pixel_to_floor(H, px, py):
    result = H @ np.array([px, py, 1.0])
    return float(result[0] / result[2]), float(result[1] / result[2])


def marker_pose_on_floor(H, corner_set):
    """Centre (x, y) in metres and yaw in degrees for one detected marker.

    OpenCV lists the corners clockwise on screen starting at the marker's top-left,
    so the vector from corner 0 to corner 1 is the marker's own 'right' direction.
    Yaw is that direction in the floor frame (0 = +X, counter-clockwise positive).
    """
    pts = np.array(corner_set, dtype=np.float32).reshape(4, 2)
    floor = np.array([pixel_to_floor(H, p[0], p[1]) for p in pts])
    centre = floor.mean(axis=0)
    right = floor[1] - floor[0]
    yaw = float(np.degrees(np.arctan2(right[1], right[0])))
    return float(centre[0]), float(centre[1]), yaw


def main():
    mtx, dist = vp.load_calibration()
    if mtx is None:
        print("Run options 1 and 2 of vision_pipeline_tool.py first.")
        return
    if not os.path.exists(vp.HOMOGRAPHY_FILE):
        print(f"No homography found at {vp.HOMOGRAPHY_FILE}. Run option 5 first.")
        return
    H = np.load(vp.HOMOGRAPHY_FILE)

    cap = vp.open_camera()
    if cap is None:
        return
    detector = make_detector()

    log_name = time.strftime("tracker_log_%Y%m%d_%H%M%S.csv")
    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), log_name)
    logging_on = True
    t0 = time.time()
    print(f"Logging to {log_path}")
    print("ESC = quit, L = log on/off")

    with open(log_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_seconds", "visible", "x_m", "y_m", "yaw_deg", "pixel_x", "pixel_y"])

        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            t = time.time() - t0
            img = cv2.undistort(frame, mtx, dist)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            found = None
            if ids is not None:
                for c, marker_id in zip(corners, ids):
                    if int(np.ravel(marker_id)[0]) == MARKER_ID:
                        found = c
                        break

            if found is not None:
                x, y, yaw = marker_pose_on_floor(H, found)
                centre = np.array(found).reshape(4, 2).mean(axis=0)
                cv2.polylines(img, [np.array(found).reshape(4, 2).astype(np.int32)], True, (0, 255, 0), 2)
                text = f"x {x:+.3f} m   y {y:+.3f} m   yaw {yaw:+.0f} deg"
                colour = (0, 255, 0)
                if logging_on:
                    writer.writerow([f"{t:.3f}", 1, f"{x:.4f}", f"{y:.4f}", f"{yaw:.1f}",
                                     f"{centre[0]:.1f}", f"{centre[1]:.1f}"])
            else:
                text = "marker not visible"
                colour = (0, 0, 255)
                if logging_on:
                    writer.writerow([f"{t:.3f}", 0, "", "", "", "", ""])

            cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)
            cv2.putText(img, "logging ON" if logging_on else "logging OFF", (10, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow("Live tracker (ESC to quit, L = log on/off)", img)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if key in (ord("l"), ord("L")):
                logging_on = not logging_on

    cap.release()
    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
