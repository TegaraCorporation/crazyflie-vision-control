"""
Crazyflie Vision Pipeline Tool — all-in-one script (Track A, no drone required)
================================================================================

Covers, in one file, run as a menu:
  1) Capture checkerboard images for calibration
  2) Run camera calibration
  3) Live ArUco marker detection test
  4) Generate a printable ArUco marker image
  5) Floor homography setup (pixel -> real-world meters)

WINDOWS SETUP (run these once, in a terminal, before running this script):

    python -m venv venv
    venv\\Scripts\\activate
    pip install opencv-contrib-python numpy

    Note: install "opencv-contrib-python", NOT plain "opencv-python" —
    the cv2.aruco module only ships in the contrib build. If both are
    installed at once they conflict; if aruco import fails below, run:
        pip uninstall opencv-python opencv-contrib-python -y
        pip install opencv-contrib-python

Then run this file:

    python vision_pipeline_tool.py

Everything below writes into a "cf_vision_data" folder created next to
this script, so you can move that one folder later without losing your
calibration/homography work.
"""

import glob
import json
import os
import sys

import cv2
import numpy as np

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cf_vision_data")
CHECKERBOARD_DIR = os.path.join(DATA_DIR, "checkerboard_images")
CALIB_FILE = os.path.join(DATA_DIR, "camera_calib.npz")
HOMOGRAPHY_FILE = os.path.join(DATA_DIR, "floor_homography.npy")
MARKER_FILE = os.path.join(DATA_DIR, "marker_id0.png")

CHECKERBOARD_CORNERS = (9, 6)   # internal corners, not squares
SQUARE_SIZE_M = 0.025           # measure your actual printed square size and update this
ARUCO_DICT_NAME = cv2.aruco.DICT_4X4_50
CAMERA_INDEX = 1                # 0 = laptop camera, 1 = usually the mounted USB camera (check with camera_setup.py)
CAMERA_WIDTH = 0                # 0 = use the camera's own default size (no change to the view).
CAMERA_HEIGHT = 0               # To force a size, set e.g. 1920 and 1080. Calibrate at the size you will fly with.

# CAP_DSHOW avoids a slow-open bug on Windows. It does not exist on Linux, where the default backend is used.
CAMERA_BACKEND = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY

# Written by camera_setup.py (press S). If this file exists, its size, focus and exposure
# are applied every time the camera is opened. Delete the file to go back to camera defaults.
SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_settings.json")


def ensure_dirs():
    os.makedirs(CHECKERBOARD_DIR, exist_ok=True)


def load_camera_settings():
    """Settings saved by camera_setup.py, or None if there is no usable file."""
    if not os.path.exists(SETTINGS_FILE):
        return None
    try:
        with open(SETTINGS_FILE) as f:
            return json.load(f)
    except (OSError, ValueError) as exc:
        print(f"WARNING: could not read {SETTINGS_FILE}: {exc}")
        return None


def apply_camera_settings(cap, settings):
    """Apply focus and exposure the same way camera_setup.py does."""
    if "autofocus" in settings:
        cap.set(cv2.CAP_PROP_AUTOFOCUS, settings["autofocus"])
        if settings["autofocus"] == 0 and "focus" in settings:
            cap.set(cv2.CAP_PROP_FOCUS, settings["focus"])
    if "auto_exposure" in settings:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if settings["auto_exposure"] else 0.25)
        if settings["auto_exposure"] == 0 and "exposure" in settings:
            cap.set(cv2.CAP_PROP_EXPOSURE, settings["exposure"])


def warn_if_calibration_mismatch(width, height):
    """A calibration (and the homography built from it) is only valid at its own image size."""
    if not os.path.exists(CALIB_FILE):
        return
    data = np.load(CALIB_FILE)
    if "size" in data:
        calib_w, calib_h = (int(v) for v in data["size"])
        mismatch = (calib_w, calib_h) != (width, height)
    else:
        # Older calibration file without a stored size: estimate it from the image centre.
        calib_w = int(round(float(data["mtx"][0, 2]) * 2))
        calib_h = int(round(float(data["mtx"][1, 2]) * 2))
        mismatch = abs(calib_w - width) > 0.2 * width or abs(calib_h - height) > 0.2 * height
    if mismatch:
        print(f"WARNING: the saved calibration was made at about {calib_w}x{calib_h}, "
              f"but the camera is now {width}x{height}.")
        print("         Redo options 1 and 2 (calibration) and option 5 (homography) at this size.")


def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX, CAMERA_BACKEND)
    if not cap.isOpened():
        print(f"ERROR: could not open camera index {CAMERA_INDEX}.")
        print("If you have more than one camera, try changing CAMERA_INDEX near the top of this file.")
        return None

    settings = load_camera_settings()
    want_w, want_h = CAMERA_WIDTH, CAMERA_HEIGHT
    if not (want_w and want_h) and settings and settings.get("width") and settings.get("height"):
        want_w, want_h = int(settings["width"]), int(settings["height"])

    if want_w and want_h:
        # Many USB webcams only give high resolutions in MJPG mode.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, want_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, want_h)
    if settings:
        apply_camera_settings(cap, settings)
        print(f"Camera settings loaded from {SETTINGS_FILE}")
        print("  " + ", ".join(f"{k} {v}" for k, v in settings.items() if k in
                               ("autofocus", "focus", "auto_exposure", "exposure")))
    if (want_w and want_h) or settings:
        for _ in range(40):  # let the camera settle on the new mode and settings
            cap.read()

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera resolution: {actual_w}x{actual_h}")
    if want_w and (actual_w, actual_h) != (want_w, want_h):
        print(f"WARNING: asked for {want_w}x{want_h} but the camera gave {actual_w}x{actual_h}.")
    warn_if_calibration_mismatch(actual_w, actual_h)
    return cap


# ---------------------------------------------------------------------------
# 1) Capture checkerboard images
# ---------------------------------------------------------------------------
def capture_checkerboard():
    print("\n--- Checkerboard capture ---")
    print(f"Print a {CHECKERBOARD_CORNERS[0]+1}x{CHECKERBOARD_CORNERS[1]+1}-square checkerboard "
          f"(= {CHECKERBOARD_CORNERS[0]}x{CHECKERBOARD_CORNERS[1]} internal corners), "
          f"tape it flat, and measure one square's real size in meters.")
    print(f"Current assumed square size: {SQUARE_SIZE_M} m — edit SQUARE_SIZE_M in this file if different.\n")
    print("A target box will appear on screen. Hold the board so it FILLS that box,")
    print("then press SPACE. The box moves to a new position every 3 captures —")
    print("just follow it. Tilt the board a bit differently each time within a position.\n")
    print("Controls: SPACE = capture frame, ESC = finish and return to menu.\n")

    cap = open_camera()
    if cap is None:
        return
    # The saved manual exposure is tuned for the floor view. A white checkerboard held
    # close to the camera would be washed out, so use automatic exposure here. Resolution
    # and focus (which are what the calibration depends on) stay as set.
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
    for _ in range(30):
        cap.read()

    existing = len(glob.glob(os.path.join(CHECKERBOARD_DIR, "img_*.png")))
    count = existing

    # Positions as fractions of frame width/height, and target box size as a
    # fraction of frame size — this is what "fills 1/3 to 2/3 of the frame"
    # actually looks like, instead of leaving distance to guesswork.
    POSITIONS = [
        ("center", 0.50, 0.50),
        ("top-left", 0.28, 0.32),
        ("top-right", 0.72, 0.32),
        ("bottom-left", 0.28, 0.70),
        ("bottom-right", 0.72, 0.70),
    ]
    BOX_FRAC_W = 0.42
    BOX_FRAC_H = 0.50
    SHOTS_PER_POSITION = 3

    pos_index = 0
    shots_here = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        h, w = frame.shape[:2]
        name, cx_frac, cy_frac = POSITIONS[pos_index]
        box_w = int(w * BOX_FRAC_W)
        box_h = int(h * BOX_FRAC_H)
        cx = int(w * cx_frac)
        cy = int(h * cy_frac)
        x0, y0 = cx - box_w // 2, cy - box_h // 2
        x1, y1 = cx + box_w // 2, cy + box_h // 2

        display = frame.copy()
        cv2.rectangle(display, (x0, y0), (x1, y1), (0, 255, 255), 2)
        cv2.putText(display, f"Fill this box: {name}  ({shots_here}/{SHOTS_PER_POSITION} here)",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(display, f"Captured total: {count}  (SPACE=capture, ESC=finish early)",
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        cv2.imshow("Checkerboard capture", display)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break
        if key == 32:  # SPACE
            path = os.path.join(CHECKERBOARD_DIR, f"img_{count:02d}.png")
            cv2.imwrite(path, frame)
            print(f"Saved {path}  [{name}]")
            count += 1
            shots_here += 1
            if shots_here >= SHOTS_PER_POSITION:
                shots_here = 0
                pos_index += 1
                if pos_index >= len(POSITIONS):
                    print("\nOne full pass done (all positions captured), finishing.")
                    break

    cap.release()
    cv2.destroyAllWindows()
    print(f"\nDone. {count} images total in {CHECKERBOARD_DIR}")
    if count < 10:
        print("Tip: aim for 15-20 images — one full pass through all 5 box positions.")


# ---------------------------------------------------------------------------
# 2) Run camera calibration
# ---------------------------------------------------------------------------
def run_calibration():
    print("\n--- Camera calibration ---")
    images = glob.glob(os.path.join(CHECKERBOARD_DIR, "img_*.png"))
    if len(images) < 5:
        print(f"Only found {len(images)} images in {CHECKERBOARD_DIR}.")
        print("Run option 1 first and capture at least 10-15 checkerboard images.")
        return

    objp = np.zeros((CHECKERBOARD_CORNERS[0] * CHECKERBOARD_CORNERS[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD_CORNERS[0], 0:CHECKERBOARD_CORNERS[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_M

    objpoints, imgpoints = [], []
    gray_shape = None
    used = 0

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_shape = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_CORNERS, None)
        if found:
            objpoints.append(objp)
            # The refinement window must be smaller than one printed square, or the corners
            # get pulled to the wrong place (this happens at 640x480, where a square is ~10 px).
            pts = corners.reshape(-1, 2)
            square_px = float(np.median(np.linalg.norm(pts[1:CHECKERBOARD_CORNERS[0]] - pts[:CHECKERBOARD_CORNERS[0] - 1], axis=1)))
            win = int(max(3, min(11, round(0.6 * square_px))))
            corners2 = cv2.cornerSubPix(
                gray, corners, (win, win), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.0001)
            )
            imgpoints.append(corners2)
            used += 1
        else:
            print(f"  (checkerboard not found in {os.path.basename(fname)}, skipping)")

    if used < 5:
        print(f"\nOnly {used}/{len(images)} images had a detectable checkerboard. "
              f"Recapture with better lighting / full board visible.")
        return

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray_shape, None, None
    )

    print(f"\nCalibrated using {used}/{len(images)} images.")
    print(f"Reprojection error: {ret:.4f}  (good is under ~0.5; recapture more varied angles if higher)")
    np.savez(CALIB_FILE, mtx=mtx, dist=dist, size=np.array(gray_shape))
    print(f"Saved: {CALIB_FILE}")


def load_calibration():
    if not os.path.exists(CALIB_FILE):
        print(f"\nNo calibration found at {CALIB_FILE}. Run option 2 first.")
        return None, None
    data = np.load(CALIB_FILE)
    return data["mtx"], data["dist"]


# ---------------------------------------------------------------------------
# 3) Live ArUco detection test
# ---------------------------------------------------------------------------
def run_aruco_detection():
    print("\n--- ArUco detection test ---")
    mtx, dist = load_calibration()
    if mtx is None:
        print("Continuing WITHOUT lens-distortion correction (less accurate, but fine for a quick test).")

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_NAME)
    aruco_params = cv2.aruco.DetectorParameters()
    # The default (0.03) rejects markers smaller than about 18 px at 1080p; a 30 mm marker
    # seen from a few metres up can be that small.
    aruco_params.minMarkerPerimeterRate = 0.01
    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    cap = open_camera()
    if cap is None:
        return

    # Try to set manual exposure to reduce flicker. Support varies by webcam/driver;
    # if detection still flickers, lower/raise the exposure value below or use your
    # webcam's own control panel software.
    # Skipped when camera_settings.json exists, because those saved settings were already applied.
    if load_camera_settings() is None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        cap.set(cv2.CAP_PROP_EXPOSURE, -6)

    print("Press ESC to close the window and return to menu.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        img = cv2.undistort(frame, mtx, dist) if mtx is not None else frame
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            for c, marker_id in zip(corners, ids):
                center = c[0].mean(axis=0)
                cv2.circle(img, tuple(center.astype(int)), 4, (0, 255, 0), -1)
                id_value = int(np.ravel(marker_id)[0])
                cv2.putText(img, f"id={id_value} px=({center[0]:.0f},{center[1]:.0f})",
                            (int(center[0]) + 10, int(center[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        else:
            cv2.putText(img, "No marker detected", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow("ArUco detection (ESC to exit)", img)
        if (cv2.waitKey(1) & 0xFF) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# 4) Generate a printable marker
# ---------------------------------------------------------------------------
def generate_marker():
    print("\n--- Generate printable ArUco marker ---")
    marker_id = 0
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_NAME)
    img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, 600)
    cv2.imwrite(MARKER_FILE, img)
    print(f"Saved: {MARKER_FILE}")
    print("Print this at roughly 7-8 cm square. Dictionary: DICT_4X4_50, ID 0.")
    print("(Make sure to also use DICT_4X4_50 / ID 0 in the detection step above — "
          "they must match.)")


# ---------------------------------------------------------------------------
# 5) Homography setup
# ---------------------------------------------------------------------------
def run_homography_setup():
    print("\n--- Floor homography setup ---")
    mtx, dist = load_calibration()
    if mtx is None:
        print("Calibration recommended before this step for accurate results, but you can continue.")

    print("You will click 4 points on the live camera image, in this exact order:")
    print("  1) Origin (0, 0)")
    print("  2) +X corner")
    print("  3) +X, +Y corner (opposite the origin)")
    print("  4) +Y corner")
    print("\nThe clicked order must go COUNTER-CLOCKWISE as you see it on screen")
    print("(for example bottom-left, bottom-right, top-right, top-left). A clockwise")
    print("order would mirror the Y axis.")
    print("\nThe drone will fly at a fixed height above the floor. For the best accuracy,")
    print("put the 4 reference points at that same height (for example small stands of")
    print("equal height at the corners) and click their tops. Measure each point's")
    print("position along the floor with a tape measure.\n")

    def ask_float(prompt, default):
        raw = input(f"{prompt} [{default}]: ").strip()
        return float(raw) if raw else default

    width = ask_float("Distance from origin to the +X corner, in metres", 2.0)
    depth = ask_float("Distance from origin to the +Y corner, in metres", 2.0)
    FLOOR_POINTS_M = np.array([
        [0.0, 0.0],
        [width, 0.0],
        [width, depth],
        [0.0, depth],
    ], dtype=np.float32)

    clicked = []

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicked) < 4:
            clicked.append([x, y])
            labels = ["origin (0,0)", "+X corner", "+X+Y corner", "+Y corner"]
            print(f"  Point {len(clicked)} ({labels[len(clicked)-1]}): pixel ({x}, {y})")

    cap = open_camera()
    if cap is None:
        return

    window_name = "Click 4 floor corners in order (ESC to abort)"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, on_click)

    while len(clicked) < 4:
        ret, frame = cap.read()
        if not ret:
            continue
        img = cv2.undistort(frame, mtx, dist) if mtx is not None else frame
        for i, p in enumerate(clicked):
            cv2.circle(img, tuple(p), 6, (0, 0, 255), -1)
            cv2.putText(img, str(i + 1), (p[0] + 8, p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow(window_name, img)
        if (cv2.waitKey(1) & 0xFF) == 27:
            print("Aborted.")
            cap.release()
            cv2.destroyAllWindows()
            return

    cap.release()
    cv2.destroyAllWindows()

    pixel_points = np.array(clicked, dtype=np.float32)

    # Mirror check: shoelace sum in image coordinates (y points down). A counter-clockwise
    # order on screen gives a negative sum; a positive sum means the order was clockwise.
    xs, ys = pixel_points[:, 0], pixel_points[:, 1]
    shoelace = float(np.sum(xs * np.roll(ys, -1) - np.roll(xs, -1) * ys))
    if shoelace > 0:
        print("\nWARNING: the points were clicked CLOCKWISE on screen. The Y axis will be")
        print("mirrored. Re-run option 5 and click counter-clockwise, or this will make")
        print("the drone move the wrong way along Y.")

    H, _ = cv2.findHomography(pixel_points, FLOOR_POINTS_M)
    np.save(HOMOGRAPHY_FILE, H)
    print(f"\nSaved: {HOMOGRAPHY_FILE}")

    def pixel_to_floor(px, py, H):
        pt = np.array([px, py, 1.0])
        result = H @ pt
        return result[0] / result[2], result[1] / result[2]

    check = pixel_to_floor(pixel_points[0][0], pixel_points[0][1], H)
    print(f"Sanity check — origin point maps back to: ({check[0]:.3f}, {check[1]:.3f})  (should be ~0, 0)")


# ---------------------------------------------------------------------------
# Menu
# ---------------------------------------------------------------------------
def main():
    ensure_dirs()

    if not hasattr(cv2, "aruco"):
        print("ERROR: cv2.aruco not found.")
        print("You likely have plain 'opencv-python' installed instead of 'opencv-contrib-python'.")
        print("Fix with:")
        print("  pip uninstall opencv-python opencv-contrib-python -y")
        print("  pip install opencv-contrib-python")
        sys.exit(1)

    while True:
        print("\n" + "=" * 60)
        print("Crazyflie Vision Pipeline Tool")
        print("=" * 60)
        print(f"Data folder: {DATA_DIR}")
        print(f"Calibration:  {'FOUND' if os.path.exists(CALIB_FILE) else 'not yet created'}")
        print(f"Homography:   {'FOUND' if os.path.exists(HOMOGRAPHY_FILE) else 'not yet created'}")
        print("-" * 60)
        print("1) Capture checkerboard images")
        print("2) Run camera calibration")
        print("3) Live ArUco detection test")
        print("4) Generate printable ArUco marker")
        print("5) Floor homography setup")
        print("0) Exit")
        choice = input("\nChoose an option: ").strip()

        if choice == "1":
            capture_checkerboard()
        elif choice == "2":
            run_calibration()
        elif choice == "3":
            run_aruco_detection()
        elif choice == "4":
            generate_marker()
        elif choice == "5":
            run_homography_setup()
        elif choice == "0":
            print("Exiting.")
            break
        else:
            print("Invalid choice, try again.")


if __name__ == "__main__":
    main()
