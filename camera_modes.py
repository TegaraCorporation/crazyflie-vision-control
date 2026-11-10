"""
camera_modes.py - find out which resolutions a camera really delivers.

For each size it tries two ways of asking the camera:
    default : just ask for the size (cameras like the RealSense D435 use this)
    MJPG    : ask for the size and also ask for MJPG compression
            (many ordinary USB webcams need this for the big sizes)
It reports the size the camera actually gave you and a sharpness score
(higher = sharper, only compare scores taken of the same scene), and it
saves one picture per mode so you can look at them.

Keep the marker and the scene the same while it runs.

Run (the number is the camera index; 0 = laptop camera, 1 = usually the mounted one):
    python camera_modes.py 1
"""

import sys

import cv2

MODES = [(640, 480), (1280, 720), (1920, 1080)]
BACKEND = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
SETTLE_FRAMES = 40  # auto-exposure needs time after a size change


def sharpness(gray):
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def try_mode(index, w, h, use_mjpg):
    cap = cv2.VideoCapture(index, BACKEND)
    if not cap.isOpened():
        return None
    if use_mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    ok, frame = False, None
    for _ in range(SETTLE_FRAMES):
        ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        return None
    return frame


def main():
    index = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    probe = cv2.VideoCapture(index, BACKEND)
    if not probe.isOpened():
        print(f"Camera {index} did not open.")
        return
    print(f"Camera {index}")
    print(f"  Zoom property reported: {probe.get(cv2.CAP_PROP_ZOOM)}  (0 or -1 often means unsupported)")
    probe.release()

    for w, h in MODES:
        for use_mjpg in (False, True):
            label = "MJPG   " if use_mjpg else "default"
            frame = try_mode(index, w, h, use_mjpg)
            if frame is None:
                print(f"  asked {w}x{h} [{label}]: no frame")
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ah, aw = frame.shape[:2]
            name = f"camera{index}_{aw}x{ah}_{label.strip().lower()}.png"
            cv2.imwrite(name, frame)
            print(f"  asked {w}x{h} [{label}]: got {aw}x{ah}, "
                  f"sharpness {sharpness(gray):.0f}, mean brightness {gray.mean():.0f}, saved {name}")


if __name__ == "__main__":
    main()
