"""
compare_saved_images.py - fair comparison of the pictures saved by camera_modes.py.

For every camera<N>_*.png in the current folder it reports:
    - whether the ArUco marker is found, and how many pixels wide it is
    - the marker width as a fraction of the picture width (so sizes can be compared)
    - sharpness measured on the picture shrunk to 640 px wide, so that all modes
      are compared on equal terms (a raw score always falls as the pixel count rises)
    - the sharpness measured around the marker only

Run it in the folder where camera_modes.py saved its pictures:
    python compare_saved_images.py
"""

import glob

import cv2
import numpy as np

DICT = cv2.aruco.DICT_4X4_50
REF_WIDTH = 640


def make_detector():
    params = cv2.aruco.DetectorParameters()
    params.minMarkerPerimeterRate = 0.01
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICT), params)


def lap_var(gray):
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def main():
    files = sorted(glob.glob("camera*_*.png"))
    if not files:
        print("No camera*_*.png pictures found in this folder. Run camera_modes.py here first.")
        return
    detector = make_detector()
    print(f"{'picture':34s} {'size':>10s} {'marker px':>10s} {'% of width':>10s} "
          f"{'sharp@640':>10s} {'sharp@marker':>12s}")
    for name in files:
        img = cv2.imread(name)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        small = cv2.resize(gray, (REF_WIDTH, int(round(h * REF_WIDTH / w))), interpolation=cv2.INTER_AREA)
        sharp_small = lap_var(small)

        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None and len(ids) > 0:
            pts = np.array(corners[0], dtype=np.float32).reshape(4, 2)
            side = float(np.mean([np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]))
            x0, y0 = pts.min(axis=0).astype(int)
            x1, y1 = pts.max(axis=0).astype(int)
            crop = gray[max(y0, 0):y1 + 1, max(x0, 0):x1 + 1]
            crop_small = cv2.resize(crop, None, fx=REF_WIDTH / w, fy=REF_WIDTH / w,
                                    interpolation=cv2.INTER_AREA) if crop.size else crop
            sharp_marker = lap_var(crop_small) if crop_small.size > 4 else float("nan")
            print(f"{name:34s} {w:>5d}x{h:<4d} {side:>10.1f} {100 * side / w:>9.2f}% "
                  f"{sharp_small:>10.0f} {sharp_marker:>12.0f}")
        else:
            print(f"{name:34s} {w:>5d}x{h:<4d} {'NOT FOUND':>10s} {'-':>10s} "
                  f"{sharp_small:>10.0f} {'-':>12s}")


if __name__ == "__main__":
    main()
