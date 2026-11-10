"""
inspect_corners.py — diagnostic tool: draws detected checkerboard corners
onto each saved calibration image so you can visually confirm they landed
in the right place. Run this from the SAME folder as vision_pipeline_tool.py
(it reads from cf_vision_data/checkerboard_images and writes annotated
copies into cf_vision_data/checkerboard_inspect).

Usage:
    python inspect_corners.py
"""

import os
import glob
import cv2

CHECKERBOARD_CORNERS = (9, 6)  # must match vision_pipeline_tool.py

BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cf_vision_data")
SRC_DIR = os.path.join(BASE, "checkerboard_images")
OUT_DIR = os.path.join(BASE, "checkerboard_inspect")

os.makedirs(OUT_DIR, exist_ok=True)

images = sorted(glob.glob(os.path.join(SRC_DIR, "img_*.png")))
if not images:
    print(f"No images found in {SRC_DIR}")
    raise SystemExit

print(f"Inspecting {len(images)} images...\n")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_CORNERS, None)

    base = os.path.basename(fname)
    if not found:
        print(f"  {base}: NOT DETECTED")
        continue

    corners2 = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    )

    # Measure sharpness (Laplacian variance) as a blur indicator
    sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()

    annotated = img.copy()
    cv2.drawChessboardCorners(annotated, CHECKERBOARD_CORNERS, corners2, found)
    cv2.putText(annotated, f"sharpness={sharpness:.0f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    out_path = os.path.join(OUT_DIR, base)
    cv2.imwrite(out_path, annotated)

    flag = "  <-- LOW SHARPNESS, check for blur" if sharpness < 100 else ""
    print(f"  {base}: OK, sharpness={sharpness:.0f}{flag}")

print(f"\nAnnotated images saved to: {OUT_DIR}")
print("Open that folder and check: do the green/red corner dots land exactly")
print("on the checkerboard corners, or do they look offset/scattered?")
print("Also compare the sharpness numbers — anything much lower than the rest")
print("is a candidate to delete and recapture.")
