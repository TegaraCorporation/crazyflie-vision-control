"""
camera_fps_test.py - measure the real frame rate of the camera for each picture size,
compression (MJPG or YUY2) and exposure mode, so we can see what limits it.

Close every other program that uses the camera, then run:
    python camera_fps_test.py          (camera index 1)
    python camera_fps_test.py 0        (another camera index)

It takes about 1 minute and prints a table. Send the whole table back.
"""

import sys
import time

import cv2

INDEX = int(sys.argv[1]) if len(sys.argv) > 1 else 1
BACKEND = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
SIZES = [(640, 480), (1280, 720), (1920, 1080)]
CODECS = ["MJPG", "YUY2"]
EXPOSURES = [("auto", None), ("manual -6", -6), ("manual -8", -8)]
MEASURE_SECONDS = 3.0


def fourcc_text(cap):
    code = int(cap.get(cv2.CAP_PROP_FOURCC))
    return "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4))


def measure(width, height, codec, exp_name, exp_value):
    cap = cv2.VideoCapture(INDEX, BACKEND)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*codec))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if exp_value is None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
    else:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        cap.set(cv2.CAP_PROP_EXPOSURE, exp_value)
    for _ in range(15):          # let the camera settle
        cap.read()
    got = f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}"
    codec_got = fourcc_text(cap)
    count = 0
    t0 = time.time()
    while time.time() - t0 < MEASURE_SECONDS:
        ok, _ = cap.read()
        if ok:
            count += 1
    fps = count / (time.time() - t0)
    cap.release()
    return got, codec_got, fps


def main():
    print(f"Camera index {INDEX}")
    print(f"{'asked size':<11}{'codec asked':<13}{'exposure':<12}{'got size':<11}{'codec got':<11}fps")
    for width, height in SIZES:
        for codec in CODECS:
            for exp_name, exp_value in EXPOSURES:
                result = measure(width, height, codec, exp_name, exp_value)
                if result is None:
                    print(f"{width}x{height:<6}{codec:<13}{exp_name:<12}could not open the camera")
                    continue
                got, codec_got, fps = result
                print(f"{str(width) + 'x' + str(height):<11}{codec:<13}{exp_name:<12}{got:<11}{codec_got:<11}{fps:.1f}")
    print("Done.")


if __name__ == "__main__":
    main()
