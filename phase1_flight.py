"""
phase1_flight.py - Phase 1 (Method 2): the camera position is sent to the Crazyflie
as an external position, the drone's own estimator fuses it with the Flow Deck, and the
drone's built-in high-level commander does take-off, move, hold and landing.

You edit the TARGETS list below. The control law itself runs inside the drone (Phase 2 is
where you write your own controller on the PC).

Needs (same folder as vision_pipeline_tool.py and live_tracker.py):
    cf_vision_data/camera_calib.npz, cf_vision_data/floor_homography.npy,
    camera_settings.json (optional), and:  pip install cflib

Floor frame (from the homography): origin at stand 1, +X towards stand 2, +Y towards stand 4.
START THE DRONE WITH ITS NOSE ALONG +X. The drone's own heading is taken as 0 at power-on,
so a different start heading would make the drone's frame differ from the camera's.

Two modes:
    python phase1_flight.py --check     NO FLIGHT. Streams the camera position to the drone
                                        and compares it with the drone's own estimate for
                                        20 s. Slide the drone by hand over the floor and
                                        watch both follow each other. Do this first.
    python phase1_flight.py             Flight: take off, go to each target, hold, return,
                                        land.   (Linux: python3)

While it runs, click on the camera window first, then:
    ESC     land now (normal landing)
    SPACE   EMERGENCY STOP: motors off immediately, the drone drops
    Ctrl+C  in the terminal also does the emergency stop

It lands by itself if: the marker is lost for more than LOST_SECONDS, the marker leaves the
flight area by more than AREA_MARGIN, or the battery falls below MIN_VBAT_FLIGHT.

Before running:
    - Close cfclient (only one program can use the Crazyradio). Battery charged (4.0 V+).
    - Floor under the drone must have texture for the Flow Deck (the black paper is untested).
    - Clear 1.5 m around the area. Keep hands and face away from the propellers.
    - Run --check first. Then fly with a low HOVER_HEIGHT.
"""

import csv
import os
import sys
import time
from collections import deque

import cv2
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import live_tracker as lt
import vision_pipeline_tool as vp

# ----------------------------------------------------------------------------------------
# Settings you may change
# ----------------------------------------------------------------------------------------
URI = "radio://0/80/2M"        # the address the connect test used
HOVER_HEIGHT = 0.3             # metres. Must equal the stand height used for the floor map.
TARGETS = [(0.465, 0.295)]     # floor positions (x, y) in metres to fly to, one after another
AREA_X, AREA_Y = 0.93, 0.59    # flight area size, the same numbers as in the floor map step

MOVE_SECONDS = 3.0             # time allowed for each move
HOLD_SECONDS = 5.0             # hold time at each target
TAKEOFF_SECONDS = 2.0
LAND_SECONDS = 2.5

MIN_VBAT_START = 4.0          # volts: will not take off below this (voltage sags about 0.4 V under load)
MIN_VBAT_FLIGHT = 3.4          # volts: lands by itself below this
LOST_SECONDS = 3.0             # marker not seen for this long -> land by itself (safe landing).
                               # Meanwhile the drone keeps flying on its Flow Deck alone.
AREA_MARGIN = 0.25             # metres outside the flight area before it lands
EXT_POS_STD = 0.03             # metres: how much the drone trusts the camera (smaller = more).
                               # A starting guess: tune it from the logs.
CONVERGE_VAR = 0.001           # estimator variance spread that counts as "settled"
MAX_START_DIFF = 0.05          # metres: camera and drone estimate must agree this well
CHECK_SECONDS = 20.0           # length of --check mode
MIN_CAMERA_HZ = 10.0           # will not take off if the camera gives fewer frames per second

# ----------------------------------------------------------------------------------------


class AbortFlight(Exception):
    pass


class EmergencyStop(Exception):
    pass


class Ctx:
    """Everything the loop shares."""

    def __init__(self):
        self.cap = None
        self.detector = None
        self.mtx = self.dist = self.H = None
        self.cf = None
        self.phase = "start"
        self.cam = None              # last camera position (x, y)
        self.last_seen = 0.0
        self.ekf = (0.0, 0.0, 0.0)   # drone's own position estimate
        self.vbat = None
        self.var_hist = deque(maxlen=10)
        self.land_request = None
        self.emergency = False
        self.writer = None
        self.t0 = time.time()
        self.flying = False
        self.frame_times = deque(maxlen=40)
        self.brightness = 0.0
        self.last_frame = None
        self.log_configs = []


def camera_rate(ctx):
    """Camera frames per second, measured over the last 40 frames."""
    if len(ctx.frame_times) < 10:
        return 0.0
    span = ctx.frame_times[-1] - ctx.frame_times[0]
    return (len(ctx.frame_times) - 1) / span if span > 0 else 0.0


def camera_xy(H, corner_set, mtx, dist):
    """Marker centre on the floor in metres (corners undistorted first)."""
    pts = np.array(corner_set, dtype=np.float32).reshape(-1, 1, 2)
    und = cv2.undistortPoints(pts, mtx, dist, P=mtx).reshape(4, 2)
    floor = np.array([lt.pixel_to_floor(H, p[0], p[1]) for p in und])
    c = floor.mean(axis=0)
    return float(c[0]), float(c[1])


def start_logging(ctx):
    cf = ctx.cf

    pos = LogConfig(name="pos", period_in_ms=50)
    for name in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z"):
        pos.add_variable(name, "float")

    def on_pos(ts, data, conf):
        ctx.ekf = (data["stateEstimate.x"], data["stateEstimate.y"], data["stateEstimate.z"])

    var = LogConfig(name="var", period_in_ms=100)
    for name in ("kalman.varPX", "kalman.varPY", "kalman.varPZ"):
        var.add_variable(name, "float")

    def on_var(ts, data, conf):
        ctx.var_hist.append((data["kalman.varPX"], data["kalman.varPY"], data["kalman.varPZ"]))

    bat = LogConfig(name="bat", period_in_ms=500)
    bat.add_variable("pm.vbat", "float")

    def on_bat(ts, data, conf):
        ctx.vbat = data["pm.vbat"]

    for conf, cb in ((pos, on_pos), (var, on_var), (bat, on_bat)):
        cf.log.add_config(conf)
        ctx.log_configs.append(conf)
        conf.data_received_cb.add_callback(cb)
        conf.start()


def estimator_settled(ctx):
    if len(ctx.var_hist) < ctx.var_hist.maxlen:
        return False
    arr = np.array(ctx.var_hist)
    return bool(np.all(arr.max(axis=0) - arr.min(axis=0) < CONVERGE_VAR))


def process_frame(ctx):
    """Read one camera frame, send the position to the drone, log, show, read keys."""
    ok, frame = ctx.cap.read()
    if not ok:
        return
    now = time.time()
    ctx.frame_times.append(now)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ctx.brightness = float(gray.mean())
    ctx.last_frame = frame
    corners, ids, _ = ctx.detector.detectMarkers(gray)

    found = None
    if ids is not None:
        for c, marker_id in zip(corners, ids):
            if int(np.ravel(marker_id)[0]) == lt.MARKER_ID:
                found = c
                break

    if found is not None:
        x, y = camera_xy(ctx.H, found, ctx.mtx, ctx.dist)
        ctx.cam = (x, y)
        ctx.last_seen = now
        # z: the drone's own height estimate is sent back, so the camera only corrects x and y.
        ctx.cf.loc.send_extpos([x, y, ctx.ekf[2]])

    visible = found is not None
    if ctx.writer is not None:
        cx, cy = (ctx.cam if visible else (None, None))
        ctx.writer.writerow([
            f"{now - ctx.t0:.3f}", ctx.phase, int(visible),
            "" if cx is None else f"{cx:.4f}", "" if cy is None else f"{cy:.4f}",
            f"{ctx.ekf[0]:.4f}", f"{ctx.ekf[1]:.4f}", f"{ctx.ekf[2]:.4f}",
            "" if ctx.vbat is None else f"{ctx.vbat:.2f}"])

    scale = 960.0 / frame.shape[1]
    view = cv2.resize(frame, None, fx=scale, fy=scale)
    if found is not None:
        poly = (np.array(found).reshape(4, 2) * scale).astype(np.int32)
        cv2.polylines(view, [poly], True, (0, 255, 0), 2)
    cam_txt = "camera: marker lost" if not visible else f"camera x {ctx.cam[0]:+.3f} y {ctx.cam[1]:+.3f}"
    cv2.putText(view, f"{ctx.phase}   {camera_rate(ctx):.0f} fps", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(view, cam_txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 0) if visible else (0, 0, 255), 2)
    cv2.putText(view, f"drone  x {ctx.ekf[0]:+.3f} y {ctx.ekf[1]:+.3f} z {ctx.ekf[2]:.2f}",
                (10, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(view, "ESC = land    SPACE = EMERGENCY STOP", (10, view.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imshow("Phase 1 flight", view)

    key = cv2.waitKey(1) & 0xFF
    if key == 27 and ctx.land_request is None:
        ctx.land_request = "ESC pressed"
    if key == 32:
        ctx.emergency = True


def check_safety(ctx):
    if ctx.emergency:
        raise EmergencyStop()
    if ctx.land_request:
        raise AbortFlight(ctx.land_request)
    if not ctx.flying:
        return
    if time.time() - ctx.last_seen > LOST_SECONDS:
        raise AbortFlight(f"marker lost for more than {LOST_SECONDS} s")
    if ctx.cam is not None:
        x, y = ctx.cam
        if (x < -AREA_MARGIN or x > AREA_X + AREA_MARGIN or
                y < -AREA_MARGIN or y > AREA_Y + AREA_MARGIN):
            raise AbortFlight(f"marker outside the flight area (x {x:.2f}, y {y:.2f})")
    if ctx.vbat is not None and ctx.vbat < MIN_VBAT_FLIGHT:
        raise AbortFlight(f"battery low ({ctx.vbat:.2f} V)")


def run_for(ctx, seconds, phase):
    ctx.phase = phase
    end = time.time() + seconds
    while time.time() < end:
        process_frame(ctx)
        check_safety(ctx)


def wait_marker_stable(ctx, seconds=1.5, timeout=30.0):
    ctx.phase = "waiting for marker"
    stable_since = None
    t_start = time.time()
    end = t_start + timeout
    next_note = 5.0
    while time.time() < end:
        process_frame(ctx)
        if ctx.emergency or ctx.land_request:
            raise AbortFlight("cancelled")
        if time.time() - t_start >= next_note:
            next_note += 5.0
            print(f"  no steady marker yet. Picture brightness {ctx.brightness:.0f} of 255 "
                  f"(healthy is about 60 to 180), camera {camera_rate(ctx):.1f} fps")
            if ctx.last_frame is not None:
                snap = os.path.join(os.path.dirname(os.path.abspath(__file__)), "phase1_no_marker.png")
                cv2.imwrite(snap, ctx.last_frame)
                print(f"  saved what the camera sees: {snap}")
        if time.time() - ctx.last_seen < 0.2:
            stable_since = stable_since or time.time()
            if time.time() - stable_since >= seconds:
                return True
        else:
            stable_since = None
    return False


def inside_area(x, y):
    return 0.0 <= x <= AREA_X and 0.0 <= y <= AREA_Y


def main():
    check_only = "--check" in sys.argv

    for tx, ty in TARGETS:
        if not inside_area(tx, ty):
            print(f"Target ({tx}, {ty}) is outside the flight area. Fix TARGETS. Not flying.")
            return

    ctx = Ctx()
    ctx.mtx, ctx.dist = vp.load_calibration()
    if ctx.mtx is None:
        print("No camera calibration. Run options 1 and 2 of vision_pipeline_tool.py.")
        return
    if not os.path.exists(vp.HOMOGRAPHY_FILE):
        print("No floor map. Run option 5 of vision_pipeline_tool.py.")
        return
    ctx.H = np.load(vp.HOMOGRAPHY_FILE)
    ctx.detector = lt.make_detector()

    ctx.cap = vp.open_camera()
    if ctx.cap is None:
        return

    log_name = time.strftime("flight_log_%Y%m%d_%H%M%S.csv")
    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), log_name)
    log_file = open(log_path, "w", newline="")
    ctx.writer = csv.writer(log_file)
    ctx.writer.writerow(["t_seconds", "phase", "cam_visible", "cam_x_m", "cam_y_m",
                         "drone_x_m", "drone_y_m", "drone_z_m", "vbat_v"])
    print(f"Logging to {log_path}")

    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")
    scf_ctx = None
    try:
        scf_ctx = SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache"))
        scf = scf_ctx.__enter__()
    except Exception as exc:
        print(f"Connection failed: {exc}")
        print("Is cfclient closed? Is the Crazyflie powered on?")
        log_file.close()
        ctx.cap.release()
        return

    ctx.cf = scf.cf
    hl = ctx.cf.high_level_commander
    try:
        start_logging(ctx)
        t_wait = time.time() + 5
        while ctx.vbat is None and time.time() < t_wait:
            time.sleep(0.1)
        print(f"Battery: {ctx.vbat if ctx.vbat is None else round(ctx.vbat, 2)} V")
        if ctx.vbat is None or ctx.vbat < MIN_VBAT_START:
            print(f"Battery missing or below {MIN_VBAT_START} V. Not flying.")
            return

        flow = None
        for name in ("deck.bcFlow2", "deck.bcFlow"):
            try:
                if str(ctx.cf.param.get_value(name)).strip() == "1":
                    flow = name
                    break
            except Exception:
                pass
        if not flow:
            print("Flow Deck NOT detected. Press it on fully and power-cycle. Not flying.")
            return
        print(f"Flow Deck detected ({flow}).")

        # Kalman estimator + high-level commander, and how much the drone trusts the camera.
        ctx.cf.param.set_value("stabilizer.estimator", "2")
        ctx.cf.param.set_value("commander.enHighLevel", "1")
        try:
            ctx.cf.param.set_value("locSrv.extPosStdDev", str(EXT_POS_STD))
        except Exception as exc:
            print(f"Warning: could not set locSrv.extPosStdDev ({exc}). Using the drone's default.")

        print("Put the marker (drone) in the flight area, nose along +X. Waiting for the marker ...")
        if not wait_marker_stable(ctx):
            print("Marker not seen steadily. Not flying.")
            return
        if not inside_area(*ctx.cam):
            print(f"Marker is at x {ctx.cam[0]:.2f}, y {ctx.cam[1]:.2f}, outside the flight area. Not flying.")
            return
        start_xy = ctx.cam

        # Reset the drone's estimate and let the camera position pull it to the right place.
        ctx.phase = "estimator reset"
        ctx.cf.param.set_value("kalman.resetEstimation", "1")
        run_for(ctx, 0.2, "estimator reset")
        ctx.cf.param.set_value("kalman.resetEstimation", "0")
        ctx.var_hist.clear()

        print("Waiting for the drone's estimate to settle on the camera position ...")
        ctx.phase = "waiting for estimator"
        settled_at = None
        end = time.time() + 15
        while time.time() < end:
            process_frame(ctx)
            check_safety(ctx)
            if estimator_settled(ctx):
                settled_at = time.time()
                break
        if settled_at is None:
            print("The estimate did not settle in 15 s. Not flying. See the log file.")
            return
        dx = ctx.ekf[0] - ctx.cam[0]
        dy = ctx.ekf[1] - ctx.cam[1]
        print(f"Settled. Drone estimate minus camera: dx {dx:+.3f} m, dy {dy:+.3f} m")
        if abs(dx) > MAX_START_DIFF or abs(dy) > MAX_START_DIFF:
            print(f"They differ by more than {MAX_START_DIFF} m. Not flying.")
            return

        rate = camera_rate(ctx)
        print(f"Camera rate: {rate:.1f} frames per second")
        if rate < MIN_CAMERA_HZ:
            print(f"The camera gives fewer than {MIN_CAMERA_HZ:.0f} frames per second. Raise the Exposure")
            print("slider in camera_setup.py (shorter exposure) until the fps readout is 20 or more, and press S.")
            if not check_only:
                print("Not flying.")
                return

        if check_only:
            print(f"CHECK MODE: for {CHECK_SECONDS:.0f} s slide the drone by hand. No flight.")
            end = time.time() + CHECK_SECONDS
            last_print = 0.0
            while time.time() < end:
                process_frame(ctx)
                if ctx.emergency or ctx.land_request:
                    break
                if time.time() - last_print > 1.0 and ctx.cam is not None:
                    last_print = time.time()
                    print(f"  camera ({ctx.cam[0]:+.3f}, {ctx.cam[1]:+.3f})   "
                          f"drone ({ctx.ekf[0]:+.3f}, {ctx.ekf[1]:+.3f})")
            print("Check finished. Compare the two columns in the log file.")
            return

        # ---- flight ----
        try:
            ctx.cf.supervisor.send_arming_request(True)
        except Exception:
            pass
        print("Taking off in 3 seconds. SPACE = emergency stop, ESC = land.")
        run_for(ctx, 3.0, "countdown")

        try:
            ctx.flying = True
            hl.takeoff(HOVER_HEIGHT, TAKEOFF_SECONDS)
            run_for(ctx, TAKEOFF_SECONDS + 1.0, "take off")
            run_for(ctx, 2.0, "hover")
            for i, (tx, ty) in enumerate(TARGETS, start=1):
                print(f"Target {i}: ({tx:.3f}, {ty:.3f})")
                hl.go_to(tx, ty, HOVER_HEIGHT, 0.0, MOVE_SECONDS)
                run_for(ctx, MOVE_SECONDS, f"moving to target {i}")
                run_for(ctx, HOLD_SECONDS, f"holding target {i}")
            print("Returning to the start point.")
            hl.go_to(start_xy[0], start_xy[1], HOVER_HEIGHT, 0.0, MOVE_SECONDS)
            run_for(ctx, MOVE_SECONDS + 1.0, "returning")
            print("Landing.")
            hl.land(0.0, LAND_SECONDS)
            ctx.flying = False
            run_for(ctx, LAND_SECONDS + 0.5, "landing")
            hl.stop()
            print("Landed.")
        except AbortFlight as exc:
            print(f"LANDING NOW: {exc}")
            ctx.flying = False
            hl.land(0.0, LAND_SECONDS)
            ctx.phase = "landing (abort)"
            end = time.time() + LAND_SECONDS + 0.5
            while time.time() < end:
                process_frame(ctx)
                if ctx.emergency:
                    raise EmergencyStop()
            hl.stop()
            print("Landed.")

    except (EmergencyStop, KeyboardInterrupt):
        print("EMERGENCY STOP")
        try:
            ctx.cf.supervisor.send_emergency_stop()
        except Exception:
            ctx.cf.loc.send_emergency_stop()
    except AbortFlight as exc:
        print(f"Stopped before take-off: {exc}")
    finally:
        for conf in ctx.log_configs:
            try:
                conf.stop()
            except Exception:
                pass
        try:
            scf_ctx.__exit__(None, None, None)
        except Exception:
            pass
        ctx.cap.release()
        cv2.destroyAllWindows()
        log_file.close()
        print(f"Log saved: {log_path}")


if __name__ == "__main__":
    main()
