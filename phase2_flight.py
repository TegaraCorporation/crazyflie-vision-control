"""
phase2_flight.py - Phase 2 (Method 1): YOUR controller runs on the PC.

Every camera frame the PC measures where the drone is (ArUco marker -> floor position in
metres), compares it with the target, and decides how fast the drone should move. That speed
is sent to the drone over the Crazyradio. The drone itself only keeps its height and its
speed (Flow Deck); it does NOT know where the targets are. All the "thinking" about position
is in the function controller() below. That function is the part you are meant to edit.

    camera -> marker position -> controller() -> speed command -> radio -> drone

This is different from Phase 1: there the drone's own flight computer got the camera position
and flew to the target by itself. Here the PC closes the loop.

Needs (same folder as vision_pipeline_tool.py and live_tracker.py):
    cf_vision_data/camera_calib.npz, cf_vision_data/floor_homography.npy,
    camera_settings.json (optional), and:  pip install cflib

Floor frame: origin at stand 1, +X towards stand 2, +Y towards stand 4.
START THE DRONE WITH ITS NOSE ALONG +X (the drone's heading is 0 at power-on). The speed is
given to the drone in the floor frame and turned into the drone's own frame using the
drone's heading, so the two frames must agree at the start. If they do not, set
YAW_OFFSET_DEG.

Two modes:
    python phase2_flight.py --dry    NO RADIO, NO FLIGHT. Only the camera. Slide the marker by
                                     hand and watch what speed the controller would command.
                                     Do this first: it checks the directions.
    python phase2_flight.py          Flight: take off, visit each target, hold, return, land.
                                     (Linux: python3)

While it runs, click on the camera window first, then:
    ESC     land now (normal landing)
    SPACE   EMERGENCY STOP: motors off immediately, the drone drops
    Ctrl+C  in the terminal also does the emergency stop

It lands by itself if: the marker is lost for more than LOST_SECONDS, the marker leaves the
flight area by more than AREA_MARGIN, a target is not reached in WAYPOINT_TIMEOUT seconds,
the flight takes longer than MAX_FLIGHT_SECONDS, or the battery falls below MIN_VBAT_FLIGHT.
If the marker is only missing for a moment (up to STALE_SECONDS), the speed command is set to
zero and the drone hovers on its Flow Deck until the marker is back.

Before running: same checklist as Phase 1 (cfclient closed, battery charged, textured floor,
1.5 m clear around the area). Run --dry first, then fly with a low VMAX.
"""

import csv
import json
import math
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
AREA_X, AREA_Y = 0.93, 0.59    # flight area size, the same numbers as in the floor map step

# Targets: (x, y, hold_seconds). The drone visits them in order, then returns to where it
# started. Example of a square path (remove the # to try it after the single target works):
#   TARGETS = [(0.25, 0.15, 2), (0.68, 0.15, 2), (0.68, 0.44, 2), (0.25, 0.44, 2)]
TARGETS = [(0.465, 0.295, 5.0)]

# Controller (used by controller() below)
KP = 1.0                       # (m/s) per metre of position error. Larger = quicker but may overshoot.
KD = 0.0                       # (m/s) per (m/s) of measured speed. Damping. Try 0.2 if it overshoots.
VMAX = 0.20                    # m/s: the largest speed ever commanded (the output is clipped to this)

ARRIVE_TOL = 0.05              # metres: closer than this to the target counts as "arrived"
WAYPOINT_TIMEOUT = 15.0        # seconds allowed to reach one target before it lands
MAX_FLIGHT_SECONDS = 90.0      # whole flight, from take-off to landing
TAKEOFF_RATE = 0.15            # m/s climb
LAND_RATE = 0.15               # m/s descent

STALE_SECONDS = 0.3            # marker older than this -> command zero speed (hover)
LOST_SECONDS = 3.0             # marker not seen for this long -> land by itself
AREA_MARGIN = 0.25             # metres outside the flight area before it lands
MIN_VBAT_START = 4.0           # volts: will not take off below this
MIN_VBAT_FLIGHT = 3.4          # volts: lands by itself below this
MIN_CAMERA_HZ = 15.0           # will not take off if the camera gives fewer frames per second
YAW_OFFSET_DEG = 0.0           # floor-frame heading of the drone minus the drone's own heading, degrees

# Should the camera position ALSO be sent to the drone's own estimator (as in Phase 1)?
# False (default): the drone's estimator uses only the Flow Deck. The PC loop is then the only
#   thing that uses the camera position, which makes the loop easy to understand.
# True: the camera also corrects the drone's own estimate, as in Phase 1.
SEND_EXTPOS = False
EXT_POS_STD = 0.03             # metres, only used when SEND_EXTPOS is True
CONVERGE_VAR = 0.001           # only used when SEND_EXTPOS is True
MAX_START_DIFF = 0.05          # only used when SEND_EXTPOS is True

DRY_SECONDS = 60.0             # length of --dry mode

# Hard limits. The sliders, the settings file and controller() can never go beyond these.
HARD_VMAX = 0.40               # m/s
HARD_KP = 3.0
HARD_KD = 1.0
CLICK_MARGIN = 0.05            # metres: a clicked target is kept at least this far inside the area

# Optional settings file next to this script. Any of these keys overrides the value above:
#   KP, KD, VMAX, TARGETS (list of [x, y, hold_seconds]), MAX_FLIGHT_SECONDS
# Example file: phase2_settings.example.json  (copy it to phase2_settings.json)
SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "phase2_settings.json")

# ----------------------------------------------------------------------------------------


def controller(ctx, tx, ty):
    """THE PART TO EDIT.  Returns the wanted speed (vx, vy) in m/s in the FLOOR frame.

    ctx.cam      measured marker position (x, y) in metres
    ctx.cam_vel  measured marker speed (vx, vy) in m/s (noisy, filtered a little)
    tx, ty       the target position in metres

    This version is a proportional controller with a little damping:
        speed = KP * (target - position)  -  KD * measured speed
    and then the speed is limited to VMAX. Replace it with whatever you like (add an integral
    term, a different gain per axis, a path that curves, ...). Keep the output in m/s.
    """
    x, y = ctx.cam
    vx = KP * (tx - x) - KD * ctx.cam_vel[0]
    vy = KP * (ty - y) - KD * ctx.cam_vel[1]
    return limit_speed(vx, vy, VMAX)


def limit_speed(vx, vy, vmax):
    """Scale (vx, vy) down so its length is at most vmax; the direction is kept."""
    n = math.hypot(vx, vy)
    if n > vmax:
        return vx * vmax / n, vy * vmax / n
    return vx, vy


def floor_to_body(vx, vy, yaw_deg):
    """Turn a floor-frame speed into the drone's own frame (x forward, y left)."""
    a = math.radians(yaw_deg)
    return (math.cos(a) * vx + math.sin(a) * vy,
            -math.sin(a) * vx + math.cos(a) * vy)


def load_settings():
    """Read phase2_settings.json (if it exists) and override the settings above."""
    global KP, KD, VMAX, TARGETS, MAX_FLIGHT_SECONDS
    if not os.path.exists(SETTINGS_FILE):
        return
    try:
        with open(SETTINGS_FILE) as f:
            cfg = json.load(f)
        if "KP" in cfg:
            KP = float(cfg["KP"])
        if "KD" in cfg:
            KD = float(cfg["KD"])
        if "VMAX" in cfg:
            VMAX = float(cfg["VMAX"])
        if "MAX_FLIGHT_SECONDS" in cfg:
            MAX_FLIGHT_SECONDS = float(cfg["MAX_FLIGHT_SECONDS"])
        if "TARGETS" in cfg:
            TARGETS = [(float(t[0]), float(t[1]), float(t[2])) for t in cfg["TARGETS"]]
        print(f"Settings read from {SETTINGS_FILE}")
    except Exception as exc:
        print(f"Could not read {SETTINGS_FILE} ({exc}). Using the values in the script.")
    KP = min(max(KP, 0.0), HARD_KP)
    KD = min(max(KD, 0.0), HARD_KD)
    VMAX = min(max(VMAX, 0.0), HARD_VMAX)


def save_settings():
    cfg = {"KP": round(KP, 2), "KD": round(KD, 2), "VMAX": round(VMAX, 2),
           "MAX_FLIGHT_SECONDS": MAX_FLIGHT_SECONDS,
           "TARGETS": [list(t) for t in TARGETS]}
    with open(SETTINGS_FILE, "w") as f:
        json.dump(cfg, f, indent=2)
    print(f"Settings saved to {SETTINGS_FILE}")


CTRL_WINDOW = "Phase 2 controls"
VIEW_WINDOW = "Phase 2 flight"


def setup_ui(ctx):
    """Slider window (live gains and speed limit) and mouse click on the picture (live target)."""
    cv2.namedWindow(VIEW_WINDOW)
    cv2.namedWindow(CTRL_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CTRL_WINDOW, 520, 140)
    nothing = lambda v: None
    cv2.createTrackbar("KP x10", CTRL_WINDOW, int(round(KP * 10)), int(HARD_KP * 10), nothing)
    cv2.createTrackbar("KD x10", CTRL_WINDOW, int(round(KD * 10)), int(HARD_KD * 10), nothing)
    cv2.createTrackbar("VMAX cm/s", CTRL_WINDOW, int(round(VMAX * 100)), int(HARD_VMAX * 100), nothing)

    def on_mouse(event, mx, my, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and ctx.mtx is not None:
            u, v = mx / ctx.view_scale, my / ctx.view_scale
            pt = np.array([[[u, v]]], dtype=np.float32)
            und = cv2.undistortPoints(pt, ctx.mtx, ctx.dist, P=ctx.mtx).reshape(2)
            fx, fy = lt.pixel_to_floor(ctx.H, und[0], und[1])
            fx = min(max(fx, CLICK_MARGIN), AREA_X - CLICK_MARGIN)
            fy = min(max(fy, CLICK_MARGIN), AREA_Y - CLICK_MARGIN)
            ctx.click = (fx, fy)
            ctx.click_px = (mx, my)
            if ctx.manual_ok:
                print(f"  new target from click: ({fx:.3f}, {fy:.3f}).  Press C to go back to the TARGETS list.")
            else:
                print(f"  click saved: ({fx:.3f}, {fy:.3f}). The drone goes there after take-off and hover.")

    cv2.setMouseCallback(VIEW_WINDOW, on_mouse)


def read_sliders():
    """Copy the slider positions into the settings (limits are enforced by the slider ranges)."""
    global KP, KD, VMAX
    try:
        KP = min(cv2.getTrackbarPos("KP x10", CTRL_WINDOW) / 10.0, HARD_KP)
        KD = min(cv2.getTrackbarPos("KD x10", CTRL_WINDOW) / 10.0, HARD_KD)
        VMAX = min(cv2.getTrackbarPos("VMAX cm/s", CTRL_WINDOW) / 100.0, HARD_VMAX)
    except cv2.error:
        pass


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
        self.cam = None               # last camera position (x, y)
        self.cam_vel = (0.0, 0.0)     # measured speed from the camera
        self.last_seen = 0.0
        self.ekf = (0.0, 0.0, 0.0)    # drone's own position estimate
        self.yaw = 0.0                # drone's own heading estimate, degrees
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
        self.target = (None, None)
        self.cmd = (0.0, 0.0)         # last speed command, floor frame
        self.z_cmd = 0.0
        self.err = None
        self.t_takeoff = 0.0
        self.click = None             # target set by a mouse click (overrides the TARGETS list)
        self.click_px = None
        self.sharp = 0.0              # picture sharpness (Laplacian variance); low = blurred
        self.sharp_seen = 0.0         # sharpness when the marker was last seen
        self.lost_shots = 0
        self.lost_saved = False
        self.manual_ok = False        # clicks are only honoured while flying between targets
        self.view_scale = 1.0


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
    for name in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z", "stateEstimate.yaw"):
        pos.add_variable(name, "float")

    def on_pos(ts, data, conf):
        ctx.ekf = (data["stateEstimate.x"], data["stateEstimate.y"], data["stateEstimate.z"])
        ctx.yaw = data["stateEstimate.yaw"]

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


def marker_is_fresh(ctx):
    return ctx.cam is not None and (time.time() - ctx.last_seen) <= STALE_SECONDS


def process_frame(ctx):
    """Read one camera frame, measure the marker, log, show, read keys."""
    ok, frame = ctx.cap.read()
    if not ok:
        return
    now = time.time()
    ctx.frame_times.append(now)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ctx.brightness = float(gray.mean())
    ctx.sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
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
        # Speed from the camera: change of position since the last time the marker was seen.
        # Only used when that was recent (otherwise the number would be meaningless).
        if ctx.cam is not None and 0.0 < now - ctx.last_seen < 0.2:
            dt = now - ctx.last_seen
            raw = ((x - ctx.cam[0]) / dt, (y - ctx.cam[1]) / dt)
            a = 0.3   # smoothing: 1.0 = no smoothing, smaller = smoother but slower
            ctx.cam_vel = (ctx.cam_vel[0] + a * (raw[0] - ctx.cam_vel[0]),
                           ctx.cam_vel[1] + a * (raw[1] - ctx.cam_vel[1]))
        else:
            ctx.cam_vel = (0.0, 0.0)
        ctx.cam = (x, y)
        ctx.last_seen = now
        ctx.sharp_seen = ctx.sharp
        ctx.lost_saved = False
        if SEND_EXTPOS and ctx.cf is not None:
            ctx.cf.loc.send_extpos([x, y, ctx.ekf[2]])

    visible = found is not None
    if (ctx.flying and not visible and not ctx.lost_saved and ctx.lost_shots < 5
            and now - ctx.last_seen > 0.5):
        ctx.lost_saved = True
        ctx.lost_shots += 1
        snap = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            time.strftime("phase2_lost_%H%M%S.png"))
        cv2.imwrite(snap, frame)
        print(f"  marker not seen for 0.5 s. Picture brightness {ctx.brightness:.0f} (healthy 60 to 180), "
              f"sharpness {ctx.sharp:.0f} (was {ctx.sharp_seen:.0f} when the marker was last seen).")
        print(f"  saved what the camera sees: {snap}")
    if ctx.writer is not None:
        cx, cy = (ctx.cam if visible else (None, None))
        tx, ty = ctx.target
        ctx.writer.writerow([
            f"{now - ctx.t0:.3f}", ctx.phase, int(visible),
            "" if cx is None else f"{cx:.4f}", "" if cy is None else f"{cy:.4f}",
            "" if tx is None else f"{tx:.4f}", "" if ty is None else f"{ty:.4f}",
            f"{ctx.cmd[0]:.3f}", f"{ctx.cmd[1]:.3f}", f"{ctx.z_cmd:.3f}",
            f"{ctx.ekf[0]:.4f}", f"{ctx.ekf[1]:.4f}", f"{ctx.ekf[2]:.4f}", f"{ctx.yaw:.1f}",
            "" if ctx.vbat is None else f"{ctx.vbat:.2f}",
            f"{ctx.brightness:.0f}", f"{ctx.sharp:.0f}"])

    read_sliders()
    scale = 960.0 / frame.shape[1]
    ctx.view_scale = scale
    view = cv2.resize(frame, None, fx=scale, fy=scale)
    if found is not None:
        poly = (np.array(found).reshape(4, 2) * scale).astype(np.int32)
        cv2.polylines(view, [poly], True, (0, 255, 0), 2)
    cam_txt = "camera: marker lost" if not visible else f"camera x {ctx.cam[0]:+.3f} y {ctx.cam[1]:+.3f}"
    cv2.putText(view, f"{ctx.phase}   {camera_rate(ctx):.0f} fps", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(view, cam_txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 0) if visible else (0, 0, 255), 2)
    if ctx.target[0] is not None:
        cv2.putText(view, f"target x {ctx.target[0]:+.3f} y {ctx.target[1]:+.3f}   "
                          f"command vx {ctx.cmd[0]:+.2f} vy {ctx.cmd[1]:+.2f} m/s",
                    (10, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    if ctx.cf is not None:
        cv2.putText(view, f"drone  z {ctx.ekf[2]:.2f}  yaw {ctx.yaw:+.0f}",
                    (10, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(view, "ESC = land    SPACE = EMERGENCY STOP", (10, view.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    if ctx.click_px is not None and ctx.click is not None:
        cv2.circle(view, ctx.click_px, 8, (0, 165, 255), 2)
    cv2.putText(view, f"KP {KP:.1f}  KD {KD:.1f}  VMAX {VMAX:.2f} m/s   click = new target, C = back to list, S = save settings",
                (10, view.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.imshow(VIEW_WINDOW, view)

    key = cv2.waitKey(1) & 0xFF
    if key == 27 and ctx.land_request is None:
        ctx.land_request = "ESC pressed"
    if key == 32:
        ctx.emergency = True
    if key in (ord("c"), ord("C")):
        ctx.click = None
        ctx.click_px = None
    if key in (ord("s"), ord("S")):
        save_settings()


def check_safety(ctx):
    if ctx.emergency:
        raise EmergencyStop()
    if ctx.land_request:
        raise AbortFlight(ctx.land_request)
    if not ctx.flying:
        return
    if time.time() - ctx.last_seen > LOST_SECONDS:
        raise AbortFlight(f"marker lost for more than {LOST_SECONDS} s")
    if time.time() - ctx.t_takeoff > MAX_FLIGHT_SECONDS:
        raise AbortFlight(f"flight longer than {MAX_FLIGHT_SECONDS:.0f} s")
    if ctx.cam is not None:
        x, y = ctx.cam
        if (x < -AREA_MARGIN or x > AREA_X + AREA_MARGIN or
                y < -AREA_MARGIN or y > AREA_Y + AREA_MARGIN):
            raise AbortFlight(f"marker outside the flight area (x {x:.2f}, y {y:.2f})")
    if ctx.vbat is not None and ctx.vbat < MIN_VBAT_FLIGHT:
        raise AbortFlight(f"battery low ({ctx.vbat:.2f} V)")


def step(ctx, tx, ty, z, safety=True):
    """One control step: read a camera frame, decide the speed, send it to the drone."""
    process_frame(ctx)
    if ctx.click is not None and ctx.manual_ok:
        tx, ty = ctx.click          # a clicked target replaces the list target
    ctx.target = (tx, ty)
    if ctx.emergency:
        raise EmergencyStop()
    if safety:
        check_safety(ctx)

    if marker_is_fresh(ctx):
        vx, vy = controller(ctx, tx, ty)
        vx, vy = limit_speed(vx, vy, min(VMAX, HARD_VMAX))   # last word, even if controller() is edited
        ctx.err = math.hypot(tx - ctx.cam[0], ty - ctx.cam[1])
    else:
        vx, vy = 0.0, 0.0        # marker not seen just now: hover instead of guessing
        ctx.err = None
    ctx.cmd = (vx, vy)
    ctx.z_cmd = z
    if ctx.cf is not None:
        bx, by = floor_to_body(vx, vy, ctx.yaw + YAW_OFFSET_DEG)
        ctx.cf.commander.send_hover_setpoint(bx, by, 0.0, z)


def hold_at(ctx, tx, ty, z, seconds, phase, safety=True):
    ctx.phase = phase
    end = time.time() + seconds
    while time.time() < end:
        step(ctx, tx, ty, z, safety)


def ramp_height(ctx, tx, ty, z_from, z_to, rate, phase, safety=True):
    """Change the height target steadily from z_from to z_to, holding position (tx, ty)."""
    ctx.phase = phase
    t_start = time.time()
    duration = abs(z_to - z_from) / rate
    while True:
        frac = min(1.0, (time.time() - t_start) / duration) if duration > 0 else 1.0
        z = z_from + (z_to - z_from) * frac
        step(ctx, tx, ty, z, safety)
        if frac >= 1.0:
            return


def go_to(ctx, tx, ty, hold, label):
    """Fly to (tx, ty) at HOVER_HEIGHT, stay there for `hold` seconds."""
    ctx.phase = f"moving to {label}"
    t_start = time.time()
    arrived = None
    ctx.manual_ok = True
    try:
        _go_to_loop(ctx, tx, ty, hold, label, t_start, arrived)
    finally:
        ctx.manual_ok = False


def _go_to_loop(ctx, tx, ty, hold, label, t_start, arrived):
    while True:
        step(ctx, tx, ty, HOVER_HEIGHT)
        now = time.time()
        if ctx.click is not None:        # manual target active: the list waits
            t_start, arrived = now, None
            ctx.phase = "manual target (C = resume list)"
            continue
        if arrived is None:
            if ctx.err is not None and ctx.err < ARRIVE_TOL:
                arrived = now
                ctx.phase = f"holding {label}"
                print(f"  reached {label} after {now - t_start:.1f} s")
            elif now - t_start > WAYPOINT_TIMEOUT:
                raise AbortFlight(f"{label} not reached in {WAYPOINT_TIMEOUT:.0f} s")
        elif now - arrived >= hold:
            return


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
                snap = os.path.join(os.path.dirname(os.path.abspath(__file__)), "phase2_no_marker.png")
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


def dry_run(ctx):
    """No radio. Shows the speed the controller would command while you slide the marker."""
    print(f"DRY RUN for {DRY_SECONDS:.0f} s. Slide the marker by hand. No radio, no flight.")
    print("The target is the first entry of TARGETS. Check that the command points TOWARDS it.")
    tx, ty = TARGETS[0][0], TARGETS[0][1]
    end = time.time() + DRY_SECONDS
    last_print = 0.0
    ctx.phase = "dry run"
    ctx.manual_ok = True
    while time.time() < end:
        step(ctx, tx, ty, HOVER_HEIGHT, safety=False)
        if ctx.land_request:
            break
        if time.time() - last_print > 1.0 and marker_is_fresh(ctx):
            last_print = time.time()
            ex, ey = ctx.target[0] - ctx.cam[0], ctx.target[1] - ctx.cam[1]   # target in use (list or click)
            words = []
            if abs(ctx.cmd[0]) > 0.01:
                words.append("+X" if ctx.cmd[0] > 0 else "-X")
            if abs(ctx.cmd[1]) > 0.01:
                words.append("+Y" if ctx.cmd[1] > 0 else "-Y")
            print(f"  marker ({ctx.cam[0]:+.3f}, {ctx.cam[1]:+.3f})  error ({ex:+.3f}, {ey:+.3f})  "
                  f"command ({ctx.cmd[0]:+.2f}, {ctx.cmd[1]:+.2f}) m/s  -> {' '.join(words) or 'stay'}")
    print("Dry run finished.")


def main():
    dry = "--dry" in sys.argv

    for tx, ty, _ in TARGETS:
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
    load_settings()

    ctx.cap = vp.open_camera()
    if ctx.cap is None:
        return

    setup_ui(ctx)
    log_name = time.strftime("flight2_log_%Y%m%d_%H%M%S.csv")
    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), log_name)
    log_file = open(log_path, "w", newline="")
    ctx.writer = csv.writer(log_file)
    ctx.writer.writerow(["t_seconds", "phase", "cam_visible", "cam_x_m", "cam_y_m",
                         "target_x_m", "target_y_m", "cmd_vx", "cmd_vy", "cmd_z_m",
                         "drone_x_m", "drone_y_m", "drone_z_m", "drone_yaw_deg", "vbat_v",
                         "brightness", "sharpness"])
    print(f"Logging to {log_path}")

    if dry:
        try:
            dry_run(ctx)
        except (EmergencyStop, KeyboardInterrupt):
            print("Stopped.")
        finally:
            ctx.cap.release()
            cv2.destroyAllWindows()
            log_file.close()
            print(f"Log saved: {log_path}")
        return

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
    in_air = False
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

        # Kalman estimator. The high-level commander is switched OFF: from now on this
        # program sends the setpoints.
        ctx.cf.param.set_value("stabilizer.estimator", "2")
        ctx.cf.param.set_value("commander.enHighLevel", "0")
        if SEND_EXTPOS:
            try:
                ctx.cf.param.set_value("locSrv.extPosStdDev", str(EXT_POS_STD))
            except Exception as exc:
                print(f"Warning: could not set locSrv.extPosStdDev ({exc}).")

        print("Put the marker (drone) in the flight area, nose along +X. Waiting for the marker ...")
        if not wait_marker_stable(ctx):
            print("Marker not seen steadily. Not flying.")
            return
        if not inside_area(*ctx.cam):
            print(f"Marker is at x {ctx.cam[0]:.2f}, y {ctx.cam[1]:.2f}, outside the flight area. Not flying.")
            return
        start_xy = ctx.cam

        # Reset the drone's estimate, as the cflib MotionCommander does.
        ctx.phase = "estimator reset"
        ctx.cf.param.set_value("kalman.resetEstimation", "1")
        end = time.time() + 0.2
        while time.time() < end:
            process_frame(ctx)
        ctx.cf.param.set_value("kalman.resetEstimation", "0")
        ctx.var_hist.clear()

        if SEND_EXTPOS:
            print("Waiting for the drone's estimate to settle on the camera position ...")
            ctx.phase = "waiting for estimator"
            end = time.time() + 15
            settled = False
            while time.time() < end:
                process_frame(ctx)
                if ctx.emergency or ctx.land_request:
                    raise AbortFlight("cancelled")
                if estimator_settled(ctx):
                    settled = True
                    break
            if not settled:
                print("The estimate did not settle in 15 s. Not flying. See the log file.")
                return
            dx, dy = ctx.ekf[0] - ctx.cam[0], ctx.ekf[1] - ctx.cam[1]
            print(f"Settled. Drone estimate minus camera: dx {dx:+.3f} m, dy {dy:+.3f} m")
            if abs(dx) > MAX_START_DIFF or abs(dy) > MAX_START_DIFF:
                print(f"They differ by more than {MAX_START_DIFF} m. Not flying.")
                return
        else:
            ctx.phase = "estimator warm-up"
            end = time.time() + 2.0
            while time.time() < end:
                process_frame(ctx)
                if ctx.emergency or ctx.land_request:
                    raise AbortFlight("cancelled")

        rate = camera_rate(ctx)
        print(f"Camera rate: {rate:.1f} frames per second")
        if rate < MIN_CAMERA_HZ:
            print(f"The camera gives fewer than {MIN_CAMERA_HZ:.0f} frames per second. The PC loop runs at the")
            print("camera rate, so this is too slow. Fix the camera settings (camera_setup.py). Not flying.")
            return

        # ---- flight ----
        try:
            ctx.cf.supervisor.send_arming_request(True)
        except Exception:
            pass
        print("Taking off in 3 seconds. SPACE = emergency stop, ESC = land.")
        ctx.phase = "countdown"
        end = time.time() + 3.0
        while time.time() < end:
            process_frame(ctx)
            if ctx.emergency:
                raise EmergencyStop()
            if ctx.land_request:
                raise AbortFlight(ctx.land_request)

        ctx.cf.commander.send_setpoint(0, 0, 0, 0)     # unlocks the thrust protection
        time.sleep(0.1)
        ctx.t_takeoff = time.time()
        ctx.flying = True
        in_air = True
        try:
            ramp_height(ctx, *start_xy, 0.0, HOVER_HEIGHT, TAKEOFF_RATE, "take off")
            hold_at(ctx, *start_xy, HOVER_HEIGHT, 2.0, "hover")
            for i, (tx, ty, hold) in enumerate(TARGETS, start=1):
                print(f"Target {i}: ({tx:.3f}, {ty:.3f}), hold {hold:.1f} s")
                go_to(ctx, tx, ty, hold, f"target {i}")
            print("Returning to the start point.")
            go_to(ctx, start_xy[0], start_xy[1], 1.0, "start point")
            print("Landing.")
            ctx.flying = False
            ramp_height(ctx, *start_xy, HOVER_HEIGHT, 0.0, LAND_RATE, "landing", safety=False)
        except AbortFlight as exc:
            print(f"LANDING NOW: {exc}")
            ctx.flying = False
            tx, ty = ctx.cam if marker_is_fresh(ctx) else start_xy
            z_now = min(HOVER_HEIGHT, max(0.0, ctx.ekf[2]))
            ramp_height(ctx, tx, ty, z_now, 0.0, LAND_RATE, "landing (abort)", safety=False)
        ctx.cf.commander.send_stop_setpoint()
        ctx.cf.commander.send_notify_setpoint_stop()
        in_air = False
        print("Landed.")

    except (EmergencyStop, KeyboardInterrupt):
        print("EMERGENCY STOP")
        in_air = False
        try:
            ctx.cf.supervisor.send_emergency_stop()
        except Exception:
            ctx.cf.loc.send_emergency_stop()
    except AbortFlight as exc:
        print(f"Stopped before take-off: {exc}")
    finally:
        if in_air:
            # something unexpected went wrong in the air: motors off is the last resort
            try:
                ctx.cf.commander.send_stop_setpoint()
            except Exception:
                pass
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
