"""
flow_hover_test.py - Step 3: first tiny flight, Flow Deck only (no camera yet).

What it does:
  1) Connects to the Crazyflie
  2) Refuses to fly (and says why) if the Flow Deck is missing or the battery is low
  3) Takes off to HOVER_HEIGHT, hovers HOVER_SECONDS, lands

SAFETY
  - Fly over a textured floor (a printed page or a patterned mat). The Flow Deck
    cannot track a plain shiny or white floor.
  - Clear about 1.5 m around the drone. Keep hands and face away from the props.
  - Press Ctrl+C in this window at any time: it cuts the motors immediately.
    The drone will drop, so keep the hover height low.

Before running:
  - Close cfclient (only one program can use the Crazyradio).
  - Charge the battery (about 4.0 V or more).
  - Place the drone flat on the floor, then power it on.

Run:
    python flow_hover_test.py
"""

import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

URI = "radio://0/80/2M"   # the address cfclient connected to; add /E7E7E7E7E7 if needed
HOVER_HEIGHT = 0.3        # metres
HOVER_SECONDS = 3.0
MIN_VBAT = 3.8            # volts; below this the script will not fly


def read_battery(scf):
    log_conf = LogConfig(name="Battery", period_in_ms=100)
    log_conf.add_variable("pm.vbat", "float")
    with SyncLogger(scf, log_conf) as logger:
        for entry in logger:
            return entry[1]["pm.vbat"]


def flow_deck_attached(scf):
    for name in ("deck.bcFlow2", "deck.bcFlow"):
        try:
            if str(scf.cf.param.get_value(name)).strip() == "1":
                return name
        except Exception:
            continue
    return None


def main():
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")
    try:
        scf_ctx = SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache"))
        scf = scf_ctx.__enter__()
    except Exception as exc:
        print(f"Connection failed: {exc}")
        print("Is cfclient closed? Is the Crazyflie powered on?")
        sys.exit(1)

    try:
        vbat = read_battery(scf)
        print(f"Battery: {vbat:.2f} V")
        if vbat < MIN_VBAT:
            print(f"Battery is below {MIN_VBAT} V. Charge it first. Not flying.")
            return

        deck = flow_deck_attached(scf)
        if not deck:
            print("Flow Deck NOT detected. Press it fully onto the underside pins,")
            print("power-cycle the Crazyflie, and run this again. Not flying.")
            return
        print(f"Flow Deck detected ({deck}).")

        print("Taking off in 3 seconds. Ctrl+C = emergency stop.")
        time.sleep(3)

        mc = MotionCommander(scf, default_height=HOVER_HEIGHT)
        try:
            mc.take_off()
            print(f"Hovering at {HOVER_HEIGHT} m for {HOVER_SECONDS} s ...")
            time.sleep(HOVER_SECONDS)
            mc.land()
            print("Landed.")
        except KeyboardInterrupt:
            print("EMERGENCY STOP")
            scf.cf.loc.send_emergency_stop()
    finally:
        scf_ctx.__exit__(None, None, None)


if __name__ == "__main__":
    main()
