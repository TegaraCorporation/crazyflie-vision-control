"""
connect_test.py - Step 2: confirm the PC can see and talk to the Crazyflie.

Does four things, in order:
  1) Scans for any Crazyflie within radio range
  2) Connects to the first one found (or to DEFAULT_URI if the scan finds none)
  3) Reads and prints the battery voltage
  4) Reports whether the Flow Deck v2 is detected

The drone does NOT fly in this script. Motors stay off.

Before running:
  - Close cfclient (the Bitcraze GUI) if it is open. Only one program can use
    the Crazyradio at a time.
  - Power on the Crazyflie and leave it still on a flat surface.

Run:
    python connect_test.py
"""

import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# Factory default address. Only used if the scan below finds nothing.
DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"


def find_crazyflie():
    print("Scanning for Crazyflies (takes a few seconds)...")
    found = cflib.crtp.scan_interfaces()
    if not found:
        print("  Nothing found by the scan.")
        return None
    for uri, description in found:
        print(f"  Found: {uri}  {description}")
    return found[0][0]


def read_battery(scf):
    log_conf = LogConfig(name="Battery", period_in_ms=100)
    log_conf.add_variable("pm.vbat", "float")
    with SyncLogger(scf, log_conf) as logger:
        for entry in logger:
            return entry[1]["pm.vbat"]


def check_flow_deck(scf):
    # Flow Deck v2 reports as deck.bcFlow2 (the older v1 is deck.bcFlow).
    for name in ("deck.bcFlow2", "deck.bcFlow"):
        try:
            value = scf.cf.param.get_value(name)
        except Exception:
            continue
        if str(value).strip() == "1":
            return name
    return None


def main():
    cflib.crtp.init_drivers()

    uri = find_crazyflie()
    if uri is None:
        print(f"Trying the default address instead: {DEFAULT_URI}")
        uri = DEFAULT_URI

    print(f"\nConnecting to {uri} ...")
    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
            print("Connected.")

            vbat = read_battery(scf)
            print(f"Battery voltage: {vbat:.2f} V  (about 4.1 V or more is a full charge)")

            deck = check_flow_deck(scf)
            if deck:
                print(f"Flow Deck detected ({deck}).")
            else:
                print("Flow Deck NOT detected. Check that it is pressed fully onto the")
                print("underside pins, then power-cycle the Crazyflie and run this again.")
    except Exception as exc:
        print(f"\nConnection failed: {exc}")
        print("Likely causes: Crazyradio not recognized (driver), Crazyflie powered off,")
        print("cfclient still open, or the drone is on a non-default radio address.")
        sys.exit(1)


if __name__ == "__main__":
    main()
