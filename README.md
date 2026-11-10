# Crazyflie Vision-Based Control Learning Environment

An overhead USB camera watches an ArUco marker on a Crazyflie 2.1+ nano drone. The PC turns the marker position into floor coordinates, shows and logs it, and flies the drone over the Crazyradio: take-off, move to a position, hold, land, and emergency stop. Everything is plain Python, written to be read and changed by students and researchers.

Developed by **Tegara Corporation**. Author: Lahiru Dhananjaya, Software Architect (all research and development).

This is an independent project. Crazyflie, Flow Deck and Crazyradio are products of Bitcraze AB, and this project is not affiliated with or endorsed by Bitcraze.

## Two ways to fly

| | Phase 1 (`phase1_flight.py`) | Phase 2 (`phase2_flight.py`) |
|---|---|---|
| Idea | The PC sends the camera position to the drone. The drone's own firmware fuses it with the Flow Deck and flies the moves. | The PC runs the position controller itself and sends speed commands. This is the part you edit. |
| Use it for | A stable, demonstrable baseline | Learning and research on control laws |
| Where to change things | `TARGETS` and the settings at the top | the function `controller()`, `TARGETS`, sliders, `phase2_settings.json` |

Both share the same camera front end: lens correction, ArUco detection and the pixel-to-floor map.

## What you need

The software is free. The hardware is not included in this repository.

The Crazyflie drone set can be bought from UNIPOS: https://www.unipos.net/products/bitcraze-crazyflie/

- Crazyflie 2.1+ with a Flow Deck v2 and a Crazyradio (the drone, deck and dongle)
- A USB camera that can give 640x480 at 30 fps and allows manual exposure, on a rigid mount pointing straight down
- A Windows 10/11 or Linux PC with Python 3.10 or newer and two free USB ports
- A textured floor or mat about 1 m by 0.6 m or more, even light, and a safety net or enclosure
- A printer (A4, 100% scale) for the ArUco marker and the checkerboard in `print/`

## Quick start

Windows (PowerShell):

```powershell
python -m venv venv
venv\Scripts\Activate.ps1
pip install -r requirements.txt
python connect_test.py
```

Linux:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 connect_test.py
```

The Crazyradio driver, the camera setup, the lens calibration, the floor map and the first flights are explained step by step in the guide. Start with section 3.

## Documentation

- [Setup and Operation Guide (English)](docs/crazyflie-implementation-guide.pdf), also as [Markdown](docs/crazyflie-implementation-guide.md)
- [セットアップと操作ガイド(日本語)](docs/crazyflie-implementation-guide_ja.pdf)、[Markdown版](docs/crazyflie-implementation-guide_ja.md)

Section 19 of the guide lists every command for Windows and Linux.

## Files

| File | Purpose |
|---|---|
| `connect_test.py`, `flow_hover_test.py` | Check the radio link and a simple hover without the camera |
| `camera_setup.py`, `camera_fps_test.py`, `camera_modes.py` | Choose camera settings and check the frame rate |
| `vision_pipeline_tool.py` | Lens calibration, floor map, position check |
| `live_tracker.py` | Live position display |
| `phase1_flight.py` | Phase 1 flight, with a no-flight `--check` mode |
| `phase2_flight.py` | Phase 2 flight, with a no-radio `--dry` mode |
| `*.example.json` | Example settings. Copy and edit them |
| `print/` | Marker and checkerboard to print |

## What has been tested

Measured on one lab setup (Windows, Python 3.14, cflib 0.1.34, camera about 1.2 m above a 0.93 m by 0.59 m floor, hover height 0.3 m):

- Phase 1: hold within about 1 cm over a 5 s hold.
- Phase 2: hold at 2.1 cm mean and 4.1 cm worst case over a 5 s hold. The marker was seen in 99% of the pictures during the flight.
- Automatic landing was seen for real on low battery and on a lost marker.

Not yet tested, so please treat them as open: the SPACE emergency stop in a real flight, Linux, and other cameras. In one flight the marker was lost at the far side of the area, most likely because the picture was too dark. Good, even light matters. Your results will differ with your camera, light and floor, so measure your own setup (guide, section 14).

## Safety

This is a flying robot with spinning propellers. Keep people clear, use a net or enclosure, fly low at first, and keep a hand on the keyboard: SPACE or Ctrl+C stops the motors, ESC lands. Check the emergency stop low and over a soft surface before any demonstration. Use a fully charged battery. The scripts refuse to fly on a low battery and land by themselves when the marker is lost, the battery is low, or the drone leaves the area, but these are aids and not a replacement for care.

## Licence

GNU General Public License, version 3 or later. See [LICENSE](LICENSE). The Crazyflie Python library (`cflib`) that this code uses is also licensed under GPL-3.0.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).
