# Perceptual Drift — Audience‑Controlled FPV Installation

Perceptual Drift is a participatory drone‑based installation where **audience motion** modulates **FPV drone** behavior and **live video processing**. It centers shared authorship: many bodies → probabilistic control → drones that “drift” perceptually. The installation leans on **DIY‑friendly stacks**: Betaflight micro‑drones, Raspberry Pi video processing (GStreamer/OBS), OpenFrameworks/Processing for gesture tracking, and an OSC→MSP control bridge. Optional **Teensy DSP/LED** layers extend visual/sonic feedback. Think of it as an unholy jam session between [Betaflight](https://betaflight.com/), [Processing](https://processing.org/), [GStreamer](https://gstreamer.freedesktop.org/), and [python-osc](https://pypi.org/project/python-osc/) with a splash of [Mozzi](https://sensorium.github.io/Mozzi/) and [CrazySwarm2](https://imrclab.github.io/crazyflie-clients-python/).

> **No breadcrumbs? No problem.** This README tries to be the missing field manual. Every subsystem below includes links, inspiration, and what to Google when things melt down.

---

## Why this repo
- **Rapid pilot → exhibition‑scale**: start with 1 drone, 1 projector; scale to a swarm and multi‑screen projections.
- **Open & reproducible**: off‑the‑shelf micro‑FPV, open libraries, simple wiring.
- **Safety‑first**: indoor netted cage, conservative flight limits, visible consent gates.

---

## System Overview
1. **Gesture tracking** *(Processing / OpenFrameworks)*
   - Webcam or depth cam feeds into a Processing sketch that estimates coarse crowd motion.
   - We ship normalized floats over OSC using [oscP5](https://www.sojamo.de/libraries/oscP5/) & [NetP5](https://www.sojamo.de/libraries/netP5/).
   - Inspirations: [OfxCv optical flow demos](https://github.com/kylemcdonald/ofxCv) and [LASER Tag (Graffiti Research Lab)](http://graffitiresearchlab.com/blog/projects/laser-tag/).
2. **Control bridge (Pi/PC)** *(Python)*
   - [`osc_msp_bridge.py`](software/control-bridge/osc_msp_bridge.py) converts the OSC data into Betaflight RC microseconds via the [Minimal Serial Protocol (MSP)](https://github.com/betaflight/betaflight/wiki/MSP). Think “software radio transmitter.”
   - Uses [pyserial](https://pyserial.readthedocs.io/en/latest/), [python-osc](https://pypi.org/project/python-osc/), and config from `config/mapping.yaml`.
   - Modelled after [DJI FPVgestures](https://github.com/whoisandrewd/fpv-gestures) and [Red Paper Heart’s drone installations](https://redpaperheart.com/).
3. **FPV video pipeline** *(GStreamer / OBS)*
   - Analog FPV feed → VRX → USB capture card → [GStreamer](https://gstreamer.freedesktop.org/documentation/) pipeline or [OBS Studio](https://obsproject.com/) scenes.
   - Presets in [`software/video-pipeline/gst_launch.sh`](software/video-pipeline/gst_launch.sh) cover low-latency monitoring and delayed glitch feedback.
4. **Lights & DSP (optional but rad)**
   - [Teensy 4.x](https://www.pjrc.com/teensy/) drives WS2812 strips via [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) and audio feedback via [Mozzi](https://sensorium.github.io/Mozzi/).
   - Serial protocol lets the control bridge or Ableton-style rigs paint colors/intensity.
5. **Swarm experiments (stretch goal)**
   - [`software/swarm`](software/swarm) sketches how to extend the control patterns into [CrazySwarm2](https://crazyswarm.readthedocs.io/) + ROS 2 for multiple quads.

See `docs/diagrams/system-overview.md` for mermaid diagrams that stitch the above together.

---

## Repo Layout
```
.
├─ config/                      # YAML/JSON mappings and presets
├─ docs/                        # Diagrams, checklists, assumption ledger, UX mapping
├─ firmware/                    # Teensy LED (and optional DSP) firmware via PlatformIO
├─ hardware/                    # Cage, LED wiring, parts lists
├─ scripts/                     # Utility scripts (logs, calibration, recorders)
└─ software/
   ├─ control-bridge/           # OSC → MSP bridge (Python)
   ├─ gesture-tracking/         # Processing/OpenFrameworks sketches
   └─ video-pipeline/           # GStreamer/OBS scenes, launchers
```

---

## Quickstart (Pilot: 1 drone, 1 projector)
1. **Hardware**
   - Net a 2×2 m cage. BetaFPV Cetus Pro or any 65–75 mm whoop running Betaflight 4.x works.
   - USB power a VRX + capture dongle (classic EasyCAP or UVC cards). Mount a short WS2812 strip (8–12 px) to the quad if you want glow feedback.
   - Borrow ideas from [NOIR Drone Shows’ safety setups](https://noirdrones.com/) and [MIT’s FlyByWire cage](https://aerobotics.mit.edu/).
2. **Tracking rig**
   - Install [Processing 4](https://processing.org/download) plus the `video`, `oscP5`, and `netP5` libraries.
   - Run `software/gesture-tracking/processing/PerceptualDrift_Tracker`. Aim any 720p-ish webcam at the audience area and tweak the `threshold` constant for your lighting.
3. **Control bridge**
   - `pip install -r software/control-bridge/requirements.txt` (python-osc + pyserial + pyyaml).
   - Plug the drone’s flight controller in over USB, then run `python3 osc_msp_bridge.py --serial /dev/ttyUSB0`.
   - Check out [Betaflight’s MSP docs](https://github.com/betaflight/betaflight/wiki/MSP) if you want to push extra AUX channels.
4. **Video pipeline**
   - `sudo apt install gstreamer1.0-tools` or use OBS.
   - Run `./software/video-pipeline/gst_launch.sh clean_low_latency` for tight monitoring or `delayed_glitch` for delayed projection loops. Adapted from [Scanlines’ GStreamer recipes](https://scanlines.xyz/t/gstreamer-recipes/1414).
5. **Safety dance**
- Follow `docs/checklists/safety_checklist.md` before every session.
- Keep the `consent` channel low until the facilitator explicitly flips it.
- Tape a hardware kill switch (BetaFPV LiteRadio or Jumper T-Lite) to your wrist. Cages help, but redundancy keeps faces intact.
- Skim `docs/experience/README.md` for the meta UX choreography so operator + crowd stay in sync.

---

## Config
- [`config/mapping.yaml`](config/mapping.yaml) provides tunable curves for **altitude**, **lateral drift**, **yaw bias**, **LED color**, and **glitch intensity**.
- [`config/video-presets.json`](config/video-presets.json) defines named GStreamer pipeline presets for OBS/GStreamer hybrids.
- [`hardware/`](hardware) includes a fleshed-out [hardware BOM](hardware/README.md), wiring, and net rig notes based on [Drone Cage DIY](https://hackaday.io/project/19102-drone-safety-cage) write-ups.

---

## Ethics & Consent
- Visible “consent gate”: drones/video remain idle until participants opt‑in (button/gesture). Inspired by [Studio Olafur Eliasson’s consent signage](https://olafureliasson.net/).
- Data minimization: no face storage, no cloud calls. See `docs/PRIVACY_ETHICS.md` and `docs/ASSUMPTION_LEDGER.md` for what we log (spoiler: almost nothing).
- Follow [ADA walkway clearance guidelines](https://www.access-board.gov/ada/) when placing cages and screens so wheelchairs aren’t boxed out.

---

## Deep-dive links & inspo
- **Control theory crash course**: [UAVTech’s Betaflight PID bible](https://www.uavtech.com/pids) for tuning feel vs. safety.
- **Interactive art ancestors**: [Daniel Rozin’s mirror series](https://rozinmuseum.com/mirrors/) and [Rafael Lozano-Hemmer’s “Pulse”](https://www.lozano-hemmer.com/pulse_room.php) for crowd-controlled ambiance.
- **Latency budgeting**: [Bitcraze’s article on Vicon + Crazyflie latency](https://www.bitcraze.io/2020/05/latency-in-position-control/) — relevant when you scale to swarms.
- **Network hygiene**: [NDI vs. SDI vs. analog FPV comparison](https://www.ptzoptics.com/guides/ndi/) if you swap video transports.
- **Show control patterns**: [Ableton Link OSC bridges](https://github.com/ideoforms/ableton-link) and [TouchDesigner OSC out](https://docs.derivative.ca/OSC_Out_DAT) make for easy interoperability.

---

## Licensing
- Code: MIT. Docs: CC‑BY 4.0. See `LICENSE`.
