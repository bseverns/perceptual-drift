# Perceptual Drift — Audience‑Controlled FPV Installation

| Status board | Signal |
| --- | --- |
| CI (full stack, all envs) | [![CI](https://github.com/bseverns/perceptual-drift/actions/workflows/ci.yml/badge.svg)](https://github.com/bseverns/perceptual-drift/actions/workflows/ci.yml) |

Perceptual Drift is a participatory drone‑based installation where **audience motion** modulates **FPV drone** behavior and **live video processing**. It centers shared authorship: many bodies → probabilistic control → drones that “drift” perceptually. The installation leans on **DIY‑friendly stacks**: Betaflight micro‑drones, Raspberry Pi video processing (GStreamer/OBS), Processing for gesture tracking, and an OSC→MSP control bridge. Optional **Teensy DSP/LED** layers extend visual/sonic feedback. Think of it as an unholy jam session between [Betaflight](https://betaflight.com/), [Processing](https://processing.org/), [GStreamer](https://gstreamer.freedesktop.org/), and [python-osc](https://pypi.org/project/python-osc/) with a splash of [Mozzi](https://sensorium.github.io/Mozzi/) and [CrazySwarm2](https://crazyswarm.readthedocs.io/).

> **No breadcrumbs? No problem.** This README tries to be the "missing field manual" for these tech stacks. Every subsystem below includes links, inspiration, and what to Google when things melt down so you can find your way home.

---

## How to tour this notebook (a.k.a. your flight school syllabus)
Want the full syllabus with reasons to care for every doc? The [touring guide lives in `docs/quickstart.md`](docs/quickstart.md#how-to-tour-this-notebook-aka-your-flight-school-syllabus) and calls out where each playbook and diagram fits in the rehearsal arc.

---

## Platform-aware drift lab (Jetson + laptop + Pi)

Ready to treat a Jetson Orin Nano as a node in the drift constellation without breaking laptop dev flow? Use the new platform profiles + scripts as your rails:

- **Pick a platform profile**
  - Laptop/dev box: [`config/platform_desktop.yaml`](config/platform_desktop.yaml) (OpenCV capture, preview window on, CPU-first).
  - Jetson Orin Nano: [`config/platform_jetson_orin_nano.yaml`](config/platform_jetson_orin_nano.yaml) (GStreamer CSI/USB pipeline placeholder, window off by default, GPU encouraged). Tweak the `gstreamer_pipeline` string to match your camera sensor/port.
  - Raspberry Pi bridge/camera node: [`config/platform_pi.yaml`](config/platform_pi.yaml) (libcamerasrc pipeline baked in, window off, OSC pointed at localhost so the bridge + gesture tracker can live on the same box). Flip `backend` to `opencv` if you’re debugging with `cv2.VideoCapture` instead of GStreamer.
- **Wire your mappings**
  - OSC: [`config/mappings/osc.yaml`](config/mappings/osc.yaml) names the drift metrics → OSC addresses. Drop it into TouchDesigner/Max or any OSC monitor to watch the numbers wiggle.
  - MIDI: [`config/mappings/midi.yaml`](config/mappings/midi.yaml) mirrors the same parameters on CCs so you can slam sliders or route into Ableton rigs.
- **Bootstrap a Jetson**
  - Run [`scripts/setup_jetson.sh`](scripts/setup_jetson.sh) on a fresh flash. It installs system deps, spins a venv at `~/venvs/perceptual-drift`, and pulls this repo to `~/code/perceptual-drift`. Edit the Git remote inside if you’re not `git@github.com:<you>/perceptual-drift.git`.
  - Sanity-check the camera bus with [`scripts/check_sensors.py`](scripts/check_sensors.py). It’ll yell if indices 0–5 are dead.
  - Time the pipeline with [`scripts/profile_pipeline.py`](scripts/profile_pipeline.py --config config/platform_jetson_orin_nano.yaml). It grabs frames, computes a tiny drift metric, and prints per-stage timing so you know if GStreamer/CUDA are doing their job.
- **Drop in a Pi control bridge (copy/paste setup)**
  - Run [`scripts/setup_pi.sh`](scripts/setup_pi.sh) on Raspberry Pi OS. It installs `python-osc`/`pyserial`, GStreamer bits, and writes a `systemd` unit that keeps [`osc_msp_bridge.py`](software/control-bridge/osc_msp_bridge.py) alive against `/dev/ttyACM0` (swap to `/dev/ttyUSB0` if your FC shows up there).
  - The unit defaults to `--hz 50` with `config/mapping.yaml`; edit `/etc/systemd/system/osc-msp-bridge.service` then `sudo systemctl daemon-reload && sudo systemctl restart osc-msp-bridge` whenever you change ports or mapping files.
  - Serial fallback / heartbeat loop: pass `--dry-run` to `osc_msp_bridge.py` when the flight controller is unplugged; the dry-run serial stub logs every MSP frame so you can rehearse OSC mappings without arming props. When `/pd/consent` drops to `0`, the bridge auto-blasts neutral RC values so Betaflight still sees a heartbeat while the quad stays napping — a safe rehearsal loop layered on top of radio failsafes.
- **Run a headless-ish hello**
  - `python3 examples/jetson_hello_camera.py --config config/platform_jetson_orin_nano.yaml`
  - Watch the console FPS/drift log; if `outputs.window.enabled` is `true` you also get an OpenCV window with a pulsing circle sized to drift. Smash `q` to bail.
- **Field notes as living log**
  - Jetson runs: log successes + weirdness in [`notes/jetson_field_notes.md`](notes/jetson_field_notes.md).
  - Laptop dev sessions: mirror the habit in [`notes/lap_field_notes.md`](notes/lap_field_notes.md) so future you remembers which USB hub or lighting tweak fixed things.
  - Pi bridge bring-up: jot what worked in a local notebook or inline comments near [`scripts/setup_pi.sh`](scripts/setup_pi.sh) so the next swap of SD cards is painless.

This is half studio notebook, half teaching guide: copy/paste the configs straight into workshops, then scribble what broke. Novelty < utility.

---

## Why this repo
- **Rapid pilot → exhibition‑scale**: start with 1 drone, 1 projector; scale to a swarm and multi‑screen projections.
- **Open & reproducible**: off‑the‑shelf micro‑FPV, open libraries, simple wiring.
- **Safety‑first**: indoor netted cage, conservative flight limits, visible consent gates.

---

## System Overview
1. **Gesture tracking** *(Processing)*
   - Webcam or depth cam feeds into a Processing sketch that estimates coarse crowd motion.
   - We ship normalized floats over OSC using [oscP5](https://www.sojamo.de/libraries/oscP5/) & [NetP5](https://www.sojamo.de/libraries/netP5/).
   - Inspirations: [OfxCv optical flow demos](https://github.com/kylemcdonald/ofxCv) and [LASER Tag (Graffiti Research Lab)](http://graffitiresearchlab.com/blog/projects/laser-tag/).
2. **Control bridge (Pi/PC)** *(Python)*
   - [`osc_msp_bridge.py`](software/control-bridge/osc_msp_bridge.py) converts the OSC data into Betaflight RC microseconds via the [Minimal Serial Protocol (MSP)](https://github.com/betaflight/betaflight.com/blob/master/docs/development/API/MSP-Extensions.md). Think “software radio transmitter.”
   - Uses [pyserial](https://pyserial.readthedocs.io/en/latest/), [python-osc](https://pypi.org/project/python-osc/), and config from `config/mapping.yaml`.
   - Modelled after [Tello gesture-flight experiments](https://github.com/kinivi/tello-gesture-control) and [Red Paper Heart’s drone installations](https://redpaperheart.com/).
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
├─ examples/                    # Classroom-scale sketches (start with drift_minimal)
├─ firmware/                    # Teensy LED (and optional DSP) firmware via PlatformIO
├─ hardware/                    # Cage, LED wiring, parts lists
├─ scripts/                     # Utility scripts (logs, calibration, recorders)
└─ software/
   ├─ control-bridge/           # OSC → MSP bridge (Python)
   ├─ gesture-tracking/         # Processing sketch + HUD for crowd consent
   └─ video-pipeline/           # GStreamer/OBS scenes, launchers
```

---

## Drift Minimal example (hello world)

Need a zero-risk sandbox for class demos? `examples/drift_minimal/drift_minimal.py` is the repo’s new training wheels:

- **Run**: `python3 examples/drift_minimal/drift_minimal.py --input path/to/photo.jpg` (input optional; it auto-builds a gradient seed if you don’t pass one).
- **Loop**: Tkinter paints a single still image three times, each with a slight color tint and sin/cos offset so the stack feels like layered parallax.
- **Learn**: code comments narrate the full input → transform → output chain; copy/paste into Processing or TouchDesigner workshops without any drone hardware nearby.

Use it as an icebreaker in classes before graduating students to the full Control Stack Playbook.

---

## Config
- [`config/mapping.yaml`](config/mapping.yaml) provides tunable curves for **altitude**, **lateral drift**, **yaw bias**, **LED color**, and **glitch intensity**.
- [`config/video-presets.json`](config/video-presets.json) defines named GStreamer pipeline presets for OBS/GStreamer hybrids (clean, glitchy, mirrored, infrared, and scope overlays).
- **See the [Video pipeline map](docs/video-pipeline.md)** for a mermaid sketch (camera/VRX → gst_launch/OBS → projector) plus a worked `gst_launch.sh` command matched to the `clean_low_latency` preset so you can trace the capture path end-to-end before you start improvising.
- [`config/recipes/`](config/recipes) bundles “whole mood” presets — gesture curves, video chains, LED notes — ready to load via the new `--recipe` flag.
- [`hardware/`](hardware) includes a fleshed-out [hardware BOM](hardware/README.md), wiring, and net rig notes based on [Drone Cage DIY](https://hackaday.io/project/19102-drone-safety-cage) write-ups.

---

## Ethics & Consent
- Visible “consent gate”: drones/video remain idle until participants opt‑in (button/gesture). Inspired by [Studio Olafur Eliasson’s participation consent practice](https://www.theartnewspaper.com/2019/07/09/olafur-eliasson-introduces-participation-consent-forms-to-new-tate-modern-show).
- Data minimization: no face storage, no cloud calls. See `docs/PRIVACY_ETHICS.md` and `docs/ASSUMPTION_LEDGER.md` for what we log (spoiler: almost nothing).
- Follow [ADA walkway clearance guidelines](https://www.access-board.gov/ada/chapter-4-accessible-routes/#403-walking-surfaces) when placing cages and screens so wheelchairs aren’t boxed out.

---

## Deep-dive links & inspo
- **Control theory crash course**: [UAVTech’s Betaflight PID bible](https://www.uavtech.com/pids) for tuning feel vs. safety.
- **Interactive art ancestors**: [Daniel Rozin’s mirror series](http://smoothware.com/danny/woodenmirror.html) and [Rafael Lozano-Hemmer’s “Pulse”](https://www.lozano-hemmer.com/pulse_room.php) for crowd-controlled ambiance.
- **Latency budgeting**: [Bitcraze’s article on Vicon + Crazyflie latency](https://www.bitcraze.io/2020/05/latency-in-position-control-with-motion-capture/) — relevant when you scale to swarms.
- **Network hygiene**: [NDI vs. SDI vs. analog FPV comparison](https://www.ptzoptics.com/guides/ndi/) if you swap video transports.
- **Show control patterns**: [Ableton Link OSC bridges](https://github.com/ideoforms/link-python) and [TouchDesigner OSC out](https://docs.derivative.ca/OSC_Out_DAT) make for easy interoperability.

---

## Licensing
- Code: MIT. Docs: CC‑BY 4.0. See `LICENSE`.
