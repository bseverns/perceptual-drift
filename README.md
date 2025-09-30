# Perceptual Drift — Audience‑Controlled FPV Installation

Perceptual Drift is a participatory drone‑based installation where **audience motion** modulates **FPV drone** behavior and **live video processing**. It centers shared authorship: many bodies → probabilistic control → drones that “drift” perceptually. The installation uses **DIY‑friendly stacks**: Betaflight micro‑drones, Raspberry Pi video processing (GStreamer/OBS), OpenFrameworks/Processing for gesture tracking, and an OSC→MSP control bridge. Optional **Teensy DSP/LED** layers extend visual/sonic feedback.

---

## Why this repo
- **Rapid pilot → exhibition‑scale**: start with 1 drone, 1 projector; scale to a swarm and multi‑screen projections.
- **Open & reproducible**: off‑the‑shelf micro‑FPV, open libraries, simple wiring.
- **Safety‑first**: indoor netted cage, conservative flight limits, visible consent gates.

---

## System Overview
- **Gesture tracking** → OSC messages (position/velocity/crowd activity)
- **Control bridge (Pi/PC)** → maps OSC to Betaflight MSP commands, sets LED states
- **FPV video** → captured by VRX + USB capture → processed (delay, glitch, color) → projected
- **(Optional) Audio/DSP** → prop/room mics → Teensy DSP → PA

See `docs/diagrams/system-overview.md` for mermaid diagrams.

### What each block actually leans on
We assume zero secret knowledge. Here are the exact pieces doing the heavy lifting:

| Layer | Core tech | Why we picked it | Deep dives |
| --- | --- | --- | --- |
| Gesture Tracking | [Processing](https://processing.org/) + the [processing.video `Capture`](https://processing.org/reference/libraries/video/Capture.html) API | Dead-simple webcam ingestion and a thriving creative coding community. | [Daniel Shiffman's Nature of Code videos](https://youtube.com/playlist?list=PLRqwX-V7Uu6aG2RJHErXKSWFDXU4qo_ro) for motion tricks. |
| OSC Plumbing | [`oscP5`](http://www.sojamo.de/libraries/oscP5/) (Processing) + [`python-osc`](https://github.com/attwad/python-osc) | These libraries hide UDP quirks and let you focus on choreography. | [OSC spec](http://opensoundcontrol.org/spec-1_0) explains the message format. |
| Flight Control | [Betaflight](https://betaflight.com/) drones steered via the [MSP 2.0 protocol](https://github.com/betaflight/betaflight/wiki/MSP-Protocol) and [`pySerial`](https://pyserial.readthedocs.io/) | MSP lets us bypass radios and talk raw RC to the quad—great for installations. | [Joshua Bardwell’s MSP deep-dive](https://www.youtube.com/watch?v=whQ0h1k3D8E) for background. |
| Video | [GStreamer 1.0](https://gstreamer.freedesktop.org/) CLI pipelines, optionally piped into [OBS Studio](https://obsproject.com/) | Handles VRX capture cards on crusty laptops without drama. | [GStreamer Plugin Reference](https://gstreamer.freedesktop.org/documentation/) when you want wild FX. |
| Optional Audio / LEDs | [Teensy 4.x](https://www.pjrc.com/store/teensy40.html) + [FastLED](https://fastled.io/) or [OctoWS2811](https://www.pjrc.com/teensy/td_libs_OctoWS2811.html) | Simple to sync pixels/FX with what the drone is doing. | [Noisebridge Drone Swarm writeups](https://noisebridge.net/wiki/Drone_Swarm) for inspiration. |

### Reference projects + pattern inspo
We built this after ogling a bunch of rad predecessors. Take cues, remix freely:

- **Participatory drone shows**: [ETH Zurich Drone Swarm performances](https://www.dfab.ch/project/spaxels) prove that swarms can be poetic, not just corporate. We steal their “safety cage + choreography” ethos.
- **Audience-controlled installations**: [Lauren McCarthy’s *US_+*](https://lauren-mccarthy.com/US_) and [Rafael Lozano-Hemmer’s *Pulse Room*](https://www.lozano-hemmer.com/pulse_room.php) show how biosignals can own the stage. We echo their consent-first framing.
- **DIY FPV art hacks**: [Jeff Thompson’s FPV performances](https://www.jeffreythompson.org/blog/2019/02/24/fpv-drone-performance-notes/) and [Mataerial drone antics](https://www.mataerial.com/) demonstrate how cheap whoops + bespoke code can run gallery-grade experiences.
- **OSC control bridges**: [Ableton Link → DMX bridges](https://github.com/radionica-abierta/abletonlink2dmx) inspired our choice to keep all channels in OSC until the final hop.

No references? No problem. Clone those links, binge them, and riff hard.

---

## Repo Layout
```
.
├─ config/                      # YAML/JSON mappings and presets
├─ docs/                        # Diagrams, checklists, assumption ledger, install notes
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
1) **Hardware**: Net a 2×2 m cage. Use a BetaFPV Cetus Pro (or TinyWhoop) with Betaflight. Attach a light WS2812 strip (8–12 px).
2) **Tracking**: Run the Processing sketch in `software/gesture-tracking/processing/PerceptualDrift_Tracker`. A webcam aimed at audience area is fine for the pilot.
3) **Control Bridge**: Start `software/control-bridge/osc_msp_bridge.py`. It listens on UDP/OSC and sends MSP control to the flight controller (USB/UART).
4) **Video**: Launch `software/video-pipeline/gst_launch.sh` to view/process the FPV feed from a VRX → USB capture device.
5) **Safety**: Follow `docs/checklists/safety_checklist.md` before each session. Keep props guarded and flight limits conservative.

---

## Config
- `config/mapping.yaml` provides tunable curves for **altitude**, **lateral drift**, **yaw bias**, **LED color**, and **glitch intensity**.
- `config/video-presets.json` defines named GStreamer pipeline presets.

### How to read the config files without going cross-eyed
- The YAML file is the single source of truth for control mapping. Every key in the `osc.address_space` map corresponds to the OSC paths broadcast by the Processing sketch. The `mapping` block then translates those normalized values into the microsecond-based RC numbers that Betaflight expects. Check [Betaflight’s receiver tab docs](https://github.com/betaflight/betaflight/wiki/Receiver) if you’re new to that world.
- The JSON presets are literally GStreamer command fragments. Paste them into `gst-launch-1.0` or drop them into OBS as “Custom” sources. If you need primer-level guidance, the [GStreamer Basic tutorial](https://gstreamer.freedesktop.org/documentation/tutorials/basic/index.html) walks through similar pipelines.

---

## Ethics & Consent
- Visible “consent gate”: drones/video remain idle until participants opt‑in (button/gesture).
- Data minimization: no face storage, no cloud calls. See `docs/PRIVACY_ETHICS.md` and `docs/ASSUMPTION_LEDGER.md`.

---

## Licensing
- Code: MIT. Docs: CC‑BY 4.0. See `LICENSE`.

---

## Crash course for the impatient builder
Want to sketch your own vibe without spelunking every file? Here’s the 10,000-foot rundown with receipts:

1. **Flash a whoop with Betaflight** using the [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator). Set up a serial port for MSP, limit throttle ranges, and arm via AUX to stay safe.
2. **Run the Processing tracker** (`software/gesture-tracking/processing/PerceptualDrift_Tracker`). It uses the [oscP5 quickstart](http://www.sojamo.de/libraries/oscP5/#hello) pattern and cheap frame differencing to emit crowd motion metrics.
3. **Launch the Python bridge** (`software/control-bridge/osc_msp_bridge.py`). Learn about MSP framing via the [official spec](https://github.com/betaflight/betaflight/wiki/MSP-Protocol). We keep the implementation stupidly small so you can audit it.
4. **Pipe FPV video with GStreamer** using `software/video-pipeline/gst_launch.sh`. Once you trust it, swap `autovideosink` for `obsksvideosink` and flow into OBS scenes. Reference the [OBS GStreamer docs](https://obsproject.com/kb/gstreamer-source) for that hop.
5. **Layer LEDs or DSP** with a Teensy + [PlatformIO](https://platformio.org/). Check `firmware/` for skeleton sketches and read the [OctoWS2811 manual](https://www.pjrc.com/teensy/td_libs_OctoWS2811.html) before you wire.

Everything here is purposely transparent. Fork it, credit your collaborators, and send us footage of whatever haunted techno swamp you conjure.
