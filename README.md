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

---

## Ethics & Consent
- Visible “consent gate”: drones/video remain idle until participants opt‑in (button/gesture).
- Data minimization: no face storage, no cloud calls. See `docs/PRIVACY_ETHICS.md` and `docs/ASSUMPTION_LEDGER.md`.

---

## Licensing
- Code: MIT. Docs: CC‑BY 4.0. See `LICENSE`.
