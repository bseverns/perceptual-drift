# Perceptual Drift — Audience‑Controlled FPV Installation

Perceptual Drift is a participatory drone‑based installation where **audience motion** modulates **FPV drone** behavior and **live video processing**. It centers shared authorship: many bodies → probabilistic control → drones that “drift” perceptually. The installation leans on **DIY‑friendly stacks**: Betaflight micro‑drones, Raspberry Pi video processing (GStreamer/OBS), Processing for gesture tracking, and an OSC→MSP control bridge. Optional **Teensy DSP/LED** layers extend visual/sonic feedback. Think of it as an unholy jam session between [Betaflight](https://betaflight.com/), [Processing](https://processing.org/), [GStreamer](https://gstreamer.freedesktop.org/), and [python-osc](https://pypi.org/project/python-osc/) with a splash of [Mozzi](https://sensorium.github.io/Mozzi/) and [CrazySwarm2](https://crazyswarm.readthedocs.io/).

> **No breadcrumbs? No problem.** This README tries to be the missing field manual. Every subsystem below includes links, inspiration, and what to Google when things melt down.

---

## Onboarding flight school (start here)

1. **Run the staged onboarding playbook** in [`docs/onboarding/`](docs/onboarding/README.md). It walks you through three levels — systems sighting, consent choreography, and safety loop drills — with "do this now" exercises so you earn muscle memory, not just vibes.
2. **Log your artifacts** (Level 1 dry-run screenshot, Level 2 consent rehearsal capture, Level 3 checklist PDF) and open a GitHub discussion titled `Onboarding sign-off — <your name>` tagging `@control-lead`, `@experience-lead`, and `@safety-second` once all drills are done.
3. **Wait for sign-off** — leads usually respond within 48 hours; once approved you’re clear to co-run rehearsals.

---

## How to tour this notebook (a.k.a. your flight school syllabus)

1. **Start with the [Control Stack Playbook](docs/control-stack-playbook.md)** — *Why you care:* it’s the soup-to-nuts wiring diagram for prototypes, so you know exactly when to open [`software/gesture-tracking/processing/PerceptualDrift_Tracker/PerceptualDrift_Tracker.pde`](software/gesture-tracking/processing/PerceptualDrift_Tracker/PerceptualDrift_Tracker.pde), [`software/control-bridge/osc_msp_bridge.py`](software/control-bridge/osc_msp_bridge.py), and the knobs inside [`config/mapping.yaml`](config/mapping.yaml) before you solder anything.
2. **Skim the [Safety Checklist](docs/checklists/safety_checklist.md)** — *Why you care:* this is the punk-rock preflight liturgy; keep it open while you’re flashing firmware or tweaking [`hardware/README.md`](hardware/README.md) so the “oops” moments stay on paper, not on people.
3. **Digest the [Experience Playbook](docs/experience/README.md)** — *Why you care:* once the prototype hovers, this choreographs rehearsals: when to flip the consent AUX in [`software/control-bridge/osc_msp_bridge.py`](software/control-bridge/osc_msp_bridge.py), when to remix projections via [`software/video-pipeline/gst_launch.sh`](software/video-pipeline/gst_launch.sh), and how to brief humans without killing the vibe.
4. **Update the [Assumption Ledger](docs/ASSUMPTION_LEDGER.md)** — *Why you care:* during runs you log surprises, then feed them back into configs like [`config/video-presets.json`](config/video-presets.json) and flight curves in [`config/mapping.yaml`](config/mapping.yaml) so the system evolves intentionally instead of by rumor.
5. **Reference the [System Diagrams](docs/diagrams/system-overview.md)** whenever something feels abstract — *Why you care:* the mermaid maps keep the OSC→MSP→LED trail legible, pointing you back to tooling like [`scripts/record_fpv.sh`](scripts/record_fpv.sh) when it’s time to capture evidence or debug latency. These diagrams also surface in the onboarding playbook with failure-mode callouts, so the visuals stay consistent between study and drills. Pair them with the [bridge telemetry cheat sheet](docs/operations/bridge_telemetry.md) so you know what the audit logs are whispering during a run.

Treat that order as gospel for newcomers: prototype, secure, rehearse, reflect, repeat. No more spelunking through tabs wondering which YAML is the boss.

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
├─ firmware/                    # Teensy LED (and optional DSP) firmware via PlatformIO
├─ hardware/                    # Cage, LED wiring, parts lists
├─ scripts/                     # Utility scripts (logs, calibration, recorders)
└─ software/
   ├─ control-bridge/           # OSC → MSP bridge (Python)
   ├─ gesture-tracking/         # Processing sketch + HUD for crowd consent
   └─ video-pipeline/           # GStreamer/OBS scenes, launchers
```

---

## Quickstart (Pilot: 1 drone, 1 projector)
1. **Hardware**
   - Net a 2×2 m cage. BetaFPV Cetus Pro or any 65–75 mm whoop running Betaflight 4.x works.
   - USB power a VRX + capture dongle (classic EasyCAP or UVC cards). Mount a short WS2812 strip (8–12 px) to the quad if you want glow feedback.
   - Borrow ideas from [Intel’s drone show safety brief](https://www.intel.com/content/www/us/en/support/articles/000026520/drones.html) and [MIT’s Flyfire cage concept](https://senseable.mit.edu/flyfire/).
2. **Tracking rig**
   - Install [Processing 4](https://processing.org/download) plus the `video`, `oscP5`, and `netP5` libraries.
   - Run `software/gesture-tracking/processing/PerceptualDrift_Tracker`. Aim any 720p-ish webcam at the audience area and tweak the `threshold` constant for your lighting.
3. **Control bridge**
   - `pip install -r software/control-bridge/requirements.txt` (python-osc + pyserial + pyyaml).
   - Plug the drone’s flight controller in over USB, then run `python3 osc_msp_bridge.py --serial /dev/ttyUSB0`.
   - Check out [Betaflight’s MSP docs](https://github.com/betaflight/betaflight.com/blob/master/docs/development/API/MSP-Extensions.md) if you want to push extra AUX channels.
4. **Stack health check**
   - Run `./scripts/check_stack.py` once your Processing sketch and bridge are humming. The [operations playbook](docs/operations/playbook.md) breaks down what “good” looks like and how to triage noise.
   - This single command pings the Processing tracker, OSC bridge, MSP framing, and Teensy mocks so you know the whole control artery is alive before you unleash a drone.
5. **Video pipeline**
   - `sudo apt install gstreamer1.0-tools` or use OBS.
   - Run `./software/video-pipeline/gst_launch.sh clean_low_latency` for tight monitoring or `delayed_glitch` for delayed projection loops. Adapted from [Scanlines’ GStreamer recipes](https://scanlines.xyz/t/gstreamer-recipes/1414).
6. **Safety dance**
   - Live inside [`docs/checklists/safety_checklist.md`](docs/checklists/safety_checklist.md). That single source of truth covers consent rituals, kill-switch drills, and spotter call-and-response. Bring it to every rehearsal so wording never drifts.

---

## Config
- [`config/mapping.yaml`](config/mapping.yaml) provides tunable curves for **altitude**, **lateral drift**, **yaw bias**, **LED color**, and **glitch intensity**.
- [`config/video-presets.json`](config/video-presets.json) defines named GStreamer pipeline presets for OBS/GStreamer hybrids (clean, glitchy, mirrored, infrared, and scope overlays).
- [`config/recipes/`](config/recipes) bundles “whole mood” presets — gesture curves, video chains, LED notes — ready to load via the new `--recipe` flag.
- [`hardware/`](hardware) includes a fleshed-out [hardware BOM](hardware/README.md), wiring, and net rig notes based on [Drone Cage DIY](https://hackaday.io/project/19102-drone-safety-cage) write-ups.

See [`docs/recipes.md`](docs/recipes.md) for how to author your own.

- **Soft Consent Lounge** — Drift only when the room says yes. Altitude/lateral curves hug the center, LEDs breathe a cool aurora, and video stays soft-focus for onboarding. Load via `--recipe config/recipes/soft_consent_lounge.yaml` when you’re narrating safety.
- **Consent Signal** — Workshop-friendly mode that literally spotlights the consent gate. LEDs flash amber until the crowd says go and the projection overlays the toggle state. `--recipe config/recipes/consent_signal.yaml`.
- **Midnight Meditation** — Slow-breath intermission. Heavy deadzones, noir LEDs, and the infrared video preset keep everyone grounded between high-energy runs. `--recipe config/recipes/midnight_meditation.yaml`.
- **Riot Mode** — Full-send feedback party. Aggressive lateral gain, yaw jitter, neon LED stabs, and a delayed glitch wall. Flip to this once spotters and the crowd are synced. `--recipe config/recipes/riot_mode.yaml`.
- **Swarm Teaser** — One quad, swarm energy. Biases yaw into lazy figure-eights, paints gradient LED laps, and mirrors the feed to hint at future drones. Run with `--recipe config/recipes/swarm_teaser.yaml` when pitching the multi-craft upgrade.
- **Afterburner Chase** — Encore banger. Sharp lateral curves, jittery yaw bursts, and riot-level video feedback so the quad looks like it’s leaving burn trails. `--recipe config/recipes/afterburner_chase.yaml`.

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
