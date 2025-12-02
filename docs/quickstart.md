# Quickstart & Onboarding — Drift Flight School

This is the no-nonsense field manual for getting a fresh human up to speed and a single drone safely in the air. It keeps the punk-rock texture of the main README but spares you the scroll.

---

## Onboarding flight school (start here)

0. **Touch the Drift Minimal sketch** in [`examples/drift_minimal/`](../examples/drift_minimal/README.md). Run `python3 examples/drift_minimal/drift_minimal.py` and watch one still image drift via three layered offsets. It’s the punk-rock lab demo that lets students feel the installation’s core behavior without drones, OSC, or GStreamer.
1. **Run the staged onboarding playbook** in [`docs/onboarding/`](onboarding/README.md). It walks you through three levels — systems sighting, consent choreography, and safety loop drills — with "do this now" exercises so you earn muscle memory, not just vibes.
2. **Log your artifacts** (Level 1 dry-run screenshot, Level 2 consent rehearsal capture, Level 3 checklist PDF) and open a GitHub discussion titled `Onboarding sign-off — <your name>` tagging `@control-lead`, `@experience-lead`, and `@safety-second` once all drills are done.
3. **Wait for sign-off** — leads usually respond within 48 hours; once approved you’re clear to co-run rehearsals.

---

## How to tour this notebook (a.k.a. your school syllabus)

1. **Start with the [Control Stack Playbook](control-stack-playbook.md)** — *Why you care:* it’s the soup-to-nuts wiring diagram for prototypes, so you know exactly when to open [`software/gesture-tracking/processing/PerceptualDrift_Tracker/PerceptualDrift_Tracker.pde`](../software/gesture-tracking/processing/PerceptualDrift_Tracker/PerceptualDrift_Tracker.pde), [`software/control-bridge/osc_msp_bridge.py`](../software/control-bridge/osc_msp_bridge.py), and the knobs inside [`config/mapping.yaml`](../config/mapping.yaml) before you solder anything.
2. **Skim the [Safety Checklist](checklists/safety_checklist.md)** — *Why you care:* this is the punk-rock preflight liturgy; keep it open while you’re flashing firmware or tweaking [`hardware/README.md`](../hardware/README.md) so the “oops” moments stay on paper, not on people.
3. **Digest the [Experience Playbook](experience/README.md)** — *Why you care:* once the prototype hovers, this choreographs rehearsals: when to flip the consent AUX in [`software/control-bridge/osc_msp_bridge.py`](../software/control-bridge/osc_msp_bridge.py), when to remix projections via [`software/video-pipeline/gst_launch.sh`](../software/video-pipeline/gst_launch.sh), and how to brief humans without killing the vibe.
4. **Update the [Assumption Ledger](ASSUMPTION_LEDGER.md)** — *Why you care:* during runs you log surprises, then feed them back into configs like [`config/video-presets.json`](../config/video-presets.json) and flight curves in [`config/mapping.yaml`](../config/mapping.yaml) so the system evolves intentionally instead of by rumor.
5. **Reference the [System Diagrams](diagrams/system-overview.md)** whenever something feels abstract — *Why you care:* the mermaid maps keep the OSC→MSP→LED trail legible, pointing you back to tooling like [`scripts/record_fpv.sh`](../scripts/record_fpv.sh) when it’s time to capture evidence or debug latency. These diagrams also surface in the onboarding playbook with failure-mode callouts, so the visuals stay consistent between study and drills. Pair them with the [bridge telemetry cheat sheet](operations/bridge_telemetry.md) so you know what the audit logs are whispering during a run.

Treat that order as gospel for newcomers: prototype, secure, rehearse, reflect, repeat. No more spelunking through tabs wondering which YAML is the boss.

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
   - Plug the drone’s flight controller in over USB, then run `python3 software/control-bridge/osc_msp_bridge.py --serial /dev/ttyUSB0` from the repo root (swap the serial device if you’re on macOS or Windows).
   - Check out [Betaflight’s MSP docs](https://github.com/betaflight/betaflight.com/blob/master/docs/development/API/MSP-Extensions.md) if you want to push extra AUX channels.
4. **Stack health check**
   - Fire `./scripts/check_stack.py` once your Processing tracker and OSC bridge are humming. The [operations playbook](operations/playbook.md) is the annotated score: it maps each printout, tells you what “normal” feels like, and lists the weird smells plus triage moves when the script throws shade.
   - The run hammers the Processing tracker, OSC bridge, MSP framing, and Teensy mocks — a full dress rehearsal for the control artery — so you know the whole stack is breathing before you unleash a drone.
   - Bonus robustness drills: pass `--neutralize-after 12` to flip consent off mid-stream and confirm the MSP feed actually chills out to neutral RC values. If you typo the fixture path or hand it garbage JSON, the harness now bails with a loud error instead of pretending everything is fine.
   - Want zero surprises between laptops, CI, and the Jetson? Follow the [environment gauntlet](operations/playbook.md#make-every-environment-prove-itself-deployment-gate) so every platform proves it can run the tests before a real audience gets anywhere near the rig.
   - GitHub Actions already runs the “CI / container sanity” loop on every push — if the bot chokes, your rig would have too. Fix it before walking into a venue.
5. **Video pipeline**
   - `sudo apt install gstreamer1.0-tools` or use OBS.
   - Run `./software/video-pipeline/gst_launch.sh clean_low_latency` for tight monitoring or `delayed_glitch` for delayed projection loops. Adapted from [Scanlines’ GStreamer recipes](https://scanlines.xyz/t/gstreamer-recipes/1414).
6. **Safety dance**
   - Live inside [`docs/checklists/safety_checklist.md`](checklists/safety_checklist.md). That single source of truth covers consent rituals, kill-switch drills, and spotter call-and-response. Bring it to every rehearsal so wording never drifts.

Use this quickstart when you need to teach or test the core loop fast. Once you have it down, graduate to the deeper playbooks referenced above.
