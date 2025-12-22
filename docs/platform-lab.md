# Platform Lab — Jetson + Laptop + Pi with the same vibe

Welcome to the drift lab where we treat a Jetson Orin Nano, a workhorse laptop, and a Raspberry Pi as interchangeable bandmates. This page pulls the platform notes, setup scripts, and headless smoke tests out of the README so the front page can breathe while still teaching the "why" behind each move.

## Profiles at a glance
Pick a profile to match the box on your desk, then tailor the camera pipeline if your sensors don’t line up with the defaults.

- **Laptop / dev box** — [`config/platform_desktop.yaml`](../config/platform_desktop.yaml)
  - Uses OpenCV capture with a preview window on so you can eyeball drift motion without extra monitors.
  - CPU-first; great for debugging GStreamer swaps before you go hardware-accelerated.
- **Jetson Orin Nano** — [`config/platform_jetson_orin_nano.yaml`](../config/platform_jetson_orin_nano.yaml)
  - GStreamer CSI/USB pipeline placeholder; swap in the right `gstreamer_pipeline` string for your sensor/port.
  - Window disabled by default so you can run headless; CUDA encouraged when available.
- **Raspberry Pi bridge/camera** — [`config/platform_pi.yaml`](../config/platform_pi.yaml)
  - `libcamerasrc` baked in with the preview window off; OSC defaults to `localhost` so the bridge and gesture tracker can share a box.
  - Flip `backend` to `opencv` if you’re debugging with `cv2.VideoCapture` instead of a GStreamer chain.

## Wiring your mappings
- **OSC** — [`config/mappings/osc.yaml`](../config/mappings/osc.yaml) names the drift metrics → OSC addresses. Drop it into TouchDesigner/Max or any OSC monitor to watch the numbers wiggle.
- **MIDI** — [`config/mappings/midi.yaml`](../config/mappings/midi.yaml) mirrors the same parameters on CCs so you can slam sliders or route into Ableton rigs.

## Bootstrap routines
Ready-to-run scripts that get each platform gig-ready.

### Jetson bring-up
1. Flash your Jetson and run [`scripts/setup_jetson.sh`](../scripts/setup_jetson.sh).
   - Installs system deps, spins a venv at `~/venvs/perceptual-drift`, and pulls this repo to `~/code/perceptual-drift` (swap the Git remote if you’re not `git@github.com:<you>/perceptual-drift.git`).
2. Sanity-check the camera bus with [`scripts/check_sensors.py`](../scripts/check_sensors.py). It’ll yell if indices 0–5 are dead.
3. Time the pipeline with [`scripts/profile_pipeline.py`](../scripts/profile_pipeline.py --config ../config/platform_jetson_orin_nano.yaml). It grabs frames, computes a tiny drift metric, and prints per-stage timing so you know if GStreamer/CUDA are doing their job.

### Pi control bridge
1. On Raspberry Pi OS, run [`scripts/setup_pi.sh`](../scripts/setup_pi.sh).
   - Installs `python-osc`/`pyserial`, GStreamer bits, and writes a `systemd` unit that keeps [`osc_msp_bridge.py`](../software/control-bridge/osc_msp_bridge.py) alive against `/dev/ttyACM0` (swap to `/dev/ttyUSB0` if your FC shows up there).
2. The unit defaults to `--hz 50` with `config/mapping.yaml`; edit `/etc/systemd/system/osc-msp-bridge.service` then `sudo systemctl daemon-reload && sudo systemctl restart osc-msp-bridge` whenever you change ports or mapping files.
3. Serial fallback / heartbeat loop: pass `--dry-run` to `osc_msp_bridge.py` when the flight controller is unplugged; the dry-run serial stub logs every MSP frame so you can rehearse OSC mappings without arming props. When `/pd/consent` drops to `0`, the bridge auto-blasts neutral RC values so Betaflight still sees a heartbeat while the quad stays napping — a safe rehearsal loop layered on top of radio failsafes.

## Headless-ish hello
- Run `python3 examples/jetson_hello_camera.py --config config/platform_jetson_orin_nano.yaml` from the repo root.
- Watch the console FPS/drift log; if `outputs.window.enabled` is `true` you also get an OpenCV window with a pulsing circle sized to drift. Smash `q` to bail.

## Field notes as a living log
- Jetson runs: log successes + weirdness in [`notes/jetson_field_notes.md`](../notes/jetson_field_notes.md).
- Laptop dev sessions: mirror the habit in [`notes/lap_field_notes.md`](../notes/lap_field_notes.md) so future you remembers which USB hub or lighting tweak fixed things.
- Pi bridge bring-up: jot what worked in a local notebook or inline comments near [`scripts/setup_pi.sh`](../scripts/setup_pi.sh) so the next swap of SD cards is painless.

Keep scribbling; this is a half studio notebook, half teaching guide. Novelty < utility.
