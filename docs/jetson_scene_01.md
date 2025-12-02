# Jetson Scene 01: camera → drift → OSC → laptop

> Minimal end-to-end chain to prove the Jetson can see the world and shout about it over the network.

## Goal
Run a single-camera drift metric on the Jetson and stream it via OSC to a laptop or live rig.

## Requirements
- Jetson Orin Nano with camera attached (CSI or USB) and network access.
- Desktop/laptop on the same LAN to receive OSC (e.g., with an OSC monitor or DAW bridge).
- Virtualenv created via `scripts/setup_jetson.sh` and dependencies installed.

## Configuration
1. Edit `config/platform_jetson_orin_nano.yaml`:
   - Set `outputs.osc.host` to the laptop's IP on the LAN.
   - Tweak `video.gstreamer_pipeline` if your camera requires different caps.
2. Ensure `config/mappings/osc.yaml` includes `frame_drift` (and any other metrics you expect).

## Running the scene (Jetson)
```bash
source ~/venvs/perceptual-drift/bin/activate
cd ~/code/perceptual-drift
python examples/jetson_hello_camera.py --config config/platform_jetson_orin_nano.yaml
```
- Press `q` to exit if a window is open; otherwise Ctrl+C.
- Console should print FPS and drift values.

## On the laptop
- Open an OSC monitor (e.g., [oscdump](https://github.com/colinbdclark/osc.js/) or a DAW bridge) listening on the host/port from the config.
- Watch for messages under `/perceptual_drift/...` (see `config/mappings/osc.yaml`).

## What to observe
- Drift values rise with motion in front of the camera and settle when still.
- OSC messages arrive with minimal lag; packet rate tracks frame rate.
- If headless, confirm CPU/GPU load and thermals stay within reason.

## Next steps
- Add more drift metrics (centroids, entropy) and wire them into OSC/MIDI.
- Experiment with multiple cameras or higher resolutions once stable.
- Pipe OSC into your live rig and push the feedback loop further.
