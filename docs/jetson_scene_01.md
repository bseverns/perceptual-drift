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
   - `outputs.osc.host` defaults to `raspberrypi.local`, pointing at the Pi that soaks up
     `/vision/gesture` and other OSC payloads in the hybrid chain. Override it when pairing
     to a different listener (e.g., the lead laptop or a stage box) without committing changes
     by exporting an env var and piping through `yq`:

     ```bash
     export PD_OSC_HOST=10.0.0.42  # whatever IP/hostname your OSC bridge is listening on
     yq '.outputs.osc.host = env(PD_OSC_HOST)' \
       config/platform_jetson_orin_nano.yaml > /tmp/pd_jetson_osc.yaml
     ```

     Then point the scene at the patched file via the CLI:

     ```bash
     python examples/jetson_hello_camera.py --config /tmp/pd_jetson_osc.yaml
     ```

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
