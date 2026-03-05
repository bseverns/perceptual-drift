# Starter Bundle (v0)

This is the low-friction launch path for new contributors.

It packages the three core lanes:
- tracker (minimal Python tracker, no Processing required)
- bridge (`osc_msp_bridge.py`)
- video (optional GStreamer preview)

## What this solves

- Avoids initial setup across Processing + ROS2 + firmware tools.
- Gives one place to verify OSC routes, consent gating, and bridge health.
- Supports synthetic mode when no camera hardware is available.

## Quick start

1. Install starter dependencies:

```bash
python3 -m pip install -r requirements-starter.txt
```

2. Run preflight:

```bash
./scripts/starter_doctor.sh
```

3. Launch starter bundle:

```bash
./scripts/starter_up.sh
```

This starts:
- bridge in `--dry-run` mode (if `--serial` is left as `FAKE`)
- minimal tracker in `synthetic` mode
- video preview if `gst-launch-1.0` and `/dev/video0` are available

## Common options

```bash
# Use real camera-driven tracker math
./scripts/starter_up.sh --tracker-mode camera

# Connect to a real FC serial port and disable video
./scripts/starter_up.sh --serial /dev/ttyUSB0 --video off

# Force video on and fail if unavailable
./scripts/starter_up.sh --video on --video-device /dev/video2
```

## Logs

Runtime logs are written to:

`runtime/starter_bundle/`

- `bridge.log`
- `tracker.log`
- `video.log` (if video launched)

## Out of scope for v0

- Processing-based tracker setup
- ROS2/CrazySwarm2 swarm flows
- Teensy firmware flashing/programming

Those stay in the full playbooks and quickstarts.

