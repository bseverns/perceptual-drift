# Starter Bundle (v0)

`pd-safe-rehearsal` is now the canonical no-hardware first run for new
contributors. This page leads with that safe path, then documents
`starter_doctor.sh` and `starter_up.sh` as the lower-level manual tools
underneath it.

The manual starter bundle still packages the three core lanes:
- tracker (minimal Python tracker, no Processing required)
- bridge (`osc_msp_bridge.py`)
- video (optional GStreamer preview)

## What this solves

- Avoids initial setup across Processing + ROS2 + firmware tools.
- Gives one place to verify OSC routes, consent gating, and bridge health.
- Supports synthetic mode when no camera hardware is available.

## Canonical no-hardware first run

1. Install starter dependencies:

```bash
python3 -m pip install -r requirements-starter.txt
python3 -m pip install -e . --no-build-isolation
```

2. Start the safe rehearsal bundle:

```bash
pd-safe-rehearsal
```

This starts:
- operator UI
- bridge in dry-run mode
- minimal tracker in synthetic mode
- consent in the documented safe default: OFF

Status and stop commands:

```bash
pd-status
pd-safe-rehearsal stop
```

Use this path when you want one obvious command for a laptop-only rehearsal.
It is the recommended first run for docs, onboarding, and CI parity.

## Manual starter path

Use the manual path when you need to drive the lower-level launcher directly,
debug startup flags, or bypass the operator UI.

1. Install starter dependencies:

```bash
python3 -m pip install -r requirements-starter.txt
```

2. Run preflight:

```bash
./scripts/starter_doctor.sh
```

3. Launch starter bundle directly:

```bash
./scripts/starter_up.sh
```

This manual path starts:
- bridge in `--dry-run` mode (if `--serial` is left as `FAKE`)
- minimal tracker in `synthetic` mode
- video preview if `gst-launch-1.0` and `/dev/video0` are available

Consent remains a required invariant here too: for no-hardware rehearsal,
keep consent OFF until an operator intentionally changes it through the UI or
an explicit control path.

## Manual path options

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

Safe rehearsal metadata, UI logs, and PID files live separately under:

`runtime/rehearsal/`

## Out of scope for v0

- Processing-based tracker setup
- ROS2/CrazySwarm2 swarm flows
- Teensy firmware flashing/programming

Those stay in the full playbooks and quickstarts.
