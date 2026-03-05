# Dependency Matrix (Baseline for Starter + Core Control Stack)

This matrix is the current reference for versions and ownership. It is focused
on the components used by the starter bundle and the core single-drone flow.

## Supported environments (current target)

| Environment | Status | Notes |
| --- | --- | --- |
| Desktop dev box (Linux/macOS) | supported | Primary contributor path |
| Jetson Orin Nano field rig | supported with caveats | Version drift around GStreamer/OpenCV needs stricter pinning |
| Raspberry Pi bridge node | supported | Bridge-first role, not primary dev host |
| ROS2/CrazySwarm2 swarm rig | experimental | Out of starter bundle scope |

## Core software dependencies

| Component | Version target | Pin source | Required for |
| --- | --- | --- | --- |
| Python | 3.10+ | `docs/operations/playbook.md` | Bridge, scripts, starter tracker |
| python-osc | 1.8.3 | `software/control-bridge/requirements.txt` | OSC IO |
| pyserial | 3.5 | `software/control-bridge/requirements.txt` | MSP serial bridge |
| PyYAML | 6.0.2 | `software/control-bridge/requirements.txt` | Mapping + recipe parsing |
| mido | 1.3.2 | `software/control-bridge/requirements.txt` | MIDI mapping support |
| python-rtmidi (optional) | 1.5.8 | `software/control-bridge/requirements.midi.txt` | Native MIDI backend |
| OpenCV (camera mode in starter tracker) | distro/pip provided | `scripts/setup_jetson.sh`, `scripts/setup_pi.sh` | Minimal camera tracker mode |
| GStreamer CLI/plugins | distro provided | `scripts/setup_jetson.sh`, `scripts/setup_pi.sh` | Video pipeline launcher |
| Constraints lock | pinned | `constraints/py310-linux.txt` | Shared reproducibility floor for Linux/Python 3.10 |
| Dev toolchain | pinned | `requirements-dev.txt` | CI + local lint/test parity |

## Hardware/firmware dependencies

| Component | Version target | Pin source | Notes |
| --- | --- | --- | --- |
| Betaflight MSP (`MSP_SET_RAW_RC`) | protocol-compatible | `software/control-bridge/osc_msp_bridge.py` | Bridge assumes MSP framing + 8 RC channels |
| Teensy LED/DSP firmware | PlatformIO-managed | `firmware/teensy-*/platformio.ini` | Optional in starter bundle |
| Processing 4 + video/osc libs | contributor-installed | `docs/quickstart.md` | Full tracker path, out of starter v0 |

## Known alignment risks

1. Jetson package versions can diverge from desktop assumptions (especially
   GStreamer and OpenCV behavior/perf).
2. `scripts/setup_jetson.sh` currently installs `requirements.txt` from repo
   root, but the repo does not use a root runtime requirements file.
3. Video toolchain is distro-managed, so plugin availability can differ between
   lab machines.
4. HDMI hotplug instability is a known field issue in Jetson notes and should
   be treated as operational risk during rehearsals.

## Immediate hardening actions

1. Use `requirements-starter.txt` for starter bundle runtime parity.
2. Install with `requirements-dev.txt` + `constraints/py310-linux.txt` for CI/local alignment.
3. Add environment diagnostics (`scripts/starter_doctor.sh`) before launch.
4. Capture setup manifests using `scripts/export_env_manifest.sh`.
