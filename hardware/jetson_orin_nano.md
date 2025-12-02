# Jetson Orin Nano – perceptual-drift Node

> Rough-and-ready field sheet for bringing up a Jetson Orin Nano as a perceptual-drift node. Equal parts checklist and lab notebook.

## Board + OS
- Board: NVIDIA Jetson Orin Nano (4GB/8GB; note SKU)
- JetPack / L4T: TODO fill with installed version (e.g., 6.x / Ubuntu 20.04)
- Storage: note microSD / NVMe size and remaining headroom

## Hostname + identity
- Suggested hostname: `perceptual-drift-jetson-01`
- Network: reserve or document LAN IP; note Wi‑Fi vs. wired
- SSH: enable key-based auth; avoid password logins

## Base configuration
1. Create user (or reuse `ubuntu`), add to `sudo`.
2. Drop SSH public key into `~/.ssh/authorized_keys`.
3. Configure networking (static/reserved DHCP); set DNS + NTP.
4. Sync time: `sudo systemctl enable systemd-timesyncd && sudo systemctl start systemd-timesyncd`.
5. Optional: set timezone `sudo timedatectl set-timezone <Region/City>`.

## Dependencies for perceptual-drift
```bash
sudo apt-get update
sudo apt-get install -y python3 python3-venv python3-pip git \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav \
  libopencv-dev python3-opencv
```
- Virtualenv: `~/venvs/perceptual-drift` (created by setup script below)
- Activate: `source ~/venvs/perceptual-drift/bin/activate`
- Repo path: `~/code/perceptual-drift`

## Repo layout on Jetson
- `config/` platform + mapping YAMLs
- `scripts/` setup, sensor checks, profiling helpers
- `examples/` runnable scenes (start with `jetson_hello_camera.py`)
- `docs/` architecture + scene recipes
- `hardware/` this doc and hardware-specific notes
- `notes/` field logs while on site

## First boot checklist
1. Flash Jetson with target JetPack.
2. Run `scripts/setup_jetson.sh` (as the Jetson user) to install deps + clone repo.
3. Activate env: `source ~/venvs/perceptual-drift/bin/activate`.
4. Confirm camera visibility: `python scripts/check_sensors.py`.
5. Tailor platform config: edit `config/platform_jetson_orin_nano.yaml` (`outputs.osc.host`, GStreamer pipeline).
6. Run hello scene: `python examples/jetson_hello_camera.py --config config/platform_jetson_orin_nano.yaml`.
7. Watch console drift values; enable window output if X11 is available.

## Notes / field observations
Use this section as a running log; jot down quirks, temperatures, throughput, and network weirdness.

- [ ] Date/time:
- [ ] Scene tested:
- [ ] Config used:
- [ ] Observations:
- [ ] Glitches/failures:
- [ ] Hypotheses / next experiments:

Stay scrappy, write everything down, and keep the node reproducible.
