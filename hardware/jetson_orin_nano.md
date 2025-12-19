# Jetson Orin Nano – perceptual-drift Node

> Rough-and-ready field sheet for bringing up a Jetson Orin Nano as a perceptual-drift node. Equal parts checklist and lab notebook.

## Board + OS
- Board: NVIDIA Jetson Orin Nano (4GB/8GB; note SKU)
- JetPack / L4T: **Validated on JetPack 6.0 GA (L4T 36.3) running Ubuntu 20.04.6 LTS**. Treat this as the golden image when cloning nodes; mismatched L4T kernels tend to break camera drivers.
- Known quirks on this combo:
  - First boot sometimes wobbles the HDMI link; a reboot settles it.
  - `nvargus-daemon` will chew a core for ~60 seconds on first camera open while it builds caches; be patient instead of killing it.
  - GStreamer `nvh264dec` occasionally disappears after an OTA update—reinstall `nvidia-l4t-gstreamer` if hardware decode vanishes.
- Storage: 64 GB microSD is the floor; 128 GB NVMe feels like breathing room. Expect ~18–22 GB consumed after flashing + deps + our venv, so leave at least 40 GB free for logs and captures.

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
- GStreamer/OpenCV alignment notes (for Pi↔Jetson parity):
  - JetPack 6.0 GA ships GStreamer 1.20.x via `nvidia-l4t-gstreamer`; keep Raspberry Pi images on GStreamer 1.20.x as well so pipeline strings stay portable.
  - OpenCV from apt is 4.5.x on JetPack 6.0. Pin Pi builds to the same major/minor (4.5) to avoid ABI roulette with `cv2.VideoCapture` backends.
  - If you must rebuild either stack, freeze the versions in your image notes and keep `gst-inspect-1.0 --version` + `python3 -c "import cv2; print(cv2.__version__)"` outputs in the field log.

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
