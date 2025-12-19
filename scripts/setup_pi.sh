#!/usr/bin/env bash
# Bootstrap a Raspberry Pi as the OSC→MSP bridge + camera scout.
# Goal: mirror the Jetson convenience script but tuned for Pi OS + serial-first setups.
set -euo pipefail

# --- Configurable bits -------------------------------------------------------
REPO_DIR="${REPO_DIR:-$HOME/code/perceptual-drift}"
VENV_DIR="${VENV_DIR:-$HOME/venvs/perceptual-drift}"
GIT_SSH_URL="${GIT_SSH_URL:-git@github.com:YOUR_GITHUB_USERNAME/perceptual-drift.git}"  # swap username
GIT_HTTPS_URL="${GIT_HTTPS_URL:-https://github.com/YOUR_GITHUB_USERNAME/perceptual-drift.git}"  # fallback clone
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"  # change to /dev/ttyUSB0 if your FC enumerates there

# --- System packages ---------------------------------------------------------
echo "[setup] Updating apt and installing Pi-friendly packages..."
sudo apt-get update
sudo apt-get install -y \
  python3 python3-venv python3-pip git \
  python3-serial \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav \
  libatlas-base-dev libopenjp2-7 libtiff5 libilmbase-dev \
  libopenexr-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

# --- Virtual environment ----------------------------------------------------
if [[ ! -d "$VENV_DIR" ]]; then
  echo "[setup] Creating virtualenv at $VENV_DIR"
  python3 -m venv "$VENV_DIR"
else
  echo "[setup] Virtualenv already exists at $VENV_DIR"
fi

# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"
echo "[setup] Using Python from $(which python3)"

# --- Repo checkout/update ----------------------------------------------------
mkdir -p "$(dirname "$REPO_DIR")"
if [[ -d "$REPO_DIR/.git" ]]; then
  echo "[setup] Repo already present, pulling latest main..."
  git -C "$REPO_DIR" pull
else
  echo "[setup] Cloning repo (SSH preferred, HTTPS fallback)..."
  if git clone "$GIT_SSH_URL" "$REPO_DIR"; then
    echo "[setup] Cloned via SSH"
  else
    echo "[setup] SSH clone failed, trying HTTPS"
    git clone "$GIT_HTTPS_URL" "$REPO_DIR"
  fi
fi

cd "$REPO_DIR"

# --- Python dependencies -----------------------------------------------------
# requirements.txt is intentionally absent; pin the bridge deps explicitly.
pip install --upgrade pip
pip install python-osc pyserial pyyaml

# --- systemd unit for osc_msp_bridge ---------------------------------------
SERVICE_PATH="/etc/systemd/system/osc-msp-bridge.service"
if [[ ! -f "$SERVICE_PATH" ]]; then
  echo "[setup] Dropping systemd unit to keep the OSC→MSP bridge alive..."
  sudo tee "$SERVICE_PATH" > /dev/null <<SERVICE
[Unit]
Description=perceptual-drift OSC to MSP bridge
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
Restart=on-failure
User=$USER
WorkingDirectory=$REPO_DIR
Environment=PYTHONUNBUFFERED=1
ExecStart=$VENV_DIR/bin/python $REPO_DIR/software/control-bridge/osc_msp_bridge.py --serial $SERIAL_PORT --config $REPO_DIR/config/mapping.yaml --hz 50

[Install]
WantedBy=multi-user.target
SERVICE
  sudo systemctl daemon-reload
  sudo systemctl enable --now osc-msp-bridge.service
else
  echo "[setup] systemd unit already exists at $SERVICE_PATH (not touching it)."
fi

# --- Final instructions ------------------------------------------------------
cat <<'MSG'
[setup] Raspberry Pi bootstrap complete.
- Activate env when hacking: source ~/venvs/perceptual-drift/bin/activate
- Edit /etc/systemd/system/osc-msp-bridge.service if your serial port/hz change, then sudo systemctl daemon-reload && sudo systemctl restart osc-msp-bridge
- For lab loops without a flight controller, run: python software/control-bridge/osc_msp_bridge.py --dry-run --config config/mapping.yaml --hz 30
MSG
