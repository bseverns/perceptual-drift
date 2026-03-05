#!/usr/bin/env bash
# Bootstrap a Raspberry Pi as the OSC→MSP bridge + camera scout.
# Goal: mirror the Jetson convenience script but tuned for Pi OS + serial-first setups.
set -euo pipefail

# --- Configurable bits -------------------------------------------------------
REPO_DIR="${REPO_DIR:-$HOME/code/perceptual-drift}"
VENV_DIR="${VENV_DIR:-$HOME/venvs/perceptual-drift}"
DEFAULT_REPO_URL="https://github.com/bseverns/perceptual-drift.git"
REPO_URL="${REPO_URL:-$DEFAULT_REPO_URL}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"  # change to /dev/ttyUSB0 if your FC enumerates there
PI_APT_REL="config/env/pi-apt-packages.txt"
REQ_REL="requirements-starter.txt"
CONSTRAINTS_REL="constraints/py310-linux.txt"

if [[ "$REPO_URL" =~ ^git@github\.com:(.*)$ ]]; then
  REPO_PATH="${BASH_REMATCH[1]}"
  GIT_SSH_URL="$REPO_URL"
  GIT_HTTPS_URL="https://github.com/${REPO_PATH}"
elif [[ "$REPO_URL" =~ ^https://github\.com/(.*)$ ]]; then
  REPO_PATH="${BASH_REMATCH[1]}"
  GIT_HTTPS_URL="$REPO_URL"
  GIT_SSH_URL="git@github.com:${REPO_PATH}"
else
  echo "[setup] Unsupported REPO_URL: $REPO_URL" >&2
  echo "[setup] Use either git@github.com:owner/repo.git or https://github.com/owner/repo.git" >&2
  exit 1
fi

echo "[setup] Repo source defaults to ${DEFAULT_REPO_URL}".
echo "[setup] To point at your fork, rerun with REPO_URL=https://github.com/YOU/perceptual-drift.git (SSH/HTTPS auto-derived)."

# --- System packages ---------------------------------------------------------
echo "[setup] Updating apt and installing bootstrap packages..."
sudo apt-get update
sudo apt-get install -y python3 python3-venv python3-pip git

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

# --- Apt package profile -----------------------------------------------------
if [[ -f "$PI_APT_REL" ]]; then
  mapfile -t apt_packages < <(
    sed -e 's/#.*//' -e '/^[[:space:]]*$/d' "$PI_APT_REL"
  )
  if [[ "${#apt_packages[@]}" -gt 0 ]]; then
    echo "[setup] Installing Pi apt profile from $PI_APT_REL"
    sudo apt-get install -y "${apt_packages[@]}"
  fi
else
  echo "[setup] Missing $PI_APT_REL; skipping Pi apt profile install" >&2
fi

# --- Python dependencies -----------------------------------------------------
# Install pinned deps using the same files as CI/local starter flows.
pip install --upgrade pip
if [[ -f "$CONSTRAINTS_REL" ]]; then
  pip install -r "$REQ_REL" -c "$CONSTRAINTS_REL"
else
  pip install -r "$REQ_REL"
fi

if [[ -x scripts/export_env_manifest.sh ]]; then
  scripts/export_env_manifest.sh --out runtime/pi_setup_manifest.txt
fi

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
- Apt profile: config/env/pi-apt-packages.txt
- Activate env when hacking: source ~/venvs/perceptual-drift/bin/activate
- Edit /etc/systemd/system/osc-msp-bridge.service if your serial port/hz change, then sudo systemctl daemon-reload && sudo systemctl restart osc-msp-bridge
- For lab loops without a flight controller, run: python software/control-bridge/osc_msp_bridge.py --dry-run --config config/mapping.yaml --hz 30
- Environment manifest: runtime/pi_setup_manifest.txt
- Repo remote defaults to https://github.com/bseverns/perceptual-drift.git; override by rerunning with REPO_URL set to your fork (SSH/HTTPS auto-derived).
MSG
