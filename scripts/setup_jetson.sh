#!/usr/bin/env bash
# Bootstrap a Jetson Orin Nano for perceptual-drift: install deps, create venv, clone/update repo.
# Assumptions: running as the target user with sudo access; GitHub SSH configured (falls back to HTTPS).
set -euo pipefail

# --- Configurable bits -------------------------------------------------------
REPO_DIR="$HOME/code/perceptual-drift"
VENV_DIR="$HOME/venvs/perceptual-drift"
GIT_SSH_URL="git@github.com:YOUR_GITHUB_USERNAME/perceptual-drift.git"  # TODO: replace username
GIT_HTTPS_URL="https://github.com/YOUR_GITHUB_USERNAME/perceptual-drift.git"  # fallback

# --- System packages ---------------------------------------------------------
echo "[setup] Updating apt-get and installing core packages..."
sudo apt-get update
sudo apt-get install -y \
  python3 python3-venv python3-pip git \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav \
  libopencv-dev python3-opencv

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
if [[ -f requirements.txt ]]; then
  echo "[setup] Installing Python dependencies from requirements.txt"
  pip install --upgrade pip
  pip install -r requirements.txt
else
  echo "[setup] No requirements.txt found; install manually as needed"
fi

# --- Final instructions ------------------------------------------------------
cat <<'MSG'
[setup] Jetson bootstrap complete.
- Activate env: source ~/venvs/perceptual-drift/bin/activate
- Run sensor check: python scripts/check_sensors.py
- Run hello scene: python examples/jetson_hello_camera.py --config config/platform_jetson_orin_nano.yaml
MSG
