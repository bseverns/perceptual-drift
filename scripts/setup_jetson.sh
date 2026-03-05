#!/usr/bin/env bash
# Bootstrap a Jetson Orin Nano for perceptual-drift with pinned Python deps and
# a central apt package profile.
set -euo pipefail

# --- Configurable bits -------------------------------------------------------
REPO_DIR="$HOME/code/perceptual-drift"
VENV_DIR="$HOME/venvs/perceptual-drift"
DEFAULT_REPO_URL="https://github.com/bseverns/perceptual-drift.git"
REPO_URL="${REPO_URL:-$DEFAULT_REPO_URL}"
JETSON_APT_REL="config/env/jetson-apt-packages.txt"
REQ_REL="requirements-starter.txt"
CONSTRAINTS_REL="constraints/py310-linux.txt"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-url)
      if [[ -z "${2:-}" ]]; then
        echo "[setup] --repo-url expects a value" >&2
        exit 1
      fi
      REPO_URL="$2"
      shift 2
      ;;
    --repo-url=*)
      REPO_URL="${1#*=}"
      shift
      ;;
    *)
      echo "[setup] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

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

# --- Bootstrap packages ------------------------------------------------------
echo "[setup] Installing bootstrap packages (python/git)..."
sudo apt-get update
sudo apt-get install -y python3 python3-venv python3-pip git

# --- Virtual environment -----------------------------------------------------
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
if [[ -f "$JETSON_APT_REL" ]]; then
  mapfile -t apt_packages < <(
    sed -e 's/#.*//' -e '/^[[:space:]]*$/d' "$JETSON_APT_REL"
  )
  if [[ "${#apt_packages[@]}" -gt 0 ]]; then
    echo "[setup] Installing Jetson apt profile from $JETSON_APT_REL"
    sudo apt-get install -y "${apt_packages[@]}"
  fi
else
  echo "[setup] Missing $JETSON_APT_REL; skipping Jetson apt profile install" >&2
fi

# --- Python dependencies -----------------------------------------------------
echo "[setup] Installing pinned Python requirements"
pip install --upgrade pip
if [[ -f "$CONSTRAINTS_REL" ]]; then
  pip install -r "$REQ_REL" -c "$CONSTRAINTS_REL"
else
  pip install -r "$REQ_REL"
fi

if [[ -x scripts/export_env_manifest.sh ]]; then
  scripts/export_env_manifest.sh --out runtime/jetson_setup_manifest.txt
fi

# --- Final instructions ------------------------------------------------------
cat <<'MSG'
[setup] Jetson bootstrap complete.
- Profile: config/env/jetson-orin-nano.profile.yaml
- Activate env: source ~/venvs/perceptual-drift/bin/activate
- Run doctor: ./scripts/starter_doctor.sh
- Run sensor check: python scripts/check_sensors.py
- Run hello scene: python examples/jetson_hello_camera.py --config config/platform_jetson_orin_nano.yaml
- Environment manifest: runtime/jetson_setup_manifest.txt
MSG
