#!/usr/bin/env bash
# Preflight checks for the starter bundle.
set -euo pipefail

STRICT=0

usage() {
  cat <<'USAGE'
Usage: ./scripts/starter_doctor.sh [--strict]

Checks required dependencies for the starter bundle:
- Python + required Python modules
- Mapping/config file presence
- Bridge and tracker script presence
- Optional video dependencies (GStreamer + /dev/video0)
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --strict)
      STRICT=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[doctor] unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

REQUIRED_FAILURES=0
WARNINGS=0

ok() {
  echo "[doctor] ok: $*"
}

warn() {
  WARNINGS=$((WARNINGS + 1))
  echo "[doctor] warn: $*"
}

fail() {
  REQUIRED_FAILURES=$((REQUIRED_FAILURES + 1))
  echo "[doctor] fail: $*"
}

check_cmd() {
  local cmd="$1"
  local label="$2"
  if command -v "$cmd" >/dev/null 2>&1; then
    ok "$label ($cmd)"
  else
    fail "$label missing ($cmd not found)"
  fi
}

check_file() {
  local path="$1"
  local label="$2"
  if [[ -f "$path" ]]; then
    ok "$label ($path)"
  else
    fail "$label missing ($path)"
  fi
}

check_cmd python3 "Python runtime"
check_file "config/mapping.yaml" "Base mapping"
check_file "software/control-bridge/osc_msp_bridge.py" "OSC->MSP bridge"
check_file "software/starter-bundle/minimal_tracker.py" "Starter tracker"
check_file "software/video-pipeline/gst_launch.sh" "Video launcher"

if python3 - <<'PY' >/dev/null 2>&1
import importlib
mods = ["pythonosc", "serial", "yaml", "mido"]
for mod in mods:
    importlib.import_module(mod)
PY
then
  ok "Required Python modules (python-osc, pyserial, PyYAML, mido)"
else
  fail "Missing Python modules. Run: python3 -m pip install -r requirements-starter.txt"
fi

if command -v gst-launch-1.0 >/dev/null 2>&1; then
  ok "GStreamer CLI detected (gst-launch-1.0)"
else
  warn "GStreamer CLI not found; starter can run without video. Install gstreamer1.0-tools."
fi

if [[ -e /dev/video0 ]]; then
  ok "Video device present (/dev/video0)"
else
  warn "No /dev/video0 found; video preview may not launch."
fi

if python3 - <<'PY' >/dev/null 2>&1
import importlib
importlib.import_module("cv2")
PY
then
  ok "OpenCV (cv2) available for camera tracker mode"
else
  warn "OpenCV missing; use synthetic tracker mode or install OpenCV."
fi

if python3 scripts/validate_config.py >/dev/null 2>&1; then
  ok "Mapping/config validation passed"
else
  fail "Config validation failed. Run: python3 scripts/validate_config.py"
fi

echo "[doctor] summary: required_failures=${REQUIRED_FAILURES} warnings=${WARNINGS}"

if [[ "$REQUIRED_FAILURES" -gt 0 ]]; then
  exit 1
fi

if [[ "$STRICT" -eq 1 && "$WARNINGS" -gt 0 ]]; then
  exit 2
fi

exit 0

