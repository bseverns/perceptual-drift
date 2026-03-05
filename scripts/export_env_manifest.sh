#!/usr/bin/env bash
# Snapshot runtime/tool versions to make rehearsals reproducible.
set -euo pipefail

OUT_PATH=""

usage() {
  cat <<'USAGE'
Usage: ./scripts/export_env_manifest.sh [--out PATH]

If --out is omitted, writes to runtime/env_manifest_<timestamp>.txt.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out)
      [[ $# -ge 2 ]] || { echo "--out expects a path" >&2; exit 2; }
      OUT_PATH="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ -z "$OUT_PATH" ]]; then
  mkdir -p runtime
  OUT_PATH="runtime/env_manifest_$(date +%Y%m%d_%H%M%S).txt"
else
  mkdir -p "$(dirname "$OUT_PATH")"
fi

{
  echo "timestamp=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "hostname=$(hostname 2>/dev/null || echo unknown)"
  echo "kernel=$(uname -srmo 2>/dev/null || echo unknown)"
  echo "python=$(python3 --version 2>&1 || echo missing)"
  echo "pip=$(python3 -m pip --version 2>&1 || echo missing)"
  echo "gstreamer=$(gst-launch-1.0 --version 2>/dev/null | head -n 1 || echo missing)"
  echo "nv_tegra_release=$(cat /etc/nv_tegra_release 2>/dev/null || echo none)"
  echo "opencv=$(python3 - <<'PY'
try:
    import cv2
    print(cv2.__version__)
except Exception:
    print("missing")
PY
)"
  if command -v dpkg-query >/dev/null 2>&1; then
    echo "apt_packages_begin"
    dpkg-query -W -f='${Package}=${Version}\n' \
      python3 python3-pip python3-venv \
      gstreamer1.0-tools gstreamer1.0-plugins-base \
      gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
      gstreamer1.0-plugins-ugly gstreamer1.0-libav \
      libopencv-dev python3-opencv 2>/dev/null || true
    echo "apt_packages_end"
  fi
} >"$OUT_PATH"

echo "[env] wrote $OUT_PATH"

