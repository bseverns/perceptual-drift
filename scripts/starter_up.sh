#!/usr/bin/env bash
# Starter bundle launcher: minimal tracker + bridge + optional video preview.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

OSC_PORT=9000
HZ=30
SERIAL_PORT="FAKE"
TRACKER_MODE="synthetic"
VIDEO_MODE="auto"  # auto | on | off
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
VIDEO_PRESET="clean_low_latency"
RECIPE=""
TRACKER_HOST="127.0.0.1"

usage() {
  cat <<'USAGE'
Usage: ./scripts/starter_up.sh [options]

Options:
  --osc-port N               OSC port for tracker -> bridge (default: 9000)
  --hz N                     Bridge rate (default: 30)
  --serial PORT              Flight-controller serial port (default: FAKE => --dry-run)
  --tracker-mode MODE        synthetic | camera (default: synthetic)
  --tracker-host HOST        OSC host for bridge (default: 127.0.0.1)
  --video MODE               auto | on | off (default: auto)
  --video-device PATH        Capture device for GStreamer (default: /dev/video0)
  --video-preset NAME        GStreamer preset (default: clean_low_latency)
  --recipe PATH              Recipe path for bridge/video
  -h, --help                 Show this help

Examples:
  ./scripts/starter_up.sh
  ./scripts/starter_up.sh --tracker-mode camera --video on
  ./scripts/starter_up.sh --serial /dev/ttyUSB0 --video off
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --osc-port)
      OSC_PORT="$2"
      shift 2
      ;;
    --hz)
      HZ="$2"
      shift 2
      ;;
    --serial)
      SERIAL_PORT="$2"
      shift 2
      ;;
    --tracker-mode)
      TRACKER_MODE="$2"
      shift 2
      ;;
    --tracker-host)
      TRACKER_HOST="$2"
      shift 2
      ;;
    --video)
      VIDEO_MODE="$2"
      shift 2
      ;;
    --video-device)
      VIDEO_DEVICE="$2"
      shift 2
      ;;
    --video-preset)
      VIDEO_PRESET="$2"
      shift 2
      ;;
    --recipe)
      RECIPE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[starter] unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ "$TRACKER_MODE" != "synthetic" && "$TRACKER_MODE" != "camera" ]]; then
  echo "[starter] --tracker-mode must be synthetic or camera" >&2
  exit 2
fi

if [[ "$VIDEO_MODE" != "auto" && "$VIDEO_MODE" != "on" && "$VIDEO_MODE" != "off" ]]; then
  echo "[starter] --video must be auto, on, or off" >&2
  exit 2
fi

if [[ -n "$RECIPE" && ! -f "$RECIPE" ]]; then
  echo "[starter] recipe not found: $RECIPE" >&2
  exit 2
fi

echo "[starter] running preflight checks"
./scripts/starter_doctor.sh

RUN_DIR="runtime/starter_bundle"
mkdir -p "$RUN_DIR"
BRIDGE_LOG="$RUN_DIR/bridge.log"
TRACKER_LOG="$RUN_DIR/tracker.log"
VIDEO_LOG="$RUN_DIR/video.log"
BRIDGE_PID_FILE="$RUN_DIR/bridge.pid"
TRACKER_PID_FILE="$RUN_DIR/tracker.pid"
VIDEO_PID_FILE="$RUN_DIR/video.pid"

PIDS=()

start_bridge() {
  local cmd=(
    python3 software/control-bridge/osc_msp_bridge.py
    --serial "$SERIAL_PORT"
    --osc_port "$OSC_PORT"
    --hz "$HZ"
  )

  if [[ -n "$RECIPE" ]]; then
    cmd+=(--recipe "$RECIPE")
  else
    cmd+=(--config config/mapping.yaml)
  fi

  if [[ "$SERIAL_PORT" == "FAKE" ]]; then
    cmd+=(--dry-run)
  fi

  echo "[starter] bridge log: $BRIDGE_LOG"
  "${cmd[@]}" >"$BRIDGE_LOG" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  echo "$pid" >"$BRIDGE_PID_FILE"
  echo "[starter] bridge pid=$pid"
}

start_tracker() {
  local cmd=(
    python3 software/starter-bundle/minimal_tracker.py
    --host "$TRACKER_HOST"
    --port "$OSC_PORT"
    --mode "$TRACKER_MODE"
    --fps 20
  )
  echo "[starter] tracker log: $TRACKER_LOG"
  "${cmd[@]}" >"$TRACKER_LOG" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  echo "$pid" >"$TRACKER_PID_FILE"
  echo "[starter] tracker pid=$pid"
}

start_video() {
  if [[ "$VIDEO_MODE" == "off" ]]; then
    echo "[starter] video disabled (--video off)"
    return
  fi

  if ! command -v gst-launch-1.0 >/dev/null 2>&1; then
    if [[ "$VIDEO_MODE" == "on" ]]; then
      echo "[starter] requested video but gst-launch-1.0 is missing" >&2
      exit 1
    fi
    echo "[starter] video skipped (gst-launch-1.0 missing)"
    return
  fi

  if [[ ! -e "$VIDEO_DEVICE" ]]; then
    if [[ "$VIDEO_MODE" == "on" ]]; then
      echo "[starter] requested video but device missing: $VIDEO_DEVICE" >&2
      exit 1
    fi
    echo "[starter] video skipped (device missing: $VIDEO_DEVICE)"
    return
  fi

  local cmd=(./software/video-pipeline/gst_launch.sh)
  if [[ -n "$RECIPE" ]]; then
    cmd+=(--recipe "$RECIPE" "$VIDEO_PRESET" --device "$VIDEO_DEVICE")
  else
    cmd+=("$VIDEO_PRESET" --device "$VIDEO_DEVICE")
  fi

  echo "[starter] video log: $VIDEO_LOG"
  "${cmd[@]}" >"$VIDEO_LOG" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  echo "$pid" >"$VIDEO_PID_FILE"
  echo "[starter] video pid=$pid"
}

cleanup() {
  rm -f "$BRIDGE_PID_FILE" "$TRACKER_PID_FILE" "$VIDEO_PID_FILE"
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

start_bridge
sleep 0.3
start_tracker
sleep 0.3
start_video

echo "[starter] running. Press Ctrl-C to stop."
echo "[starter] logs: $RUN_DIR"
echo "[starter] quick tail: tail -f $BRIDGE_LOG $TRACKER_LOG"

while true; do
  for pid in "${PIDS[@]}"; do
    if ! kill -0 "$pid" >/dev/null 2>&1; then
      if wait "$pid"; then
        status=0
      else
        status=$?
      fi
      echo "[starter] process exited (pid=$pid, status=$status). Check logs in $RUN_DIR." >&2
      exit "$status"
    fi
  done
  sleep 1
done
