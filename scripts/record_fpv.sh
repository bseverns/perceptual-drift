#!/usr/bin/env bash
# Record the analog FPV feed to disk with FFmpeg.
#
# Default behavior: grab from /dev/video0 for 10 minutes and dump a Matroska
# file under ./logs/.  The idea is you can trigger this during a run to snag
# archival footage without touching OBS.
#
# Links to keep on hand:
# * FFmpeg V4L2 capture — https://trac.ffmpeg.org/wiki/Capture/Webcam
# * TinyUSB capture latency measurements — https://docs.luxonis.com/projects/hardware/en/latest/pages/guides/usb_latency/

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
export PERCEPTUAL_DRIFT_LOG_DIR="${PERCEPTUAL_DRIFT_LOG_DIR:-$REPO_ROOT/logs}"

log_event() {
  local action="$1"
  local status="$2"
  local message="$3"
  local details_json="${4:-}"

  python3 - "$action" "$status" "$message" "$details_json" <<'PY'
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

action, status, message, details = sys.argv[1:5]
log_dir = Path(os.environ.get("PERCEPTUAL_DRIFT_LOG_DIR", Path("logs")))
if not log_dir.is_absolute():
    log_dir = Path.cwd() / log_dir
log_dir.mkdir(parents=True, exist_ok=True)
log_path = log_dir / "ops_events.jsonl"

event = {
    "timestamp": datetime.now(timezone.utc).isoformat(),
    "operator": os.environ.get("OPERATOR_ID")
    or os.environ.get("USER")
    or os.environ.get("USERNAME")
    or "unknown",
    "host": os.environ.get("HOSTNAME", "unknown_host"),
    "action": action,
    "status": status,
    "message": message,
}

if details:
    try:
        event["details"] = json.loads(details)
    except json.JSONDecodeError:
        event["details"] = {"raw": details}

with log_path.open("a", encoding="utf-8") as fh:
    fh.write(json.dumps(event, ensure_ascii=False) + "\n")
PY
}

mkdir -p "$PERCEPTUAL_DRIFT_LOG_DIR"

ts=$(date +%Y%m%d_%H%M%S)
device="${VIDEO_DEVICE:-/dev/video0}"
duration="${DURATION:-00:10:00}"
codec_args=("-c" "copy")  # Swap for "-c:v h264_v4l2m2m" on Pi if you want encoded files.
output_path="$PERCEPTUAL_DRIFT_LOG_DIR/fpv_${ts}.mkv"

export RECORD_FPV_DEVICE="$device"
export RECORD_FPV_DURATION="$duration"
export RECORD_FPV_OUTPUT="$output_path"
START_DETAILS=$(python3 <<'PY'
import json
import os

print(json.dumps({
    "device": os.environ.get("RECORD_FPV_DEVICE"),
    "duration": os.environ.get("RECORD_FPV_DURATION"),
    "output": os.environ.get("RECORD_FPV_OUTPUT"),
    "phase": "start",
}))
PY
)
log_event "record_fpv" "started" "FPV capture initializing." "$START_DETAILS"
unset RECORD_FPV_DEVICE RECORD_FPV_DURATION

trap 'log_event "record_fpv" "error" "FFmpeg aborted during FPV capture."' ERR

echo "Recording from $device for $duration → $output_path" >&2

ffmpeg -hide_banner -loglevel info \
  -f v4l2 -i "$device" \
  -t "$duration" \
  "${codec_args[@]}" \
  "$output_path"

trap - ERR

export RECORD_FPV_OUTPUT="$output_path"
END_DETAILS=$(python3 <<'PY'
import json
import os

print(json.dumps({
    "output": os.environ.get("RECORD_FPV_OUTPUT"),
    "phase": "complete",
}))
PY
)
log_event "record_fpv" "completed" "FPV capture finalized." "$END_DETAILS"
unset RECORD_FPV_OUTPUT END_DETAILS START_DETAILS
