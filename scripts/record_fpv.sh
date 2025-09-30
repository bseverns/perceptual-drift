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

mkdir -p logs

ts=$(date +%Y%m%d_%H%M%S)
device="${VIDEO_DEVICE:-/dev/video0}"
duration="${DURATION:-00:10:00}"
codec_args=("-c" "copy")  # Swap for "-c:v h264_v4l2m2m" on Pi if you want encoded files.

echo "Recording from $device for $duration → logs/fpv_${ts}.mkv" >&2

ffmpeg -hide_banner -loglevel info \
  -f v4l2 -i "$device" \
  -t "$duration" \
  "${codec_args[@]}" \
  "logs/fpv_${ts}.mkv"
