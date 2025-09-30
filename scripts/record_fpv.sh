#!/usr/bin/env bash
mkdir -p logs
ts=$(date +%Y%m%d_%H%M%S)
ffmpeg -f v4l2 -i /dev/video0 -t 00:10:00 -c copy "logs/fpv_$ts.mkv"
