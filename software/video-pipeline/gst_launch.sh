#!/usr/bin/env bash
# Simple GStreamer pipelines (edit device indexes as needed)

preset="${1:-clean_low_latency}"

if [ "$preset" = "clean_low_latency" ]; then
  gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! autovideosink sync=false
elif [ "$preset" = "delayed_glitch" ]; then
  gst-launch-1.0 v4l2src device=/dev/video0 ! queue max-size-buffers=180 ! videorate ! videoconvert ! autovideosink sync=false
else
  echo "Unknown preset: $preset"
fi
