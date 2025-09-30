#!/usr/bin/env bash
# gst_launch.sh — choose-your-own GStreamer adventure for the FPV capture chain.
#
# HOW THIS WORKS:
#   * We lean on the `gst-launch-1.0` CLI to avoid writing a ton of Python/GLSL just to test video ideas.
#   * Each preset is a hand-written pipeline fragment. Swap camera devices, add effects, or copy/paste into OBS.
#   * Need help? Read the GStreamer docs: https://gstreamer.freedesktop.org/documentation/tutorials/basic/ .
#
# QUICK TIPS:
#   * `/dev/video0` usually maps to your USB capture card. Run `v4l2-ctl --list-devices` if the device path is different.
#   * Add `videobalance`/`glitch` plugins right in the pipeline string when you want to get weird.
#   * `sync=false` keeps latency low by not waiting for vsync. For gallery installs, feed the output into OBS or a projector directly.
#
# Example custom pipeline (line breaks keep things readable; trailing "\" continues each stage):
#   gst-launch-1.0 \ ← start the capture graph
#     v4l2src device=/dev/video1 ! queue ! videoconvert ! tee name=t \ ← split the feed
#     t. ! queue ! videoscale ! autovideosink sync=false \ ← clean preview window
#     t. ! queue ! videobalance saturation=1.5 ! autovideosink sync=false   # ← saturated FX tap

preset="${1:-clean_low_latency}"   # Default preset unless the user supplies one as $1

if [ "$preset" = "clean_low_latency" ]; then
  # Lean pipeline: USB capture → colorspace fix → display. Minimal latency, good for rehearsals.
  gst-launch-1.0 \
    v4l2src device=/dev/video0 ! \
    videoconvert ! \
    autovideosink sync=false
elif [ "$preset" = "delayed_glitch" ]; then
  # Adds a deep queue for frame buffering + rate limiting for VHS-y smears.
  # Tweak `max-size-buffers` and add effects (e.g., `videobalance hue=0.2`) to taste.
  gst-launch-1.0 \
    v4l2src device=/dev/video0 ! \
    queue max-size-buffers=180 ! \
    videorate ! \
    videoconvert ! \
    autovideosink sync=false
else
  echo "Unknown preset: $preset"
  echo "Try one of: clean_low_latency, delayed_glitch"
fi
