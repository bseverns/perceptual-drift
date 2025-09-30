#!/usr/bin/env bash
# Tiny launcher for the video side of Perceptual Drift.
#
# Why GStreamer?  It ships on Raspberry Pi OS, plays nice with low-latency USB
# capture dongles, and mirrors the recipes from Scanlines and VDMX communities.
# You can absolutely port these to OBS (build a Scene Collection with the same
# sources), but the shell script helps during rehearsal when you just need a
# window fast.
#
# Helpful links while you tweak:
# * v4l2src docs — https://gstreamer.freedesktop.org/documentation/v4l2/v4l2src.html
# * gst-launch primer — https://gstreamer.freedesktop.org/documentation/tutorials/basic/gstreamer-tools.html
# * Autovideosink quirks — https://gstreamer.freedesktop.org/documentation/autodetect/autovideosink.html

set -euo pipefail

preset="${1:-clean_low_latency}"
device="${VIDEO_DEVICE:-/dev/video0}"  # Override with VIDEO_DEVICE=/dev/video2 ./gst_launch.sh

case "$preset" in
  clean_low_latency)
    # Direct monitor pipeline.  sync=false keeps the sink from blocking when the
    # projector can't keep up — useful in clubs with mystery frame rates.
    gst-launch-1.0 \
      v4l2src device="$device" ! \
      queue leaky=downstream max-size-buffers=5 ! \
      videoconvert ! \
      autovideosink sync=false
    ;;
  delayed_glitch)
    # A campy feedback loop: deep queue for ~6 s delay at 30 fps, then videorate
    # to smooth jitter before display.  Tweak ``max-size-buffers`` to time-shift.
    gst-launch-1.0 \
      v4l2src device="$device" ! \
      queue max-size-buffers=180 max-size-bytes=0 max-size-time=0 ! \
      videorate ! \
      videobalance saturation=1.6 brightness=-0.05 ! \
      videoconvert ! \
      autovideosink sync=false
    ;;
  *)
    echo "Unknown preset: $preset" >&2
    echo "Try one of: clean_low_latency | delayed_glitch" >&2
    exit 1
    ;;
esac
