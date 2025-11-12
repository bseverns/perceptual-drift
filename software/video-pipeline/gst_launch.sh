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

usage() {
  cat <<'USAGE'
Usage: ./gst_launch.sh [PRESET] [--device /dev/videoX]
       ./gst_launch.sh --recipe config/recipes/soft_consent_lounge.yaml [PRESET]

Options:
  --recipe PATH   Load pipeline + intent from a recipe file (YAML/JSON).
  --device PATH   Override the video capture device (defaults to $VIDEO_DEVICE or /dev/video0).
  -h, --help      Show this message.

Without --recipe the legacy presets (clean_low_latency, delayed_glitch) are available.
When a recipe is supplied, PRESET selects a named chain inside that recipe. If not
provided, the recipe's default_chain is used.
USAGE
}

recipe=""
preset=""
device="${VIDEO_DEVICE:-/dev/video0}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --recipe)
      [[ $# -ge 2 ]] || { echo "--recipe expects a path" >&2; exit 1; }
      recipe="$2"
      shift 2
      ;;
    --device)
      [[ $# -ge 2 ]] || { echo "--device expects a path" >&2; exit 1; }
      device="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      echo "Unknown flag: $1" >&2
      usage
      exit 1
      ;;
    *)
      if [[ -z "$preset" ]]; then
        preset="$1"
        shift
      else
        echo "Unexpected positional argument: $1" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

if [[ -n "$recipe" ]]; then
  [[ -f "$recipe" ]] || { echo "Recipe not found: $recipe" >&2; exit 1; }
  mapfile -t parsed < <(python3 - "$recipe" "$device" "${preset:-}" <<'PY'
import sys
from pathlib import Path

import yaml

recipe_path = Path(sys.argv[1])
device = sys.argv[2]
selected = sys.argv[3]

with recipe_path.open() as fh:
    data = yaml.safe_load(fh)

if not isinstance(data, dict):
    sys.stderr.write(f"Recipe {recipe_path} does not parse into a mapping\n")
    sys.exit(1)

video = data.get("video_pipeline") or data.get("video") or {}
chains = video.get("gst_chains")
chain_name = selected

if chains:
    if chain_name:
        pipeline = chains.get(chain_name)
        if pipeline is None:
            options = ", ".join(chains.keys())
            sys.stderr.write(
                f"Recipe {recipe_path} has no chain '{chain_name}'. Available: {options}\n"
            )
            sys.exit(1)
    else:
        chain_name = video.get("default_chain") or next(iter(chains.keys()))
        pipeline = chains[chain_name]
else:
    pipeline = (
        video.get("gst_chain")
        or video.get("gst")
        or video.get("pipeline")
    )
    if isinstance(pipeline, dict):
        chain_name = chain_name or next(iter(pipeline.keys()))
        if chain_name not in pipeline:
            options = ", ".join(pipeline.keys())
            sys.stderr.write(
                f"Recipe {recipe_path} pipeline dict missing key '{chain_name}'. Available: {options}\n"
            )
            sys.exit(1)
        pipeline = pipeline[chain_name]
    elif not pipeline:
        sys.stderr.write(f"Recipe {recipe_path} is missing a video pipeline definition\n")
        sys.exit(1)
    elif not chain_name:
        chain_name = "default"

pipeline = pipeline.replace("{device}", device)
normalized = " ".join(
    segment.strip()
    for segment in pipeline.strip().splitlines()
    if segment.strip()
)
recipe_name = data.get("name", recipe_path.stem)
print(normalized)
print(chain_name)
print(recipe_name)
PY
)
  pipeline="${parsed[0]:-}"
  chain_name="${parsed[1]:-}"
  recipe_name="${parsed[2]:-}"
  if [[ -z "$pipeline" ]]; then
    echo "Failed to read pipeline from recipe $recipe" >&2
    exit 1
  fi
  echo "Launching recipe '$recipe_name' chain '$chain_name' via $device" >&2
  eval "gst-launch-1.0 ${pipeline}"
  exit 0
fi

preset="${preset:-clean_low_latency}"

case "$preset" in
  clean_low_latency)
    gst-launch-1.0 \
      v4l2src device="$device" ! \
      queue leaky=downstream max-size-buffers=5 ! \
      videoconvert ! \
      autovideosink sync=false
    ;;
  delayed_glitch)
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
    echo "Try one of: clean_low_latency | delayed_glitch or provide --recipe" >&2
    exit 1
    ;;
esac
