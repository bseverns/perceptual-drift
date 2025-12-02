#!/usr/bin/env python3
"""Profile a minimal capture + drift loop on desktop or Jetson.
Loads a platform config, opens the configured camera backend, and reports timing stats.
This is a pragmatic pre-flight tool before deeper drift-core integrations."""

import argparse
import time
from typing import Dict, List, Tuple

import cv2
import numpy as np
import yaml


def load_config(path: str) -> Dict:
  with open(path, "r", encoding="utf-8") as f:
    return yaml.safe_load(f)


def build_capture(video_cfg: Dict) -> cv2.VideoCapture:
  backend = video_cfg.get("backend", "opencv")
  if backend == "gstreamer":
    pipeline = video_cfg.get("gstreamer_pipeline")
    if not pipeline:
      raise ValueError("GStreamer backend requested but no pipeline provided in config")
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
  else:
    device_index = int(video_cfg.get("device_index", 0))
    cap = cv2.VideoCapture(device_index)
  return cap


def compute_stats(samples: List[float]) -> Tuple[float, float, float]:
  if not samples:
    return 0.0, 0.0, 0.0
  return (sum(samples) / len(samples), min(samples), max(samples))


def main() -> None:
  parser = argparse.ArgumentParser(description="Profile capture + simple drift")
  parser.add_argument(
    "--config",
    default="config/platform_jetson_orin_nano.yaml",
    help="Path to platform config YAML",
  )
  parser.add_argument(
    "--frames",
    type=int,
    default=300,
    help="Number of frames to profile",
  )
  args = parser.parse_args()

  cfg = load_config(args.config)
  video_cfg = cfg.get("video", {})
  perf_cfg = cfg.get("performance", {})
  max_fps = int(perf_cfg.get("max_fps", 0))

  cap = build_capture(video_cfg)
  if not cap.isOpened():
    raise RuntimeError("Could not open video capture with provided config")

  capture_times: List[float] = []
  convert_times: List[float] = []
  drift_times: List[float] = []

  prev_gray = None
  target_dt = 1.0 / max_fps if max_fps > 0 else 0.0

  for i in range(args.frames):
    loop_start = time.perf_counter()

    t0 = time.perf_counter()
    ok, frame = cap.read()
    t1 = time.perf_counter()
    if not ok or frame is None:
      print(f"[profile] Frame {i}: capture failed, stopping")
      break

    t2 = time.perf_counter()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    t3 = time.perf_counter()

    drift_value = 0.0
    if prev_gray is not None:
      diff = cv2.absdiff(gray, prev_gray)
      drift_value = float(np.mean(diff) / 255.0)
    prev_gray = gray
    t4 = time.perf_counter()

    capture_times.append((t1 - t0) * 1000.0)
    convert_times.append((t3 - t2) * 1000.0)
    drift_times.append((t4 - t3) * 1000.0)

    if target_dt > 0:
      elapsed = time.perf_counter() - loop_start
      remaining = target_dt - elapsed
      if remaining > 0:
        time.sleep(remaining)

  cap.release()

  capture_avg, capture_min, capture_max = compute_stats(capture_times)
  convert_avg, convert_min, convert_max = compute_stats(convert_times)
  drift_avg, drift_min, drift_max = compute_stats(drift_times)

  print("[profile] Results over {0} frames".format(len(capture_times)))
  print("  Capture  : avg={0:.3f} ms, min={1:.3f} ms, max={2:.3f} ms".format(capture_avg, capture_min, capture_max))
  print("  Convert  : avg={0:.3f} ms, min={1:.3f} ms, max={2:.3f} ms".format(convert_avg, convert_min, convert_max))
  print("  Drift    : avg={0:.3f} ms, min={1:.3f} ms, max={2:.3f} ms".format(drift_avg, drift_min, drift_max))


if __name__ == "__main__":
  main()
