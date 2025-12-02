#!/usr/bin/env python3
"""Simple Jetson-ready camera loop that measures frame-to-frame drift.
Loads a platform config (desktop or Jetson), opens the appropriate backend,
computes a normalized drift metric, and overlays it onto the frame.
This is meant to be a pragmatic "hello" scene before hooking into drift-core.
"""

import argparse
import time
from typing import Dict, Optional

import cv2
import numpy as np
import yaml


def load_config(path: str) -> Dict:
  with open(path, "r", encoding="utf-8") as f:
    return yaml.safe_load(f)


def open_capture(video_cfg: Dict) -> cv2.VideoCapture:
  backend = video_cfg.get("backend", "opencv")
  if backend == "gstreamer":
    pipeline = video_cfg.get("gstreamer_pipeline")
    if not pipeline:
      raise RuntimeError("GStreamer backend selected but no pipeline provided")
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
  else:
    device_index = int(video_cfg.get("device_index", 0))
    cap = cv2.VideoCapture(device_index)

  if not cap.isOpened():
    raise RuntimeError("Could not open camera with provided configuration")
  return cap


def compute_drift(prev_gray: Optional[np.ndarray], gray: np.ndarray) -> float:
  if prev_gray is None:
    return 0.0
  diff = cv2.absdiff(gray, prev_gray)
  return float(np.mean(diff) / 255.0)


def overlay_drift(frame: np.ndarray, drift_value: float) -> np.ndarray:
  overlay = frame.copy()
  h, w = overlay.shape[:2]
  center = (w // 2, h // 2)
  max_radius = int(min(w, h) * 0.4)
  radius = max(5, int(max_radius * drift_value))
  cv2.circle(overlay, center, radius, (0, 255, 0), 2)
  cv2.putText(
    overlay,
    f"drift: {drift_value:.4f}",
    (20, h - 20),
    cv2.FONT_HERSHEY_SIMPLEX,
    0.8,
    (0, 255, 0),
    2,
    cv2.LINE_AA,
  )
  return overlay


def main() -> None:
  parser = argparse.ArgumentParser(description="Hello camera/drift loop for Jetson or desktop")
  parser.add_argument(
    "--config",
    default="config/platform_jetson_orin_nano.yaml",
    help="Path to platform config YAML",
  )
  args = parser.parse_args()

  cfg = load_config(args.config)
  video_cfg = cfg.get("video", {})
  outputs_cfg = cfg.get("outputs", {})
  perf_cfg = cfg.get("performance", {})

  cap = open_capture(video_cfg)
  prev_gray = None

  max_fps = int(perf_cfg.get("max_fps", 0))
  target_dt = 1.0 / max_fps if max_fps > 0 else 0.0

  frame_count = 0
  start_time = time.perf_counter()

  show_window = outputs_cfg.get("window", {}).get("enabled", False)
  window_name = "perceptual-drift"

  try:
    while True:
      loop_start = time.perf_counter()
      ok, frame = cap.read()
      if not ok or frame is None:
        raise RuntimeError("Failed to read frame from camera")

      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      drift_value = compute_drift(prev_gray, gray)
      prev_gray = gray

      output_frame = overlay_drift(frame, drift_value)

      frame_count += 1
      elapsed = time.perf_counter() - start_time
      fps = frame_count / elapsed if elapsed > 0 else 0.0
      print(f"[hello] fps={fps:.2f} drift={drift_value:.4f}", end="\r")

      if show_window:
        cv2.imshow(window_name, output_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
          break
      else:
        # Even headless, respect a quit key if available (noop otherwise).
        if cv2.waitKey(1) & 0xFF == ord("q"):
          break

      if target_dt > 0:
        loop_elapsed = time.perf_counter() - loop_start
        remaining = target_dt - loop_elapsed
        if remaining > 0:
          time.sleep(remaining)
  finally:
    cap.release()
    cv2.destroyAllWindows()
    print()  # newline after carriage returns


if __name__ == "__main__":
  main()
