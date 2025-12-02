#!/usr/bin/env python3
"""Quick camera probe for desktops and Jetson targets.
Opens a handful of VideoCapture indices and reports whether frames can be read.
This is intentionally minimal so it can run before the rest of the stack is ready."""

import cv2


MAX_INDEX = 5


def main() -> None:
  print("[check] probing camera indices 0..{0}".format(MAX_INDEX))
  for index in range(MAX_INDEX + 1):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
      print(f"[check] index {index}: not available (could not open)")
      continue

    ok, frame = cap.read()
    if not ok or frame is None:
      print(f"[check] index {index}: opened but no frames returned")
    else:
      h, w = frame.shape[:2]
      print(f"[check] index {index}: OK ({w}x{h})")

    cap.release()


if __name__ == "__main__":
  main()
