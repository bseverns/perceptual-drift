#!/usr/bin/env python3
"""Minimal OSC tracker for starter-bundle rehearsals.

This keeps onboarding lightweight by offering:
1) synthetic mode (no camera dependency), and
2) camera mode (OpenCV motion/centroid estimate).
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Optional


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class GestureState:
    altitude: float
    lateral: float
    yaw: float
    crowd: float
    consent: int


class CameraTracker:
    """Simple frame-difference tracker for quick starter rehearsals."""

    def __init__(
        self,
        camera_index: int,
        threshold: int,
        consent_threshold: float,
        blur_kernel: int,
    ) -> None:
        try:
            import cv2  # type: ignore
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "camera mode requires OpenCV (cv2). "
                "Install python3-opencv or opencv-python."
            ) from exc

        if blur_kernel % 2 == 0:
            blur_kernel += 1

        self.cv2 = cv2
        self.threshold = threshold
        self.consent_threshold = consent_threshold
        self.blur_kernel = blur_kernel
        self.prev_gray = None

        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera index {camera_index}")

    def sample(self) -> GestureState:
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("Camera read failed")

        cv2 = self.cv2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (self.blur_kernel, self.blur_kernel), 0)

        if self.prev_gray is None:
            self.prev_gray = gray
            return GestureState(0.2, 0.0, 0.0, 0.0, 1)

        diff = cv2.absdiff(self.prev_gray, gray)
        self.prev_gray = gray
        _, mask = cv2.threshold(diff, self.threshold, 255, cv2.THRESH_BINARY)

        motion_ratio = float(mask.mean() / 255.0)
        crowd = clamp(motion_ratio * 4.0, 0.0, 1.0)
        altitude = clamp(0.15 + (crowd * 0.8), 0.0, 1.0)

        moments = cv2.moments(mask, binaryImage=True)
        if moments["m00"] > 0:
            width = mask.shape[1]
            center_x = moments["m10"] / moments["m00"]
            lateral = clamp((center_x / width) * 2.0 - 1.0, -1.0, 1.0)
        else:
            lateral = 0.0
        yaw = clamp(lateral, -1.0, 1.0)
        consent = 1 if crowd >= self.consent_threshold else 0
        return GestureState(altitude, lateral, yaw, crowd, consent)

    def close(self) -> None:
        self.cap.release()


def synthetic_state(
    elapsed: float,
    consent_mode: str,
    consent_cycle_seconds: float,
) -> GestureState:
    altitude = 0.45 + 0.30 * math.sin(elapsed * 0.37)
    lateral = 0.75 * math.sin(elapsed * 0.29)
    yaw = 0.65 * math.cos(elapsed * 0.23 + 1.0)
    crowd = 0.55 + 0.40 * math.sin(elapsed * 0.71 + 0.3)

    if consent_mode == "always_on":
        consent = 1
    elif consent_mode == "always_off":
        consent = 0
    else:
        phase = elapsed % max(consent_cycle_seconds, 1.0)
        consent = 1 if phase < (consent_cycle_seconds / 2.0) else 0

    return GestureState(
        clamp(altitude, 0.0, 1.0),
        clamp(lateral, -1.0, 1.0),
        clamp(yaw, -1.0, 1.0),
        clamp(crowd, 0.0, 1.0),
        consent,
    )


def send_state(client, state: GestureState) -> None:
    client.send_message("/pd/alt", state.altitude)
    client.send_message("/pd/lat", state.lateral)
    client.send_message("/pd/yaw", state.yaw)
    client.send_message("/pd/crowd", state.crowd)
    client.send_message("/pd/consent", state.consent)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Starter-bundle tracker that emits Perceptual Drift OSC routes."
        )
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="OSC target host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9000,
        help="OSC target port (default: 9000)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=20.0,
        help="Send rate in frames/second (default: 20)",
    )
    parser.add_argument(
        "--mode",
        choices=["synthetic", "camera"],
        default="synthetic",
        help="Input mode: synthetic (no camera) or camera (OpenCV).",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=0,
        help="Camera index for OpenCV mode (default: 0)",
    )
    parser.add_argument(
        "--threshold",
        type=int,
        default=24,
        help="Pixel threshold for camera frame differencing (default: 24)",
    )
    parser.add_argument(
        "--blur-kernel",
        type=int,
        default=9,
        help="Gaussian blur kernel size for camera mode (default: 9)",
    )
    parser.add_argument(
        "--consent-threshold",
        type=float,
        default=0.10,
        help="Crowd threshold for consent=1 in camera mode (default: 0.10)",
    )
    parser.add_argument(
        "--consent-mode",
        choices=["always_on", "pulse", "always_off"],
        default="always_on",
        help="Consent behavior in synthetic mode.",
    )
    parser.add_argument(
        "--consent-cycle-seconds",
        type=float,
        default=12.0,
        help="Pulse cycle length in synthetic mode (default: 12s).",
    )
    parser.add_argument(
        "--print-every",
        type=float,
        default=2.0,
        help="Log interval in seconds (default: 2.0).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    interval = 1.0 / max(args.fps, 1.0)
    try:
        from pythonosc import udp_client
    except ImportError:
        print(
            "python-osc is missing. Install with: "
            "python3 -m pip install -r requirements-starter.txt"
        )
        return 1

    client = udp_client.SimpleUDPClient(args.host, args.port)
    tracker: Optional[CameraTracker] = None

    print(
        "[tracker] mode=%s host=%s port=%d fps=%.2f"
        % (args.mode, args.host, args.port, args.fps)
    )

    if args.mode == "camera":
        tracker = CameraTracker(
            camera_index=args.camera_index,
            threshold=args.threshold,
            consent_threshold=args.consent_threshold,
            blur_kernel=args.blur_kernel,
        )

    started = time.monotonic()
    last_print = started
    try:
        while True:
            now = time.monotonic()
            elapsed = now - started
            if tracker is None:
                state = synthetic_state(
                    elapsed=elapsed,
                    consent_mode=args.consent_mode,
                    consent_cycle_seconds=args.consent_cycle_seconds,
                )
            else:
                state = tracker.sample()

            send_state(client, state)

            if now - last_print >= args.print_every:
                print(
                    "[tracker] alt=%.3f lat=%.3f yaw=%.3f crowd=%.3f consent=%d"
                    % (
                        state.altitude,
                        state.lateral,
                        state.yaw,
                        state.crowd,
                        state.consent,
                    )
                )
                last_print = now

            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n[tracker] stopping")
    finally:
        if tracker is not None:
            tracker.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
