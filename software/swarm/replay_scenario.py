#!/usr/bin/env python3
"""Replay multi-user gesture scenarios into swarm OSC routes."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

from pythonosc import udp_client

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


@dataclass
class Keyframe:
    t: float
    alt: float
    lat: float
    yaw: float
    crowd: float
    consent: float


@dataclass
class Participant:
    pid: str
    weight: float
    keyframes: List[Keyframe]


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _parse_keyframe(raw: Dict[str, object]) -> Keyframe:
    return Keyframe(
        t=float(raw.get("t", 0.0)),
        alt=float(raw.get("alt", 0.0)),
        lat=float(raw.get("lat", 0.0)),
        yaw=float(raw.get("yaw", 0.0)),
        crowd=float(raw.get("crowd", 0.0)),
        consent=float(raw.get("consent", 1.0)),
    )


def _load_scenario(path: Path) -> Dict[str, object]:
    text = path.read_text()
    if path.suffix.lower() in {".yml", ".yaml"}:
        if yaml is None:
            raise RuntimeError("PyYAML is required for YAML scenario files.")
        data = yaml.safe_load(text)
    else:
        data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError("Scenario must parse to a mapping/object.")
    return data


def _sample_participant(p: Participant, t: float) -> Keyframe:
    frames = p.keyframes
    if not frames:
        return Keyframe(t=t, alt=0.0, lat=0.0, yaw=0.0, crowd=0.0, consent=0.0)
    if t <= frames[0].t:
        return frames[0]
    if t >= frames[-1].t:
        return frames[-1]

    for idx in range(len(frames) - 1):
        a = frames[idx]
        b = frames[idx + 1]
        if a.t <= t <= b.t:
            span = max(b.t - a.t, 1e-9)
            alpha = (t - a.t) / span
            # Consent interpolates numerically so we can use weighted thresholds.
            return Keyframe(
                t=t,
                alt=a.alt + (b.alt - a.alt) * alpha,
                lat=a.lat + (b.lat - a.lat) * alpha,
                yaw=a.yaw + (b.yaw - a.yaw) * alpha,
                crowd=a.crowd + (b.crowd - a.crowd) * alpha,
                consent=a.consent + (b.consent - a.consent) * alpha,
            )
    return frames[-1]


def _aggregate(samples: List[Keyframe], weights: List[float], mode: str) -> Keyframe:
    if not samples:
        return Keyframe(t=0.0, alt=0.0, lat=0.0, yaw=0.0, crowd=0.0, consent=0.0)

    if mode == "max":
        return Keyframe(
            t=samples[0].t,
            alt=max(s.alt for s in samples),
            lat=max(samples, key=lambda s: abs(s.lat)).lat,
            yaw=max(samples, key=lambda s: abs(s.yaw)).yaw,
            crowd=max(s.crowd for s in samples),
            consent=max(s.consent for s in samples),
        )

    denom = max(sum(weights), 1e-9)

    def wsum(field: str) -> float:
        return sum(getattr(s, field) * w for s, w in zip(samples, weights)) / denom

    return Keyframe(
        t=samples[0].t,
        alt=wsum("alt"),
        lat=wsum("lat"),
        yaw=wsum("yaw"),
        crowd=wsum("crowd"),
        consent=wsum("consent"),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay a multi-user gesture scenario into /pd/* OSC routes."
    )
    parser.add_argument("--scenario", required=True, help="Path to scenario yaml/json file")
    parser.add_argument("--host", default="127.0.0.1", help="Target OSC host")
    parser.add_argument("--port", type=int, default=9010, help="Target OSC port")
    parser.add_argument("--loops", type=int, default=1, help="How many times to replay the scenario")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    parser.add_argument(
        "--aggregation",
        choices=["mean", "weighted", "max"],
        default=None,
        help="Override scenario aggregation mode",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print values without sending OSC")
    parser.add_argument("--print-every", type=float, default=1.0, help="Seconds between progress prints")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    scenario_path = Path(args.scenario).expanduser().resolve()
    data = _load_scenario(scenario_path)

    name = str(data.get("name", scenario_path.stem))
    frame_rate = float(data.get("frame_rate", 20.0))
    dt = 1.0 / max(frame_rate, 1.0)
    agg_mode = (args.aggregation or str(data.get("aggregation", "weighted"))).lower()
    if agg_mode == "mean":
        agg_mode = "weighted"
    participants_raw = data.get("participants", [])
    if not isinstance(participants_raw, list) or not participants_raw:
        raise ValueError("Scenario must include a non-empty participants list.")

    participants: List[Participant] = []
    for raw in participants_raw:
        if not isinstance(raw, dict):
            continue
        pid = str(raw.get("id", f"user_{len(participants)+1}"))
        weight = float(raw.get("weight", 1.0))
        keyframes_raw = raw.get("keyframes", [])
        if not isinstance(keyframes_raw, list):
            continue
        keyframes = sorted((_parse_keyframe(kf) for kf in keyframes_raw), key=lambda k: k.t)
        if keyframes:
            participants.append(Participant(pid=pid, weight=weight, keyframes=keyframes))

    if not participants:
        raise ValueError("No valid participants/keyframes found in scenario.")

    duration_s = float(data.get("duration_s", 0.0))
    if duration_s <= 0.0:
        duration_s = max(p.keyframes[-1].t for p in participants)
    duration_s = max(duration_s, dt)

    client = udp_client.SimpleUDPClient(args.host, args.port)
    print(
        f"[replay] scenario='{name}' participants={len(participants)} "
        f"duration={duration_s:.2f}s frame_rate={frame_rate:.1f}Hz mode={agg_mode}"
    )
    print(
        f"[replay] target={args.host}:{args.port} loops={args.loops} speed={args.speed:.2f} dry_run={args.dry_run}"
    )

    last_print = 0.0
    for loop in range(max(args.loops, 1)):
        t = 0.0
        while t <= duration_s + 1e-9:
            samples = [_sample_participant(p, t) for p in participants]
            weights = [max(p.weight, 0.0) for p in participants]
            merged = _aggregate(samples, weights, agg_mode)

            alt = _clamp(merged.alt, 0.0, 1.0)
            lat = _clamp(merged.lat, -1.0, 1.0)
            yaw = _clamp(merged.yaw, -1.0, 1.0)
            crowd = _clamp(merged.crowd, 0.0, 1.0)
            consent = 1 if merged.consent >= 0.5 else 0

            if not args.dry_run:
                client.send_message("/pd/alt", alt)
                client.send_message("/pd/lat", lat)
                client.send_message("/pd/yaw", yaw)
                client.send_message("/pd/crowd", crowd)
                client.send_message("/pd/consent", consent)

            now = time.time()
            if now - last_print >= max(args.print_every, 0.1):
                print(
                    f"[replay] loop={loop+1}/{args.loops} t={t:.2f}s "
                    f"alt={alt:.2f} lat={lat:.2f} yaw={yaw:.2f} crowd={crowd:.2f} consent={consent}"
                )
                last_print = now

            scaled_dt = dt / max(args.speed, 0.05)
            time.sleep(scaled_dt)
            t += dt

    print("[replay] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
