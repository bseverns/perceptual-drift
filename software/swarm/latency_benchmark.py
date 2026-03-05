#!/usr/bin/env python3
"""Benchmark OSC latency for swarm simulation mode.

This script sends benchmark pings to ``swarm_demo.py --simulate`` and measures
latency statistics from the emitted ack packets.
"""

from __future__ import annotations

import argparse
import json
import statistics
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

from pythonosc import dispatcher, osc_server, udp_client

PING_ROUTE = "/pd/bench/ping"
ACK_ROUTE = "/pd/sim/bench/ack"


@dataclass
class Sample:
    seq: int
    roundtrip_s: float
    sim_processing_s: float


def percentile(values: List[float], pct: float) -> float:
    if not values:
        return 0.0
    if pct <= 0.0:
        return min(values)
    if pct >= 100.0:
        return max(values)
    ordered = sorted(values)
    rank = (len(ordered) - 1) * (pct / 100.0)
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    frac = rank - lower
    return ordered[lower] * (1.0 - frac) + ordered[upper] * frac


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure OSC latency against swarm simulation benchmark hooks."
    )
    parser.add_argument("--target-host", default="127.0.0.1", help="swarm_demo OSC host")
    parser.add_argument("--target-port", type=int, default=9010, help="swarm_demo OSC port")
    parser.add_argument("--listen-host", default="0.0.0.0", help="Local bind host for ack server")
    parser.add_argument("--listen-port", type=int, default=9101, help="Local bind port for ack server")
    parser.add_argument("--samples", type=int, default=200, help="Number of measured ping samples")
    parser.add_argument("--warmup", type=int, default=20, help="Warmup pings before measuring")
    parser.add_argument("--interval", type=float, default=0.03, help="Seconds between pings")
    parser.add_argument("--timeout", type=float, default=2.0, help="Ack timeout in seconds per ping")
    parser.add_argument("--json-out", help="Optional path to write full metrics JSON")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    client = udp_client.SimpleUDPClient(args.target_host, args.target_port)

    lock = threading.Lock()
    pending: Dict[int, float] = {}
    samples: List[Sample] = []
    dropped = 0

    def on_ack(addr: str, *vals) -> None:
        nonlocal dropped
        if len(vals) < 3:
            return
        try:
            seq = int(vals[0])
            recv_rel = float(vals[1])
            emit_rel = float(vals[2])
        except (TypeError, ValueError):
            return
        recv_local = time.monotonic()
        with lock:
            if seq not in pending:
                # Late ack for a timed out sample; keep a counter and ignore.
                dropped += 1
                return
            send_local = pending.pop(seq, None)
            if send_local is None:
                dropped += 1
                return
            sample = Sample(
                seq=seq,
                roundtrip_s=max(0.0, recv_local - send_local),
                sim_processing_s=max(0.0, emit_rel - recv_rel),
            )
            samples.append(sample)

    disp = dispatcher.Dispatcher()
    disp.map(ACK_ROUTE, on_ack)
    server = osc_server.ThreadingOSCUDPServer((args.listen_host, args.listen_port), disp)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    print(
        f"[bench] target={args.target_host}:{args.target_port} "
        f"listen={args.listen_host}:{args.listen_port}"
    )
    print(
        f"[bench] warmup={args.warmup} samples={args.samples} interval={args.interval:.3f}s timeout={args.timeout:.2f}s"
    )

    seq = 0
    total = args.warmup + args.samples
    try:
        for index in range(total):
            now = time.monotonic()
            with lock:
                pending[seq] = now
            client.send_message(PING_ROUTE, [seq])

            deadline = time.time() + args.timeout
            while time.time() < deadline:
                with lock:
                    acked = seq not in pending
                if acked:
                    break
                time.sleep(0.001)

            with lock:
                timed_out = seq in pending
                if timed_out:
                    pending.pop(seq, None)
                    dropped += 1

            seq += 1
            if (index + 1) % 25 == 0:
                print(f"[bench] progress: {index + 1}/{total}")
            time.sleep(max(0.0, args.interval))
    finally:
        server.shutdown()
        server.server_close()

    # Drop warmup samples by seq index.
    measured = [s for s in samples if s.seq >= args.warmup]
    if not measured:
        print("[bench] no measured samples captured; is simulation running?")
        return 1

    rtt = [s.roundtrip_s for s in measured]
    processing = [s.sim_processing_s for s in measured]

    summary = {
        "captured": len(measured),
        "dropped": dropped,
        "roundtrip_ms": {
            "mean": statistics.fmean(rtt) * 1000.0,
            "p50": percentile(rtt, 50.0) * 1000.0,
            "p95": percentile(rtt, 95.0) * 1000.0,
            "max": max(rtt) * 1000.0,
        },
        "sim_processing_ms": {
            "mean": statistics.fmean(processing) * 1000.0,
            "p95": percentile(processing, 95.0) * 1000.0,
        },
    }

    print(
        "[bench] RTT ms: mean={mean:.2f} p50={p50:.2f} p95={p95:.2f} max={max:.2f}".format(
            **summary["roundtrip_ms"]
        )
    )
    print(
        "[bench] Simulation processing p95 ms: {p:.2f}".format(
            p=summary["sim_processing_ms"]["p95"],
        )
    )
    print(f"[bench] captured={summary['captured']} dropped={summary['dropped']}")

    if args.json_out:
        out_path = Path(args.json_out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(summary, indent=2) + "\n")
        print(f"[bench] wrote {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
