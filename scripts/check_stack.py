#!/usr/bin/env python3
"""End-to-end control stack check for Perceptual Drift.

This harness pretends to be the whole squad: OSC crowd tracker, MSP flight
bridge, LED strip, and Teensy DSP.  Run it before showtime to catch broken
serial paths or configuration drift without pulling props out of the pelican
case.
"""
from __future__ import annotations

import argparse
import json
import queue
import random
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Tuple

import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import yaml
from pythonosc import dispatcher, udp_client
from pythonosc.osc_server import ThreadingOSCUDPServer

import importlib.util

BRIDGE_PATH = REPO_ROOT / "software" / "control-bridge" / "osc_msp_bridge.py"
spec = importlib.util.spec_from_file_location("osc_msp_bridge", BRIDGE_PATH)
if spec is None or spec.loader is None:  # pragma: no cover - sanity guard
    raise RuntimeError(f"Unable to load osc_msp_bridge from {BRIDGE_PATH}")
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


MSP_HEADER = bridge.MSP_HEADER
MSP_SET_RAW_RC = bridge.MSP_SET_RAW_RC


class MSPSerialLoopback:
    """Capture MSP frames so we can inspect what the bridge is sending."""

    def __init__(self) -> None:
        self._writes: List[bytes] = []
        self._lock = threading.Lock()

    def write(self, data: bytes) -> int:
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError("MSP link expects bytes")
        with self._lock:
            self._writes.append(bytes(data))
        return len(data)

    def flush(self) -> None:
        return None

    def close(self) -> None:  # pragma: no cover - symmetry with real serial
        return None

    def packets(self) -> List[bytes]:
        with self._lock:
            return list(self._writes)


class TeensyLEDMock:
    """Mimic the teensy-led sketch so higher layers can rehearse commands."""

    def __init__(self) -> None:
        self.base_rgb = (0, 0, 32)
        self.intensity = 128
        self._outbox: "queue.Queue[bytes]" = queue.Queue()

    def write(self, data: bytes) -> int:
        text = data.decode("utf-8")
        for line in text.splitlines():
            if not line:
                continue
            head = line[0]
            if head == "C":
                try:
                    _, r, g, b = line.split()
                    self.base_rgb = tuple(max(0, min(255, int(v))) for v in (r, g, b))
                except ValueError:
                    self._outbox.put(b"ERR led parse\n")
            elif head == "I":
                try:
                    _, x = line.split()
                    self.intensity = max(0, min(255, int(x)))
                except ValueError:
                    self._outbox.put(b"ERR led intensity\n")
            elif line == "PING":
                self._outbox.put(b"LED-HEARTBEAT\n")
            else:
                self._outbox.put(b"ERR led unknown\n")
        return len(data)

    def readline(self) -> bytes:
        try:
            return self._outbox.get_nowait()
        except queue.Empty:
            return b""


class TeensyDSPMock:
    """Heartbeat-only DSP stub — mirrors the Mozzi sketch envelope."""

    def __init__(self) -> None:
        self._outbox: "queue.Queue[bytes]" = queue.Queue()

    def write(self, data: bytes) -> int:
        text = data.decode("utf-8")
        for line in text.splitlines():
            if line == "PING":
                self._outbox.put(b"DSP-HEARTBEAT\n")
            else:
                self._outbox.put(b"ERR dsp unknown\n")
        return len(data)

    def readline(self) -> bytes:
        try:
            return self._outbox.get_nowait()
        except queue.Empty:
            return b""


@dataclass
class FixtureFrame:
    lat: float
    alt: float
    yaw: float
    crowd: float


def load_fixture_vectors(path: Path, threshold: int = 35) -> List[FixtureFrame]:
    """Translate the prerecorded JSON fixture into gesture vectors.

    The math mirrors the Processing sketch in
    ``software/gesture-tracking/processing/PerceptualDrift_Tracker``.  We don't
    care about drawing, just the pixel diff/centroid logic that yields
    ``lat``, ``alt``, ``yaw``, and ``crowd`` values.
    """

    payload = json.loads(path.read_text())
    frames: List[List[List[int]]] = payload["frames"]
    width = payload["meta"]["width"]
    height = payload["meta"]["height"]

    def brightness(val: int) -> float:
        return float(val)

    vectors: List[FixtureFrame] = []
    prev = frames[0]
    sample_step = max(1, width // 16)

    for frame in frames[1:]:
        motion_count = 0
        diff_map = [[0 for _ in range(width)] for _ in range(height)]
        for y in range(height):
            for x in range(width):
                d = abs(brightness(frame[y][x]) - brightness(prev[y][x]))
                if d > threshold:
                    diff_map[y][x] = 1
                    motion_count += 1
        cx = 0.0
        cy = 0.0
        active = 0
        for y in range(0, height, sample_step):
            for x in range(0, width, sample_step):
                if diff_map[y][x]:
                    cx += x
                    cy += y
                    active += 1
        if active:
            cx /= active
            cy /= active
            norm_x = (cx / float(width)) * 2 - 1
            norm_y = (cy / float(height)) * 2 - 1
        else:
            norm_x = 0.0
            norm_y = 0.0
        lat = bridge.clamp(norm_x, -1.0, 1.0)
        alt = bridge.clamp(-norm_y, -1.0, 1.0)
        yaw = bridge.clamp(lat * 0.2, -1.0, 1.0)
        crowd = bridge.clamp((motion_count / float(width * height)) * 5.0, 0.0, 1.0)
        vectors.append(FixtureFrame(lat=lat, alt=alt, yaw=yaw, crowd=crowd))
        prev = frame
    return vectors


class BridgeHarness:
    """Minimal copy of osc_msp_bridge.main with a deterministic serial link."""

    def __init__(self, mapping_path: Path, serial_link: MSPSerialLoopback, *, osc_port: int) -> None:
        cfg = yaml.safe_load(mapping_path.read_text())
        cfg = json.loads(json.dumps(cfg))  # deep copy without yaml nodes
        cfg.setdefault("mapping", {}).setdefault("yaw_bias", {}).setdefault("jitter", 0.0)
        cfg["mapping"]["yaw_bias"]["jitter"] = 0.0
        self.cfg = cfg
        self.mapper = bridge.Mapper(cfg)
        self.serial = serial_link
        self.osc_port = osc_port
        self._stop = threading.Event()
        self._started = threading.Event()
        self._loop_thread: threading.Thread | None = None
        self._osc_thread: threading.Thread | None = None
        self._server: ThreadingOSCUDPServer | None = None
        self._last = 0.0

        addresses = cfg["osc"]["address_space"]
        disp = dispatcher.Dispatcher()
        disp.map(addresses["altitude"], self._mk_handler("alt", -1.0, 1.0))
        disp.map(addresses["lateral"], self._mk_handler("lat", -1.0, 1.0))
        disp.map(addresses["yaw"], self._mk_handler("yaw", -1.0, 1.0))
        disp.map(addresses["crowd"], self._mk_handler("crowd", 0.0, 1.0))
        disp.map(addresses["consent"], self._consent_handler)
        self.addresses = addresses
        self._dispatcher = disp

    def _mk_handler(self, key: str, lo: float, hi: float):
        def handler(_addr: str, value: float) -> None:
            self.mapper.state[key] = float(bridge.clamp(value, lo, hi))

        return handler

    def _consent_handler(self, _addr: str, value: float) -> None:
        prev = self.mapper.state["consent"]
        self.mapper.state["consent"] = int(value)
        if self.mapper.state["consent"] != 1 and prev == 1:
            self.mapper.state.update({"alt": 0.0, "lat": 0.0, "yaw": 0.0, "crowd": 0.0})

    def start(self) -> None:
        self._stop.clear()
        self._server = ThreadingOSCUDPServer(("127.0.0.1", self.osc_port), self._dispatcher)
        self._server.daemon_threads = True
        # Bind to an ephemeral port when osc_port == 0 so tests/CI don't clash
        # with a real bridge session.
        self.osc_port = self._server.server_address[1]
        self._osc_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._osc_thread.start()
        self._loop_thread = threading.Thread(target=self._loop, daemon=True)
        self._loop_thread.start()
        self._started.set()

    @property
    def listening_port(self) -> int:
        return self.osc_port

    def wait_ready(self, timeout: float = 1.0) -> bool:
        return self._started.wait(timeout)

    def _loop(self) -> None:
        neutral_rc = (
            bridge.map_float_to_rc(0.0),
            bridge.map_float_to_rc(0.0),
            bridge.map_float_to_rc(-1.0),
            bridge.map_float_to_rc(0.0),
        )
        neutral_aux = (1500, 1500, 1500, 1500)
        while not self._stop.is_set():
            now = time.time()
            if now - self._last > 0.02:
                rc_channels, aux_channels = self.mapper.apply()
                payload = struct.pack(
                    "<8H",
                    rc_channels[0],
                    rc_channels[1],
                    rc_channels[2],
                    rc_channels[3],
                    aux_channels[0],
                    aux_channels[1],
                    aux_channels[2],
                    aux_channels[3],
                )
                pkt = bridge.msp_packet(MSP_SET_RAW_RC, payload)
                if self.mapper.state["consent"] == 1:
                    self.serial.write(pkt)
                else:
                    chill_payload = struct.pack(
                        "<8H",
                        neutral_rc[0],
                        neutral_rc[1],
                        neutral_rc[2],
                        neutral_rc[3],
                        neutral_aux[0],
                        neutral_aux[1],
                        neutral_aux[2],
                        neutral_aux[3],
                    )
                    self.serial.write(bridge.msp_packet(MSP_SET_RAW_RC, chill_payload))
                self._last = now
            time.sleep(0.005)

    def stop(self) -> None:
        self._stop.set()
        if self._server:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        if self._loop_thread:
            self._loop_thread.join(timeout=1)
            self._loop_thread = None
        if self._osc_thread:
            self._osc_thread.join(timeout=1)
            self._osc_thread = None
        self._started.clear()


def decode_msp_frame(frame: bytes) -> Tuple[Tuple[int, ...], Tuple[int, ...]]:
    if len(frame) < 6:
        raise ValueError("Frame too short to be MSP")
    if not frame.startswith(MSP_HEADER):
        raise ValueError("Bad MSP header")
    size = frame[3]
    cmd = frame[4]
    payload = frame[5 : 5 + size]
    checksum = frame[5 + size]
    if cmd != MSP_SET_RAW_RC:
        raise ValueError(f"Unexpected MSP command {cmd}")
    calc = (size ^ cmd ^ sum(payload)) & 0xFF
    if checksum != calc:
        raise ValueError("Checksum mismatch")
    channels = struct.unpack("<8H", payload)
    return channels[:4], channels[4:]


def assert_msp_activity(serial_link: MSPSerialLoopback) -> None:
    frames = serial_link.packets()
    if not frames:
        raise AssertionError("Bridge never emitted MSP frames — check OSC input")
    decoded = [decode_msp_frame(pkt) for pkt in frames]
    rc_first, aux_first = decoded[-1]
    if not (1100 <= rc_first[0] <= 1900):
        raise AssertionError(f"Roll channel outside expected range: {rc_first[0]}")
    if not (1100 <= aux_first[0] <= 1900):
        raise AssertionError(f"AUX1 channel outside expected range: {aux_first[0]}")


def run_led_mock_check() -> None:
    led = TeensyLEDMock()
    led.write(b"C 255 64 0\n")
    led.write(b"I 200\n")
    led.write(b"PING\n")
    hb = led.readline()
    if hb.strip() != b"LED-HEARTBEAT":
        raise AssertionError("LED mock heartbeat missing")
    if led.base_rgb != (255, 64, 0) or led.intensity != 200:
        raise AssertionError("LED mock failed to latch color/intensity")


def run_dsp_mock_check() -> None:
    dsp = TeensyDSPMock()
    dsp.write(b"PING\n")
    hb = dsp.readline()
    if hb.strip() != b"DSP-HEARTBEAT":
        raise AssertionError("DSP mock heartbeat missing")


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Spin up a control stack loopback smoke test.")

    default_mapping = (REPO_ROOT / "config" / "mapping.yaml").resolve()
    default_fixture = (REPO_ROOT / "config" / "test-fixtures" / "gesture_fixture_frames.json").resolve()

    parser.add_argument(
        "--mapping",
        default=str(default_mapping),
        help=(
            "Path to the mapping YAML. Relative paths are resolved from the repo root"
            " (defaults to config/mapping.yaml)."
        ),
    )
    parser.add_argument(
        "--fixture",
        default=str(default_fixture),
        help=(
            "Gesture fixture JSON for the camera stub. Relative paths are resolved"
            " from the repo root (defaults to config/test-fixtures/gesture_fixture_frames.json)."
        ),
    )
    parser.add_argument("--osc-port", type=int, default=9100, help="OSC port for the harness (0 = auto)")
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Limit how many fixture frames get replayed (handy for fast CI runs)",
    )
    parser.add_argument(
        "--send-interval",
        type=float,
        default=0.03,
        help="Delay between OSC fixture sends in seconds",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=0.1,
        help="Warmup sleep after the harness boots so the OSC server is ready",
    )
    parser.add_argument(
        "--cooldown",
        type=float,
        default=0.2,
        help="Post-stream pause to let the bridge emit its final MSP frame",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    def resolve_repo_path(value: str) -> Path:
        path = Path(value)
        if path.is_absolute():
            return path
        return (REPO_ROOT / path).resolve()

    random.seed(0)

    fixture_path = resolve_repo_path(args.fixture)
    mapping_path = resolve_repo_path(args.mapping)

    fixture_vectors = load_fixture_vectors(fixture_path)
    if args.max_frames:
        fixture_vectors = fixture_vectors[: args.max_frames]
    serial_link = MSPSerialLoopback()
    harness = BridgeHarness(mapping_path, serial_link, osc_port=args.osc_port)
    harness.start()
    harness.wait_ready()
    client = udp_client.SimpleUDPClient("127.0.0.1", harness.listening_port)
    time.sleep(max(0.0, args.warmup))

    client.send_message(harness.addresses["consent"], 1)
    time.sleep(max(0.0, min(args.send_interval, 0.05)))
    for vec in fixture_vectors:
        client.send_message(harness.addresses["lateral"], vec.lat)
        client.send_message(harness.addresses["altitude"], vec.alt)
        client.send_message(harness.addresses["yaw"], vec.yaw)
        client.send_message(harness.addresses["crowd"], vec.crowd)
        time.sleep(max(0.0, args.send_interval))

    time.sleep(max(0.0, args.cooldown))
    harness.stop()

    assert_msp_activity(serial_link)
    run_led_mock_check()
    run_dsp_mock_check()

    print("✅ MSP bridge emitted frames and decoded without checksum drama.")
    print("✅ Gesture fixture drove OSC handlers and MSP payloads.")
    print("✅ LED + DSP mocks answered heartbeat pings.")
    print("All green. Go run the real rig.")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
