import math
from pathlib import Path

from software.trainer_control import runtime


class FakeSerial:
    def __init__(self, lines=()):
        self.lines = list(lines)
        self.writes = []
        self.closed = False

    def readline(self):
        return self.lines.pop(0) if self.lines else b""

    def write(self, payload):
        self.writes.append(payload)

    def close(self):
        self.closed = True


def test_live_source_requires_fresh_roll_yaw_and_consent():
    source = runtime.LiveOscIntentSource(clock=lambda: 100.0)
    source.on_lateral("/pd/lat", 0.8)
    source.on_yaw("/pd/yaw", -0.4)
    assert source.snapshot() is None

    source.on_consent("/pd/consent", 1)
    intent = source.snapshot()

    assert intent is not None
    assert intent.roll == 0.8
    assert intent.pitch == 0.0
    assert intent.yaw == -0.4
    assert intent.throttle == 0.0
    assert intent.consent is True
    assert intent.timestamp == 100.0


def test_live_source_ignores_nonfinite_values_without_refreshing():
    timestamps = iter((10.0, 20.0, 30.0))
    source = runtime.LiveOscIntentSource(clock=lambda: next(timestamps))
    source.on_lateral("/pd/lat", 0.25)
    source.on_yaw("/pd/yaw", -0.5)
    source.on_consent("/pd/consent", 1)
    before = source.snapshot()

    source.on_lateral("/pd/lat", math.nan)
    source.on_yaw("/pd/yaw", math.inf)
    after = source.snapshot()

    assert before == after


def test_heartbeat_reader_accepts_only_bridge_heartbeats():
    serial_port = FakeSerial((b"noise\n", b"HB,42\n", b""))
    backend = runtime.TrainerBackend(serial_port, clock=lambda: 12.5)

    assert runtime._drain_heartbeats(serial_port, backend) == 1
    assert backend.last_heartbeat_timestamp == 12.5


def test_runtime_composes_serial_server_envelope_and_neutral_shutdown(
    monkeypatch,
):
    serial_port = FakeSerial((b"HB,1\n", b""))
    seen = {}

    class FakeServer:
        def __init__(self, address, osc_dispatcher):
            self.server_address = address
            seen["dispatcher"] = osc_dispatcher

        def shutdown(self):
            seen["shutdown"] = True

        def server_close(self):
            seen["closed"] = True

    class FakeThread:
        def join(self, timeout):
            seen["joined"] = timeout

    monkeypatch.setattr(
        runtime.serial, "Serial", lambda *_a, **_k: serial_port
    )
    monkeypatch.setattr(
        runtime.osc_server, "ThreadingOSCUDPServer", FakeServer
    )
    monkeypatch.setattr(runtime, "_start_osc_server", lambda _s: FakeThread())
    monkeypatch.setattr(runtime.time, "sleep", lambda _seconds: None)

    result = runtime.main(
        [
            "--serial",
            "/dev/fake",
            "--max-frames",
            "1",
            "--aircraft-profile",
            str(Path("config/aircraft/ez_pilot_pro.yaml")),
            "--mapping",
            str(Path("config/mapping.yaml")),
        ]
    )

    assert result == 0
    assert len(serial_port.writes) == 2
    for packet in serial_port.writes:
        assert packet.decode().split(",")[2:6] == ["0.000000"] * 4
    assert serial_port.closed is True
    assert seen["shutdown"] is True
    assert seen["closed"] is True
