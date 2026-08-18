import math
from pathlib import Path

import pytest

from software.trainer_control import runtime
from software.trainer_control.mapping import MappingController
from software.trainer_control.safety import SafetyEnvelope


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
    assert intent.lateral == 0.8
    assert intent.yaw == -0.4
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


def test_heartbeat_reader_retains_split_nonblocking_lines():
    serial_port = FakeSerial((b"HB,", b"42\n", b""))
    backend = runtime.TrainerBackend(serial_port, clock=lambda: 12.5)
    line_buffer = runtime.HeartbeatLineBuffer()

    assert runtime._drain_heartbeats(serial_port, backend, line_buffer) == 1
    assert backend.last_heartbeat_timestamp == 12.5


def test_artistic_mapping_applies_deadzone_gain_bias_and_recipe():
    base = MappingController(
        Path("config/mapping.yaml"), uniform=lambda _a, _b: 0.0
    )
    signals = runtime.TrackerSignals(0.08, -0.04, 0.5, True, 100.0)

    base_intent = base.map(signals)
    assert base_intent.roll == pytest.approx((0.08 - 0.05) / 0.95 * 0.5)
    assert base_intent.yaw == pytest.approx(0.06)

    base.select("ambient")
    ambient_intent = base.map(signals)
    assert ambient_intent.roll == 0.0
    assert ambient_intent.yaw == pytest.approx(0.01)


def test_invalid_recipe_is_rejected_without_replacing_active_mapper(tmp_path):
    mapper = MappingController(
        Path("config/mapping.yaml"), uniform=lambda _a, _b: 0.0
    )
    signals = runtime.TrackerSignals(0.5, 0.0, 0.5, True, 100.0)
    before = mapper.map(signals)
    invalid = tmp_path / "invalid.yaml"
    base_path = Path("config/mapping.yaml").resolve()
    invalid.write_text(
        "control_bridge:\n"
        f"  extends: {base_path}\n"
        "  mapping:\n"
        "    lateral:\n"
        "      gain: .nan\n"
    )

    with pytest.raises(ValueError, match="gain"):
        mapper.select(str(invalid))

    assert mapper.active_recipe == "base"
    assert mapper.map(signals) == before


@pytest.mark.parametrize("name", ["../ambient", "/tmp/a", "ambient.yaml", ""])
def test_live_recipe_route_accepts_names_not_paths(name):
    assert runtime._is_safe_recipe_name(name) is False


def test_state_reporter_logs_only_effective_transitions():
    events = []

    class Audit:
        def write(self, *args, **kwargs):
            events.append((args, kwargs))

    messages = []
    reporter = runtime.StateTransitionReporter(Audit(), messages.append)
    neutral = runtime.SafetyState.neutral("consent_off", "ez_pilot_pro")
    active = runtime.SafetyState(
        active=True,
        consent=True,
        operator_enabled=True,
        input_fresh=True,
        heartbeat_fresh=True,
        neutral_reason="none",
        profile_id="ez_pilot_pro",
    )

    assert reporter.report(neutral) is True
    assert reporter.report(neutral) is False
    assert reporter.report(active) is True
    assert messages == [
        "TRAINER STATE → NEUTRAL: consent_off",
        "TRAINER STATE → ACTIVE",
    ]
    assert len(events) == 2
    assert events[0][0][0] == "trainer_state_transition"


def test_complete_live_authority_lifecycle_reaches_active_serial_packet():
    now = [100.0]
    source = runtime.LiveOscIntentSource(clock=lambda: now[0])
    source.on_lateral("/pd/lat", 0.08)
    source.on_yaw("/pd/yaw", -0.04)
    source.on_consent("/pd/consent", 1)
    mapper = MappingController(
        Path("config/mapping.yaml"), uniform=lambda _a, _b: 0.0
    )
    serial_port = FakeSerial()
    backend = runtime.TrainerBackend(serial_port, clock=lambda: now[0])
    backend.observe_line(b"HB,1\n")
    envelope = SafetyEnvelope.from_path(
        Path("config/aircraft/ez_pilot_pro.yaml")
    )

    class Audit:
        def write(self, *_args, **_kwargs):
            pass

    messages = []
    reporter = runtime.StateTransitionReporter(Audit(), messages.append)

    def evaluate(operator_enabled=True):
        return runtime._evaluate_frame(
            source=source,
            mapper=mapper,
            envelope=envelope,
            backend=backend,
            reporter=reporter,
            now=now[0],
            operator_enabled=operator_enabled,
        )

    active = evaluate()
    fields = serial_port.writes[-1].decode().strip().split(",")
    assert active.active is True
    assert float(fields[2]) == pytest.approx(
        (0.08 - 0.05) / 0.95 * 0.5, abs=1e-6
    )
    assert float(fields[4]) == pytest.approx(0.06, abs=1e-6)
    assert messages == ["TRAINER STATE → ACTIVE"]

    now[0] = 100.4
    assert evaluate().neutral_reason == "stale_input"

    now[0] = 101.0
    source.on_lateral("/pd/lat", 0.08)
    source.on_yaw("/pd/yaw", -0.04)
    source.on_consent("/pd/consent", 0)
    backend.observe_line(b"HB,2\n")
    assert evaluate().neutral_reason == "consent_off"

    source.on_consent("/pd/consent", 1)
    now[0] = 101.3
    source.on_lateral("/pd/lat", 0.08)
    source.on_yaw("/pd/yaw", -0.04)
    source.on_consent("/pd/consent", 1)
    assert evaluate().neutral_reason == "trainer_bridge_heartbeat_lost"

    backend.observe_line(b"HB,3\n")
    assert (
        evaluate(operator_enabled=False).neutral_reason
        == "operator_enable_false"
    )

    assert all(
        packet.decode().split(",")[2:6] == ["0.000000"] * 4
        for packet in serial_port.writes[1:]
    )


@pytest.mark.parametrize("stale_field", ["roll", "yaw", "consent"])
def test_each_required_live_input_expires_independently(stale_field):
    timestamp = [200.0]
    source = runtime.LiveOscIntentSource(clock=lambda: timestamp[0])
    handlers = {
        "roll": lambda: source.on_lateral("/pd/lat", 0.08),
        "yaw": lambda: source.on_yaw("/pd/yaw", -0.04),
        "consent": lambda: source.on_consent("/pd/consent", 1),
    }
    for name, handler in handlers.items():
        timestamp[0] = 199.5 if name == stale_field else 200.0
        handler()

    mapper = MappingController(Path("config/mapping.yaml"))
    serial_port = FakeSerial()
    backend = runtime.TrainerBackend(serial_port, clock=lambda: 200.0)
    backend.observe_line(b"HB,1\n")
    envelope = SafetyEnvelope.from_path(
        Path("config/aircraft/ez_pilot_pro.yaml")
    )

    class Reporter:
        def report(self, state):
            return state

    state = runtime._evaluate_frame(
        source=source,
        mapper=mapper,
        envelope=envelope,
        backend=backend,
        reporter=Reporter(),
        now=200.0,
        operator_enabled=True,
    )

    assert state.neutral_reason == "stale_input"
    assert serial_port.writes[-1].decode().split(",")[2:6] == ["0.000000"] * 4


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
