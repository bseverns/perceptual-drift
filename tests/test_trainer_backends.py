import struct
from pathlib import Path

from software.trainer_control.backends import (
    DryRunBackend,
    LegacyMSPBackend,
    TrainerBackend,
    encode_trainer_packet,
)
from software.trainer_control.safety import SafetyState
from software.trainer_control.profiles import (
    ProfileError,
    validate_transmitter_profile,
)

import pytest


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.closed = False

    def write(self, payload):
        self.writes.append(payload)

    def close(self):
        self.closed = True


def bounded_state(**overrides):
    values = {
        "roll": 0.15,
        "pitch": 0.0,
        "yaw": -0.15,
        "throttle": 0.0,
        "consent": True,
        "operator_enabled": True,
        "input_fresh": True,
        "heartbeat_fresh": True,
        "active": True,
        "neutral_reason": "none",
        "profile_id": "ez_pilot_pro",
    }
    values.update(overrides)
    return SafetyState(**values)


def test_dry_run_exposes_effective_state_without_hardware():
    output = []
    backend = DryRunBackend(output.append)
    backend.send(bounded_state())
    assert backend.frames == 1
    assert backend.last_state.roll == 0.15
    assert '"profile_id": "ez_pilot_pro"' in output[0]


def test_trainer_packet_is_inspectable_and_forces_zero_throttle():
    packet = encode_trainer_packet(7, bounded_state(throttle=0.9))
    fields = packet.decode().strip().split(",")
    assert fields[:2] == ["PD1", "7"]
    assert fields[2:6] == ["0.150000", "0.000000", "-0.150000", "0.000000"]
    assert len(fields[-1]) == 2


def test_trainer_backend_only_accepts_explicit_bridge_heartbeat():
    serial = FakeSerial()
    backend = TrainerBackend(serial, clock=lambda: 12.5)
    assert backend.last_heartbeat_timestamp is None
    assert backend.observe_line(b"garbage\n") is False
    assert backend.observe_line(b"HB,42\n") is True
    assert backend.last_heartbeat_timestamp == 12.5


def test_legacy_msp_backend_has_no_arm_and_forces_low_throttle():
    serial = FakeSerial()
    backend = LegacyMSPBackend(serial)
    backend.send(bounded_state(throttle=1.0))
    packet = serial.writes[0]
    channels = struct.unpack("<8H", packet[5:21])
    assert channels[2] == 1000
    assert channels[4:] == (1500, 1500, 1500, 1500)
    assert not hasattr(backend, "arm")


def test_transmitter_profile_cannot_raise_trainer_weight(tmp_path):
    profile = Path("config/transmitters/tx16s_pd.yaml")
    unsafe = tmp_path / "unsafe_tx.yaml"
    unsafe.write_text(
        profile.read_text().replace(
            "maximum_weight_percent: 15", "maximum_weight_percent: 20"
        )
    )
    with pytest.raises(ProfileError, match="cannot exceed 15"):
        validate_transmitter_profile(unsafe)
