import math
from dataclasses import replace
from pathlib import Path

import pytest

from software.trainer_control.intent import ControlIntent
from software.trainer_control.safety import SafetyEnvelope


PROFILE = Path("config/aircraft/ez_pilot_pro.yaml")
NOW = 100.0


@pytest.fixture
def envelope():
    loaded = SafetyEnvelope.from_path(PROFILE)
    assert loaded.profile_error is None
    return loaded


@pytest.fixture
def active_intent():
    return ControlIntent(
        roll=0.8,
        pitch=0.8,
        yaw=-0.8,
        throttle=1.0,
        crowd=0.7,
        consent=True,
        timestamp=NOW,
    )


def evaluate(envelope, intent, **overrides):
    values = {
        "now": NOW,
        "operator_enabled": True,
        "bridge_heartbeat_timestamp": NOW,
    }
    values.update(overrides)
    return envelope.evaluate(intent, **values)


def assert_zero(state, reason):
    assert (state.roll, state.pitch, state.yaw, state.throttle) == (0, 0, 0, 0)
    assert state.active is False
    assert state.neutral_reason == reason


def test_consent_off_is_explicit_safety_neutral(envelope, active_intent):
    state = evaluate(envelope, replace(active_intent, consent=False))
    assert_zero(state, "consent_off")


def test_stale_input_is_explicit_safety_neutral(envelope, active_intent):
    state = evaluate(envelope, active_intent, now=NOW + 1)
    assert_zero(state, "stale_input")


def test_startup_before_valid_input_is_neutral(envelope):
    state = evaluate(envelope, None)
    assert_zero(state, "startup_before_valid_input")


@pytest.mark.parametrize(
    "raw",
    [
        "not a mapping",
        {"roll": "left", "timestamp": NOW},
        {"roll": math.nan, "timestamp": NOW},
        {"yaw": math.inf, "timestamp": NOW},
        {"arm": True, "timestamp": NOW},
    ],
)
def test_malformed_or_nonfinite_data_fails_safe(envelope, raw):
    state = envelope.evaluate_raw(
        raw,
        now=NOW,
        operator_enabled=True,
        bridge_heartbeat_timestamp=NOW,
    )
    assert_zero(state, "malformed_input")


def test_recipe_scale_cannot_exceed_aircraft_limits(envelope):
    exaggerated_recipe_output = ControlIntent(
        roll=1.0,
        pitch=-1.0,
        yaw=-1.0,
        throttle=1.0,
        crowd=1.0,
        consent=True,
        timestamp=NOW,
    )
    state = evaluate(envelope, exaggerated_recipe_output)
    assert state.roll == 0.15
    assert state.pitch == 0
    assert state.yaw == -0.15
    assert state.throttle == 0


def test_throttle_is_always_zero(envelope, active_intent):
    assert evaluate(envelope, active_intent).throttle == 0


def test_operator_disable_is_neutral(envelope, active_intent):
    state = evaluate(envelope, active_intent, operator_enabled=False)
    assert_zero(state, "operator_enable_false")


def test_lost_bridge_heartbeat_is_neutral(envelope, active_intent):
    state = evaluate(envelope, active_intent, bridge_heartbeat_timestamp=None)
    assert_zero(state, "trainer_bridge_heartbeat_lost")
    stale = evaluate(
        envelope,
        active_intent,
        bridge_heartbeat_timestamp=NOW - 1,
    )
    assert_zero(stale, "trainer_bridge_heartbeat_lost")


def test_profile_validation_failure_is_neutral(tmp_path, active_intent):
    bad_profile = tmp_path / "bad.yaml"
    bad_profile.write_text("kind: aircraft\nsafety: nope\n")
    envelope = SafetyEnvelope.from_path(bad_profile)
    state = evaluate(envelope, active_intent)
    assert_zero(state, "safety_profile_validation_failure")


def test_aircraft_profile_cannot_raise_initial_roll_limit(
    tmp_path, active_intent
):
    unsafe_profile = tmp_path / "unsafe.yaml"
    unsafe_profile.write_text(
        PROFILE.read_text().replace("roll: 0.15", "roll: 0.16")
    )
    envelope = SafetyEnvelope.from_path(unsafe_profile)
    state = evaluate(envelope, active_intent)
    assert_zero(state, "safety_profile_validation_failure")


def test_intent_api_has_no_arm_field():
    assert "arm" not in ControlIntent.__dataclass_fields__
