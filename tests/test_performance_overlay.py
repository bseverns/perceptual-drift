from pathlib import Path

import pytest

from software.trainer_control.mapping import MappingController
from software.trainer_control.performance import (
    PerformanceOverlay,
    load_macros,
)
from software.trainer_control.performance_midi import load_performance_midi
from software.trainer_control.runtime import TrackerSignals
from software.trainer_control.safety import SafetyEnvelope


ROOT = Path(__file__).resolve().parents[1]


def test_macros_reject_forbidden_target(tmp_path):
    config = tmp_path / "bad.yaml"
    config.write_text(
        "macros:\n  nope:\n    type: unipolar\n    targets:\n"
        "      - path: bridge.stale_after\n        min: 0\n"
        "        max: 1\n"
    )
    with pytest.raises(ValueError, match="forbidden"):
        load_macros(config)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), "nope"])
def test_macro_values_reject_malformed_input_without_replacing_prior_state(
    value,
):
    overlay = PerformanceOverlay()
    before = overlay.snapshot()["targets"]
    with pytest.raises(ValueError):
        overlay.set({"drift": value})
    assert overlay.snapshot()["targets"] == before


def test_overlay_reset_restores_recipe_mapping():
    now = [0.0]
    mapper = MappingController(
        ROOT / "config/mapping.yaml",
        uniform=lambda *_: 0.0,
        performance=PerformanceOverlay(clock=lambda: now[0]),
    )
    signal = TrackerSignals(0.8, 0.0, 0.0, True, 100.0)
    baseline = mapper.map(signal)
    mapper.set_performance({"drift": 1.0})
    # Advance beyond the 220ms artistic slew.
    now[0] = 1.0
    changed = mapper.map(TrackerSignals(0.8, 0.0, 0.0, True, 101.0))
    mapper.reset_performance()
    restored = mapper.map(signal)
    assert changed.roll != baseline.roll
    assert restored == baseline


def test_first_recipe_relative_drift_touch_has_no_scene_discontinuity():
    now = [100.0]
    mapper = MappingController(
        ROOT / "config/mapping.yaml",
        uniform=lambda *_: 0.0,
        performance=PerformanceOverlay(clock=lambda: now[0]),
    )
    mapper.select("riot_mode")
    signal = TrackerSignals(0.8, 0.0, 0.0, True, now[0])
    before = mapper.map(signal)
    mapper.set_performance({"drift": 0.01})
    after_first_touch = mapper.map(signal)
    assert after_first_touch.roll == pytest.approx(before.roll)
    now[0] += 0.22
    after_slew = mapper.map(TrackerSignals(0.8, 0.0, 0.0, True, now[0]))
    assert abs(after_slew.roll - before.roll) < 0.02


def test_touch_continuously_changes_linear_and_legacy_expo_recipes():
    now = [100.0]
    overlay = PerformanceOverlay(clock=lambda: now[0])
    base = {"mapping": {"lateral": {"curve": "linear", "deadzone": 0.05}}}
    overlay.set({"touch": 1.0})
    now[0] += 0.22
    linear = overlay.apply(base, now=now[0])
    assert linear["mapping"]["lateral"]["curvature"] > 0.0
    expo = {"mapping": {"lateral": {"curve": "expo", "expo_strength": 0.5}}}
    softer = PerformanceOverlay(clock=lambda: now[0])
    softer.set({"touch": -1.0})
    now[0] += 0.22
    assert (
        softer.apply(expo, now=now[0])["mapping"]["lateral"]["curvature"] < 0.5
    )


def test_safety_transitions_are_not_smoothed_by_performance_overlay():
    mapper = MappingController(
        ROOT / "config/mapping.yaml", uniform=lambda *_: 0.0
    )
    mapper.set_performance({"drift": 1.0})
    envelope = SafetyEnvelope.from_path(
        ROOT / "config/aircraft/ez_pilot_pro.yaml"
    )
    intent = mapper.map(TrackerSignals(1.0, 0.0, 0.0, False, 100.0))
    state = envelope.evaluate(
        intent,
        now=100.0,
        operator_enabled=True,
        bridge_heartbeat_timestamp=100.0,
    )
    assert state.active is False
    assert state.neutral_reason == "consent_off"
    assert state.roll == 0.0


def test_performance_midi_cannot_bind_forbidden_controls(tmp_path):
    path = tmp_path / "bad-midi.yaml"
    path.write_text(
        "bindings:\n  - type: cc\n    cc_number: 1\n"
        "    target: macro:consent\n"
    )
    with pytest.raises(ValueError, match="forbidden"):
        load_performance_midi(path)
