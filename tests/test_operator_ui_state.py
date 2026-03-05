from pathlib import Path
from unittest.mock import patch

from software.operator_ui.state import OperatorState, shape_axis


ROOT = Path(__file__).resolve().parents[1]


def test_shape_axis_deadzone_and_expo_behavior():
    cfg = {"deadzone": 0.1, "curve": "expo", "expo_strength": 0.5}
    assert shape_axis(cfg, 0.05) == 0.0
    shaped = shape_axis(cfg, 0.8)
    assert 0.0 < shaped <= 1.0


def test_operator_state_lists_base_and_recipe_entries():
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
    )
    recipes = state.list_recipes()
    recipe_ids = {r["id"] for r in recipes}
    assert "base" in recipe_ids
    assert "ambient" in recipe_ids


def test_operator_state_recipe_switch_and_snapshot():
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
    )
    snap = state.set_recipe("ambient")
    assert snap["active_recipe"] == "ambient"
    reset = state.set_recipe("base")
    assert reset["active_recipe"] == "base"


def test_operator_state_consent_toggle():
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
    )
    on = state.set_consent(1)
    assert on["consent_state"] == 1
    off = state.set_consent(0)
    assert off["consent_state"] == 0


def test_operator_state_curve_payload_shape():
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
    )
    curves = state.mapping_curves(points=31)
    assert "curves" in curves
    assert "altitude" in curves["curves"]
    assert len(curves["curves"]["altitude"]["series"]) == 31


def test_operator_state_dispatches_consent_to_runtime_targets():
    sent = []

    class DummyClient:
        def __init__(self, host, port):
            self.host = host
            self.port = port

        def send_message(self, route, payload):
            sent.append((self.host, self.port, route, payload))

    with patch("software.operator_ui.state.udp_client.SimpleUDPClient", DummyClient):
        state = OperatorState(
            base_mapping_path=ROOT / "config" / "mapping.yaml",
            recipes_dir=ROOT / "config" / "recipes",
            runtime_targets=[("127.0.0.1", 9000), ("127.0.0.1", 9010)],
        )
        snapshot = state.set_consent(1)
        assert snapshot["last_dispatch"]["action"] == "consent"
        assert len(snapshot["last_dispatch"]["results"]) == 2
        assert sent == [
            ("127.0.0.1", 9000, "/pd/consent", 1),
            ("127.0.0.1", 9010, "/pd/consent", 1),
        ]


def test_operator_state_base_recipe_skips_runtime_patch():
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
        runtime_targets=[("127.0.0.1", 9010)],
    )
    snapshot = state.set_recipe("base")
    assert snapshot["active_recipe"] == "base"
    assert snapshot["last_dispatch"]["action"] == "recipe"
    assert snapshot["last_dispatch"]["results"] == []
