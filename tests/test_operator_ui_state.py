import json
import os
from pathlib import Path
from unittest.mock import patch

import pytest

from software.operator_ui.state import (
    CONTROL_BRIDGE_VALIDATOR,
    OperatorState,
    shape_axis,
)


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


def test_operator_state_rejects_invalid_recipe_before_activation(tmp_path):
    recipe = tmp_path / "invalid_expo.yaml"
    recipe.write_text(
        """
mapping:
  altitude:
    curve: expo
    expo_strength: -1.5
""".strip()
    )
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=tmp_path,
    )

    with pytest.raises(
        CONTROL_BRIDGE_VALIDATOR.ValidationError, match="expo_strength"
    ):
        state.set_recipe("invalid_expo")

    snapshot = state.snapshot()
    assert snapshot["active_recipe"] == "base"
    assert state.mapping_curves(points=5)["curves"]["altitude"]["series"]


def test_operator_state_can_pause_participation_without_publishing_consent():
    sent = []

    class DummyClient:
        def __init__(self, *_args):
            pass

        def send_message(self, route, payload):
            sent.append((route, payload))

    with patch(
        "software.operator_ui.state.udp_client.SimpleUDPClient", DummyClient
    ):
        state = OperatorState(
            base_mapping_path=ROOT / "config" / "mapping.yaml",
            recipes_dir=ROOT / "config" / "recipes",
            runtime_targets=[("127.0.0.1", 9000)],
        )
        paused = state.set_participation_pause(True)
        resumed = state.set_participation_pause(False)

    assert paused["participation_paused"] is True
    assert resumed["participation_paused"] is False
    assert sent == [("/pd/operator_pause", 1), ("/pd/operator_pause", 0)]


def test_operator_state_curve_payload_shape():
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
    )
    curves = state.mapping_curves(points=31)
    assert "curves" in curves
    assert "altitude" in curves["curves"]
    assert len(curves["curves"]["altitude"]["series"]) == 31


def test_operator_state_dispatches_participation_pause_to_runtime_targets():
    sent = []

    class DummyClient:
        def __init__(self, host, port):
            self.host = host
            self.port = port

        def send_message(self, route, payload):
            sent.append((self.host, self.port, route, payload))

    with patch(
        "software.operator_ui.state.udp_client.SimpleUDPClient", DummyClient
    ):
        state = OperatorState(
            base_mapping_path=ROOT / "config" / "mapping.yaml",
            recipes_dir=ROOT / "config" / "recipes",
            runtime_targets=[("127.0.0.1", 9000), ("127.0.0.1", 9010)],
        )
        snapshot = state.set_participation_pause(True)
        assert snapshot["last_dispatch"]["action"] == "participation_pause"
        assert len(snapshot["last_dispatch"]["results"]) == 2
        assert sent == [
            ("127.0.0.1", 9000, "/pd/operator_pause", 1),
            ("127.0.0.1", 9010, "/pd/operator_pause", 1),
        ]


def test_operator_state_base_recipe_resets_runtime_patch():
    with patch(
        "software.operator_ui.state.udp_client.SimpleUDPClient"
    ) as client:
        state = OperatorState(
            base_mapping_path=ROOT / "config" / "mapping.yaml",
            recipes_dir=ROOT / "config" / "recipes",
            runtime_targets=[("127.0.0.1", 9010)],
        )
        snapshot = state.set_recipe("base")
    assert snapshot["active_recipe"] == "base"
    assert snapshot["last_dispatch"]["action"] == "recipe"
    assert snapshot["last_dispatch"]["results"][0]["ok"] is True
    client.return_value.send_message.assert_called_once_with(
        "/pd/patch", "base"
    )


def test_operator_state_session_export_includes_state_and_telemetry(tmp_path):
    export_dir = tmp_path / "exports"
    telemetry_file = tmp_path / "telemetry.json"
    telemetry_file.write_text(json.dumps({"p95_ms": 37.2}))

    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
        export_dir=export_dir,
        telemetry_snapshot_file=telemetry_file,
    )

    state.set_recipe("ambient")
    state.set_participation_pause(True)
    session = state.export_session(label="rehearsal_01", notes="dry run")

    out_path = Path(session["path"])
    assert out_path.is_file()
    assert out_path.parent == export_dir
    assert session["bytes"] == out_path.stat().st_size

    exported = json.loads(out_path.read_text())
    assert exported["label"] == "rehearsal_01"
    assert exported["notes"] == "dry run"
    assert exported["state"]["active_recipe"] == "ambient"
    assert exported["state"]["participation_paused"] is True
    assert "rehearsal" in exported
    assert exported["dispatch_history"][-1]["action"] == "participation_pause"
    assert exported["telemetry_snapshot"]["ok"] is True
    assert exported["telemetry_snapshot"]["data"]["p95_ms"] == 37.2
    assert "altitude" in exported["curves"]["curves"]

    latest = state.latest_export()
    assert latest["ok"] is True
    assert latest["exists"] is True
    assert latest["path"] == session["path"]


def test_operator_state_latest_export_and_missing_telemetry(tmp_path):
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
        export_dir=tmp_path / "exports",
        telemetry_snapshot_file=tmp_path / "missing.json",
    )

    assert state.latest_export()["ok"] is False
    session = state.export_session()

    exported = json.loads(Path(session["path"]).read_text())
    assert exported["telemetry_snapshot"]["ok"] is False
    assert exported["telemetry_snapshot"]["reason"] == "missing_file"


def test_live_flow_rejects_stale_trainer_telemetry(tmp_path):
    telemetry_file = tmp_path / "trainer.json"
    telemetry_file.write_text(
        json.dumps({"observed_at": 1.0, "signal_flow": {"active": True}})
    )
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
        telemetry_snapshot_file=telemetry_file,
    )
    with patch("software.operator_ui.state.time.time", return_value=3.0):
        flow = state.live_signal_flow()
    assert flow["available"] is False
    assert flow["reason"] == "stale"


def test_operator_state_session_exports_do_not_overwrite_same_second(tmp_path):
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
        export_dir=tmp_path / "exports",
    )

    with patch(
        "software.operator_ui.state.time.strftime",
        return_value="20260818_123149",
    ):
        first = state.export_session(label="double", notes="first")
        second = state.export_session(label="double", notes="second")

    first_path = Path(first["path"])
    second_path = Path(second["path"])
    assert first_path != second_path
    assert first_path.name == "session_20260818_123149_double.json"
    assert second_path.name == "session_20260818_123149_double_2.json"
    assert json.loads(first_path.read_text())["notes"] == "first"
    assert json.loads(second_path.read_text())["notes"] == "second"


def test_operator_state_runtime_health_uses_pid_file(tmp_path):
    pid_file = tmp_path / "bridge.pid"
    pid_file.write_text(str(os.getpid()))
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
        runtime_services=[
            {
                "id": "bridge",
                "name": "Bridge",
                "pid_file": str(pid_file),
                "match": "software/control-bridge/osc_msp_bridge.py",
            }
        ],
    )
    with patch.object(
        state,
        "_ps_processes",
        return_value=[
            (
                os.getpid(),
                "python software/control-bridge/osc_msp_bridge.py",
            )
        ],
    ):
        health = state.runtime_health()
    assert health["healthy"] == 1
    assert health["total"] == 1
    service = health["services"][0]
    assert service["healthy"] is True
    assert service["source"] == "pid_file"
    assert service["pid"] == os.getpid()


def test_operator_state_runtime_health_rejects_reused_pid_and_scans(tmp_path):
    pid_file = tmp_path / "bridge.pid"
    pid_file.write_text("1234")
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
        runtime_services=[
            {
                "id": "bridge",
                "name": "Bridge",
                "pid_file": str(pid_file),
                "match": "software/control-bridge/osc_msp_bridge.py",
            }
        ],
    )
    processes = [
        (1234, "python unrelated_service.py"),
        (5678, "python software/control-bridge/osc_msp_bridge.py"),
    ]
    with (
        patch.object(state, "_pid_running", return_value=True),
        patch.object(state, "_ps_processes", return_value=processes),
    ):
        health = state.runtime_health()

    service = health["services"][0]
    assert service["healthy"] is True
    assert service["source"] == "process_scan"
    assert service["pid"] == 5678


def test_operator_state_runtime_health_falls_back_to_process_scan():
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
        runtime_services=[
            {
                "id": "swarm",
                "name": "Swarm Demo",
                "pid_file": "",
                "match": "software/swarm/swarm_demo.py",
            }
        ],
    )
    with patch.object(
        state,
        "_ps_processes",
        return_value=[
            (2222, "python software/swarm/swarm_demo.py --simulate")
        ],
    ):
        health = state.runtime_health()
    service = health["services"][0]
    assert service["healthy"] is True
    assert service["source"] == "process_scan"
    assert service["pid"] == 2222


def test_default_runtime_health_includes_reference_trainer():
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
    )
    with patch.object(state, "_ps_processes", return_value=[]):
        health = state.runtime_health()

    trainer = next(
        service
        for service in health["services"]
        if service["id"] == "trainer_runtime"
    )
    assert trainer["name"] == "Trainer Runtime"
    assert trainer["healthy"] is False


def test_operator_state_rehearsal_lifecycle_updates_snapshot():
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
    )
    preflight = {"ok": True, "required_failures": 0, "warnings": 1}
    session = state.set_rehearsal_preflight(preflight)
    assert session["last_preflight"]["ok"] is True

    started = state.start_rehearsal(
        label="friday_soundcheck",
        profile_id="safe_synthetic",
        notes="lights pass",
        start_options={"tracker_mode": "synthetic"},
        preflight=preflight,
    )
    assert started["active"] is True
    assert started["label"] == "friday_soundcheck"
    assert started["profile_id"] == "safe_synthetic"

    snap = state.snapshot()
    assert snap["rehearsal"]["active"] is True
    assert snap["rehearsal"]["label"] == "friday_soundcheck"
    assert snap["rehearsal"]["preflight_ok"] is True

    stopped = state.stop_rehearsal()
    assert stopped["active"] is False
