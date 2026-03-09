import json
import os
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

    with patch(
        "software.operator_ui.state.udp_client.SimpleUDPClient", DummyClient
    ):
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
    state.set_consent(1)
    session = state.export_session(label="rehearsal_01", notes="dry run")

    out_path = Path(session["path"])
    assert out_path.is_file()
    assert out_path.parent == export_dir
    assert session["bytes"] == out_path.stat().st_size

    exported = json.loads(out_path.read_text())
    assert exported["label"] == "rehearsal_01"
    assert exported["notes"] == "dry run"
    assert exported["state"]["active_recipe"] == "ambient"
    assert exported["state"]["consent_state"] == 1
    assert "rehearsal" in exported
    assert exported["dispatch_history"][-1]["action"] == "consent"
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
                "match": "does-not-matter",
            }
        ],
    )
    with patch.object(state, "_ps_processes", return_value=[]):
        health = state.runtime_health()
    assert health["healthy"] == 1
    assert health["total"] == 1
    service = health["services"][0]
    assert service["healthy"] is True
    assert service["source"] == "pid_file"
    assert service["pid"] == os.getpid()


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
