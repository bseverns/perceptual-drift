import json
import sys
import time
from pathlib import Path

from pythonosc import udp_client

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import scripts.check_stack as check_stack

FIXTURE = Path("config/test-fixtures/gesture_fixture_frames.json")
MAPPING = Path("config/mapping.yaml")


def test_fixture_loader_returns_vectors():
    frames = check_stack.load_fixture_vectors(FIXTURE)
    assert frames, "fixture should produce at least one motion vector"
    first = frames[0]
    assert isinstance(first, check_stack.FixtureFrame)
    assert -1.0 <= first.lat <= 1.0
    assert 0.0 <= first.crowd <= 1.0


def test_bridge_emits_msp_frames():
    serial_link = check_stack.MSPSerialLoopback()
    harness = check_stack.BridgeHarness(MAPPING, serial_link, osc_port=0)
    harness.start()
    try:
        assert harness.wait_ready(), "OSC server never announced readiness"
        client = udp_client.SimpleUDPClient("127.0.0.1", harness.listening_port)
        client.send_message(harness.addresses["consent"], 1)
        time.sleep(0.01)
        for vec in check_stack.load_fixture_vectors(FIXTURE)[:25]:
            client.send_message(harness.addresses["lateral"], vec.lat)
            client.send_message(harness.addresses["altitude"], vec.alt)
            client.send_message(harness.addresses["yaw"], vec.yaw)
            client.send_message(harness.addresses["crowd"], vec.crowd)
            time.sleep(0.003)
        time.sleep(0.05)
    finally:
        harness.stop()
    check_stack.assert_msp_activity(serial_link)


def test_led_and_dsp_mocks_answer():
    check_stack.run_led_mock_check()
    check_stack.run_dsp_mock_check()


def test_cli_entrypoint_runs_fast():
    exit_code = check_stack.main(
        [
            "--osc-port",
            "0",
            "--max-frames",
            "30",
            "--send-interval",
            "0.004",
            "--warmup",
            "0.02",
            "--cooldown",
            "0.05",
        ]
    )
    assert exit_code == 0


def test_cli_rejects_missing_fixture(tmp_path, capsys):
    missing_fixture = tmp_path / "nope.json"
    exit_code = check_stack.main(["--osc-port", "0", "--fixture", str(missing_fixture)])
    captured = capsys.readouterr()
    assert exit_code == 2
    assert "Fixture file not found" in captured.err


def test_each_recipe_initializes_bridge():
    fixture_vectors = check_stack.load_fixture_vectors(FIXTURE)[:5]
    recipe_dir = REPO_ROOT / "config" / "recipes"
    for recipe_path in sorted(recipe_dir.glob("*.yaml")):
        serial_link = check_stack.MSPSerialLoopback()
        cfg, _meta = check_stack.bridge.load_recipe(recipe_path)
        harness = check_stack.BridgeHarness(
            recipe_path,
            serial_link,
            osc_port=0,
            cfg=cfg,
        )
        harness.start()
        try:
            assert harness.wait_ready(), f"Harness never ready for {recipe_path.name}"
            client = udp_client.SimpleUDPClient("127.0.0.1", harness.listening_port)
            client.send_message(harness.addresses["consent"], 1)
            time.sleep(0.01)
            for vec in fixture_vectors:
                client.send_message(harness.addresses["lateral"], vec.lat)
                client.send_message(harness.addresses["altitude"], vec.alt)
                client.send_message(harness.addresses["yaw"], vec.yaw)
                client.send_message(harness.addresses["crowd"], vec.crowd)
                time.sleep(0.002)
            time.sleep(0.02)
        finally:
            harness.stop()
        check_stack.assert_msp_activity(serial_link)


def test_bridge_neutralizes_after_consent_drop():
    serial_link = check_stack.MSPSerialLoopback()
    harness = check_stack.BridgeHarness(MAPPING, serial_link, osc_port=0)
    fixture_vectors = check_stack.load_fixture_vectors(FIXTURE)[:10]
    harness.start()
    try:
        assert harness.wait_ready(), "OSC server never announced readiness"
        client = udp_client.SimpleUDPClient("127.0.0.1", harness.listening_port)
        client.send_message(harness.addresses["consent"], 1)
        time.sleep(0.01)
        for idx, vec in enumerate(fixture_vectors):
            client.send_message(harness.addresses["lateral"], vec.lat)
            client.send_message(harness.addresses["altitude"], vec.alt)
            client.send_message(harness.addresses["yaw"], vec.yaw)
            client.send_message(harness.addresses["crowd"], vec.crowd)
            if idx == 4:
                client.send_message(harness.addresses["consent"], 0)
            time.sleep(0.003)
        client.send_message(harness.addresses["consent"], 0)
        time.sleep(0.1)
    finally:
        harness.stop()

    check_stack.assert_msp_activity(serial_link)
    last_rc, _aux = check_stack.decode_msp_frame(serial_link.packets()[-1])
    neutral_rc = (
        check_stack.bridge.map_float_to_rc(0.0),
        check_stack.bridge.map_float_to_rc(0.0),
        check_stack.bridge.map_float_to_rc(-1.0),
        check_stack.bridge.map_float_to_rc(0.0),
    )
    assert last_rc == neutral_rc


def test_check_stack_logs_audit_events(tmp_path, monkeypatch):
    log_dir = tmp_path / "logs"
    monkeypatch.setenv("PERCEPTUAL_DRIFT_LOG_DIR", str(log_dir))
    exit_code = check_stack.main(
        [
            "--osc-port",
            "0",
            "--max-frames",
            "20",
            "--send-interval",
            "0.002",
            "--warmup",
            "0.01",
            "--cooldown",
            "0.02",
            "--log-events",
        ]
    )
    assert exit_code == 0
    log_path = log_dir / "ops_events.jsonl"
    assert log_path.exists(), "ops_events.jsonl missing despite logging flag"
    actions = [
        json.loads(line)["action"]
        for line in log_path.read_text().splitlines()
        if line.strip()
    ]

    def assert_subsequence(sequence, subsequence):
        iterator = iter(sequence)
        for target in subsequence:
            for item in iterator:
                if item == target:
                    break
            else:  # pragma: no cover - defensive guard
                raise AssertionError(f"{target} missing from actions")

    assert_subsequence(actions, ["consent_toggle", "osc_bridge_stream", "osc_bridge_shutdown"])
