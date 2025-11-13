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
