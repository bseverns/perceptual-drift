import math
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from software.swarm import swarm_demo
from software.swarm.replay_scenario import (
    Keyframe,
    Participant,
    _aggregate,
    _sample_participant,
)
from software.swarm.virtual_swarm import (
    VirtualDrone,
    enforce_min_separation,
    minimum_pairwise_distance,
)


def test_enforce_min_separation_pushes_overlapping_drones_apart():
    a = VirtualDrone(drone_id=0, x=0.0, y=0.0, z=0.5)
    b = VirtualDrone(drone_id=1, x=0.0, y=0.0, z=0.5)
    c = VirtualDrone(drone_id=2, x=1.0, y=0.0, z=0.5)
    drones = [a, b, c]

    adjusted = enforce_min_separation(drones, min_separation_m=0.35)
    min_dist = minimum_pairwise_distance(drones)

    assert adjusted > 0
    assert min_dist >= 0.35 - 1e-6


def test_minimum_pairwise_distance_with_single_drone_is_infinite():
    d = VirtualDrone(drone_id=0, x=0.0, y=0.0, z=0.0)
    assert math.isinf(minimum_pairwise_distance([d]))


def test_replay_sampling_interpolates_between_keyframes():
    p = Participant(
        pid="p1",
        weight=1.0,
        keyframes=[
            Keyframe(
                t=0.0, alt=0.2, lat=-0.6, yaw=-0.2, crowd=0.1, consent=1.0
            ),
            Keyframe(t=2.0, alt=0.6, lat=0.2, yaw=0.4, crowd=0.7, consent=0.0),
        ],
    )
    sample = _sample_participant(p, 1.0)
    assert sample.alt == pytest.approx(0.4)
    assert sample.lat == pytest.approx(-0.2)
    assert sample.yaw == pytest.approx(0.1)
    assert sample.crowd == pytest.approx(0.4)
    assert sample.consent == pytest.approx(0.5)


def test_replay_aggregate_weighted_mode():
    samples = [
        Keyframe(t=0.0, alt=0.2, lat=-0.5, yaw=-0.2, crowd=0.2, consent=1.0),
        Keyframe(t=0.0, alt=0.8, lat=0.5, yaw=0.2, crowd=0.8, consent=0.0),
    ]
    merged = _aggregate(samples, [1.0, 3.0], mode="weighted")

    assert abs(merged.alt - 0.65) < 1e-9
    assert abs(merged.lat - 0.25) < 1e-9
    assert abs(merged.yaw - 0.1) < 1e-9
    assert abs(merged.crowd - 0.65) < 1e-9
    assert abs(merged.consent - 0.25) < 1e-9


def test_ros_swarm_bridge_starts_consent_locked():
    source = Path("software/swarm/ros2_nodes/pd_swarm_bridge.py").read_text()

    assert "self._consent_enabled = False" in source
    for callback in ("on_alt", "on_lat", "on_yaw"):
        start = source.index(f"    def {callback}")
        body = source[start : source.index("\n    def ", start + 1)]
        assert "if not self._consent_enabled:\n            return" in body


def test_swarm_demo_physical_sends_refuse_without_fresh_consent():
    node = swarm_demo.SwarmNode.__new__(swarm_demo.SwarmNode)
    node._has_fresh_consent = lambda: False
    node._wait_for_service = lambda *_args: pytest.fail(
        "service lookup must not occur without fresh consent"
    )
    state = SimpleNamespace(
        config=SimpleNamespace(name="cf1"),
        go_to_client=object(),
        takeoff_client=object(),
        current_altitude=0.8,
    )
    node.crafts = {"cf1": state}

    node._send_go_to(state)
    node._send_takeoff(state)
    node._apply_altitude("cf1", 0.7, source="test")
    assert state.current_altitude == 0.8


@pytest.mark.parametrize("last_consent_at", [None, 10.0])
def test_swarm_demo_watchdog_forces_off_through_land_stop_edge(
    last_consent_at,
):
    events = []

    class Logger:
        def error(self, *_args):
            events.append("stale-log")

        def warning(self, *_args):
            events.append("land-log")

    node = swarm_demo.SwarmNode.__new__(swarm_demo.SwarmNode)
    node._consent_lock = threading.Lock()
    node._consent_state = 1
    node._last_consent_at = last_consent_at
    node._consent_timeout_s = 1.0
    node.gesture_state = {"consent": 1.0}
    node._mapping_lock = threading.Lock()
    node.mapping = {"consent": {"auto_recipes": False}}
    node.crafts = {"cf1": SimpleNamespace(current_altitude=0.8)}
    node._last_broadcast_consent = 1
    node.get_logger = lambda: Logger()
    node._send_land = lambda _state: events.append("land")
    node._send_stop = lambda _state: events.append("stop")
    node._broadcast_consent_state = lambda state: events.append(
        f"broadcast-{state}"
    )

    assert node._expire_stale_consent(now=11.01) is True
    assert node._consent_state == 0
    assert node.gesture_state["consent"] == 0.0
    assert events == [
        "stale-log",
        "land-log",
        "land",
        "stop",
        "broadcast-0",
    ]


def test_swarm_demo_defaults_to_loopback_and_warns_on_exposure(capsys):
    parsed, _remaining = swarm_demo._parse_cli([])
    assert parsed.bind == "127.0.0.1"
    assert swarm_demo.OSC_BIND_ADDRESS == "127.0.0.1"

    warning = swarm_demo._warn_if_exposed_bind("0.0.0.0", 9010)
    assert warning is not None
    assert "Any reachable host" in capsys.readouterr().err
