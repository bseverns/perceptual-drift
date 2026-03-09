import math

import pytest

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
