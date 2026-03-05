"""Lightweight virtual drone kinematics for simulation/teaching mode.

This intentionally toy model mirrors the swarm's gesture-to-behavior mapping
without touching real hardware. It is meant to be legible and tweakable: keep it
roughly in the spirit of a studio notebook rather than a physics engine.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Dict, Iterable, List, Tuple

ROOM_BOUND = 5.0


@dataclass
class VirtualDrone:
    """Extremely simple kinematic model for a simulated drone.

    The goal is to visualize gesture-driven behavior safely: positions, altitude,
    and yaw shift in response to mapped "behavior" values while staying inside a
    bounded virtual room. No attempt is made to replicate Crazyflie dynamics; we
    only want something that *moves* so downstream OSC consumers have meaningful
    data to chew on.
    """

    drone_id: int
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    vyaw: float = 0.0

    def update(self, dt: float, behavior: Dict[str, float]) -> None:
        """Update position/orientation over ``dt`` seconds using mapped behavior.

        Parameters
        ----------
        dt: float
            Simulation timestep in seconds.
        behavior: Dict[str, float]
            Dictionary of high-level behavior values (altitude target, lateral
            bias, yaw bias, jitter) derived from the gesture mapping layer.
        """

        altitude_target = behavior.get("altitude_target", self.z)
        lateral_bias = behavior.get("lateral_bias", 0.0)
        yaw_bias = behavior.get("yaw_bias", 0.0)
        jitter = behavior.get("jitter", 0.0)

        # Smoothly nudge altitude toward the target.
        self.vz += (altitude_target - self.z) * 0.5 * dt
        self.z += self.vz * dt

        # Shuffle laterally with a tiny easing toward the bias.
        self.vx += lateral_bias * 0.2 * dt
        self.vy += lateral_bias * 0.1 * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

        # Spin lazily in response to yaw bias.
        self.vyaw += yaw_bias * 0.3 * dt
        self.yaw += self.vyaw * dt

        # Add gentle chaos if asked: tiny random walk, bounded.
        if jitter > 0.0:
            scale = min(jitter, 1.0) * 0.05
            self.x += random.uniform(-scale, scale)
            self.y += random.uniform(-scale, scale)
            self.z += random.uniform(-scale, scale)
            self.yaw += random.uniform(-scale, scale)

        self._clamp_position()

    def _clamp_position(self) -> None:
        self.x = max(-ROOM_BOUND, min(ROOM_BOUND, self.x))
        self.y = max(-ROOM_BOUND, min(ROOM_BOUND, self.y))
        self.z = max(0.0, min(ROOM_BOUND, self.z))
        # Keep yaw bounded-ish for readability while allowing wrap-around feel.
        self.yaw = math.fmod(self.yaw, math.tau)


def pairwise_distances(drones: Iterable[VirtualDrone]) -> List[Tuple[int, int, float]]:
    """Return all pairwise Euclidean distances between drones."""

    fleet = list(drones)
    pairs: List[Tuple[int, int, float]] = []
    for i in range(len(fleet)):
        for j in range(i + 1, len(fleet)):
            dx = fleet[j].x - fleet[i].x
            dy = fleet[j].y - fleet[i].y
            dz = fleet[j].z - fleet[i].z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            pairs.append((i, j, dist))
    return pairs


def minimum_pairwise_distance(drones: Iterable[VirtualDrone]) -> float:
    """Return minimum pairwise distance, inf when <2 drones."""

    pairs = pairwise_distances(drones)
    if not pairs:
        return math.inf
    return min(dist for _, _, dist in pairs)


def enforce_min_separation(
    drones: Iterable[VirtualDrone],
    min_separation_m: float,
    max_passes: int = 3,
) -> int:
    """Nudge drones apart in XY if they violate ``min_separation_m``.

    Returns the number of pair adjustments performed.
    """

    if min_separation_m <= 0.0:
        return 0

    fleet = list(drones)
    adjustments = 0
    for _ in range(max_passes):
        changed = False
        for i in range(len(fleet)):
            for j in range(i + 1, len(fleet)):
                a = fleet[i]
                b = fleet[j]
                dx = b.x - a.x
                dy = b.y - a.y
                # Keep this horizontal: avoid surprise altitude snaps.
                dist = math.sqrt(dx * dx + dy * dy)
                if dist >= min_separation_m:
                    continue

                # Degenerate overlap: choose a deterministic axis.
                if dist < 1e-9:
                    ux, uy = 1.0, 0.0
                else:
                    ux = dx / dist
                    uy = dy / dist

                push = (min_separation_m - dist) * 0.5
                a.x -= ux * push
                a.y -= uy * push
                b.x += ux * push
                b.y += uy * push
                a._clamp_position()
                b._clamp_position()
                adjustments += 1
                changed = True
        if not changed:
            break
    return adjustments
