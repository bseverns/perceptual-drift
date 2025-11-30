"""Lightweight virtual drone kinematics for simulation/teaching mode.

This intentionally toy model mirrors the swarm's gesture-to-behavior mapping
without touching real hardware. It is meant to be legible and tweakable: keep it
roughly in the spirit of a studio notebook rather than a physics engine.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Dict

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
