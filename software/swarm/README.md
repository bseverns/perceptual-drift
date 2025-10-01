# Swarm Control (ROS2 + CrazySwarm2)

This folder scaffolds integration with **CrazySwarm2** for multi-drone coordination.

This is the place where Perceptual Drift's squishy human gestures become tight,
punky drone choreography. The `swarm_demo.py` script listens to OSC messages and
pushes them straight into CrazySwarm2 services.

## Prerequisites

| What | Why it matters |
| ---- | -------------- |
| ROS 2 Foxy or Humble workspace | `swarm_demo.py` is a ROS 2 node; source your workspace before launching. |
| [`crazyflie_ros2`](https://github.com/IMRCLab/crazyswarm2) built in that workspace | Provides the `crazyflie_interfaces/srv/Takeoff` and `.../GoTo` services. |
| Motion capture or Lighthouse tracking dialed in | CrazySwarm2 assumes you know where every drone lives. |
| [`python-osc`](https://pypi.org/project/python-osc/) | Feeds the crowd-control data into ROS. |
| Fully charged Crazyflies & a big enough safety net | We're commanding flight. Don't be reckless. |

## Run it live

1. Fire up your CrazySwarm2 stack (mocap, radio, etc.) and verify a single
   drone responds to `ros2 service call /cf1/takeoff` in a safe, empty space.
2. Copy `software/swarm/swarm_demo.py` into your ROS workspace or invoke it via
   `ros2 run perceptual_drift_swarm swarm_demo` if you have it installed as a
   package.
3. Edit the tweakables at the top of the file: service names, axis choice,
   scaling factors. Treat them like guard rails, not suggestions.
4. In one terminal (with your workspace sourced):

   ```bash
   ros2 run perceptual_drift_swarm swarm_demo
   ```

   or, if you're just executing the script directly:

   ```bash
   python3 software/swarm/swarm_demo.py
   ```

5. Point Perceptual Drift's OSC output at the IP/port you configured (defaults
   to `0.0.0.0:9010`).
6. Flap your limbs. `/pd/alt` drives `Takeoff` height, `/pd/lat` slides the
   craft laterally. Keep a thumb on the emergency stop.

## Safety riffs

* Never test in a crowd. Even punk shows have barricades.
* Stay under local legal altitude limits and respect RF rules.
* Keep the room soft: nets, curtains, no fragile art pieces.
* Have a spotter ready to yank power the second a drone misbehaves.
* Log everything. When things go weird (they will), you'll want receipts.
