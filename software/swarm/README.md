# Swarm Control (ROS2 + CrazySwarm2)

This folder scaffolds integration with **CrazySwarm2** for multi-drone coordination.

This is the place where Perceptual Drift's squishy human gestures become tight,
punky drone choreography. The revamped `swarm_demo.py` fans OSC across every Crazyfly
listed in `DEFAULT_FLEET`, mapping both the global `/pd/*` gestures and per-craft
`/pd/<craft>/lat` + `/pd/<craft>/alt` overrides. It also tails telemetry topics and prints
heartbeat snapshots so you know round-trip data is flowing before you invite an audience.
We still ship a full Ignition Gazebo rehearsal rig that mirrors those inputs, so you can
stress the score without ever arming a real quad.

## Prerequisites

| What | Why it matters |
| ---- | -------------- |
| ROS 2 Foxy or Humble workspace | `swarm_demo.py` is a ROS 2 node; source your workspace before launching. |
| [`crazyflie_ros2`](https://github.com/IMRCLab/crazyswarm2) built in that workspace | Provides the `crazyflie_interfaces/srv/Takeoff` and `.../GoTo` services. |
| Motion capture or Lighthouse tracking dialed in | CrazySwarm2 assumes you know where every drone lives. |
| [`python-osc`](https://pypi.org/project/python-osc/) | Feeds the crowd-control data into ROS. |
| Fully charged Crazyflies & a big enough safety net | We're commanding flight. Don't be reckless. |

## Run it live (hardware)

1. Fire up your CrazySwarm2 stack (mocap, radio, etc.) and verify a single
   drone responds to `ros2 service call /cf1/takeoff` in a safe, empty space.
2. Copy `software/swarm/swarm_demo.py` into your ROS workspace or invoke it via
   `ros2 run perceptual_drift_swarm swarm_demo` if you have it installed as a
   package.
3. Edit the tweakables at the top of the file: swap `DEFAULT_FLEET` entries for
   your Crazyflies, adjust the axis choice, and tune scaling factors. Treat them
   like guard rails, not suggestions.
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
6. Flap your limbs. `/pd/alt` + `/pd/lat` move the full formation; override a
   single craft with `/pd/<craft>/alt` and `/pd/<craft>/lat`. Watch the telemetry
   snapshots in the ROS console—if they stall, freeze the show and troubleshoot.

## Rehearse it in sim

1. Source your ROS 2 workspace (the same one that provides `ros_ign_gazebo`,
   `ros_gz_interfaces`, and `crazyflie_interfaces`).
2. Fire up the full stack:

   ```bash
   ./software/swarm/swarm_rehearsal.py
   ```

   Add `--headless` if you want to ditch the GUI or `--dry-run` to peek at the
   resolved launch arguments.
3. Point your OSC stream at the same `/pd/*` topics you would use on hardware.
   The bridge node, `software/swarm/ros2_nodes/pd_swarm_bridge.py`, translates
   altitude, lateral pushes, and yaw into CrazySwarm2 service calls. It also
   publishes fake-but-plausible `PoseStamped` telemetry on `/pd/sim/*`.
4. Ignition’s X500 proxies track those poses via
   `software/swarm/ros2_nodes/pd_sim_pose_driver.py`, so you get a 1:1 visual of
   what your choreography will demand from real drones.
5. Update formations, rate limits, and bounding boxes by tweaking the declared
   parameters inside `pd_swarm_bridge.py`. Every knob is annotated; treat them
   like safety rails, not style suggestions.

## ROS 2 nodes in this folder

| File | Role |
| ---- | ---- |
| `swarm_demo.py` | Multi-drone OSC→CrazySwarm2 bridge with per-craft overrides and telemetry snapshots. |
| `ros2_nodes/pd_swarm_bridge.py` | Multi-drone ROS 2 node that consumes `/pd/*` topics, fans them out to CrazySwarm2 services, and publishes simulated telemetry. |
| `ros2_nodes/pd_sim_pose_driver.py` | Mirrors those telemetry messages into Ignition by calling `/world/<name>/set_pose`. |
| `swarm_rehearsal.py` | Launch helper that spins up Ignition + both ROS nodes with one command. |

The rehearsal stack expects a CrazySwarm2-style workspace (for message/service
types) plus the `ros_ign_gazebo` launch files. If you install these scripts as a
ROS 2 package, update the launch file under `simulation/launch/` to reference
your package name instead of the placeholder `perceptual_drift_swarm`.

## Safety riffs

* Never test in a crowd. Even punk shows have barricades.
* Stay under local legal altitude limits and respect RF rules.
* Keep the room soft: nets, curtains, no fragile art pieces.
* Have a spotter ready to yank power the second a drone misbehaves.
* Log everything. When things go weird (they will), you'll want receipts.
* Rehearse the kill chain in simulation before you step into the venue. Muscle
  memory is a safety feature.
