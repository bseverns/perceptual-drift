# Swarm Quickstart — OSC to CrazySwarm2 bridge notes

This is the rung after a single Betaflight whoop: how to point our OSC stream at the `software/swarm` bridge, what ROS 2/CrazySwarm2 bits must be alive, and copy/paste command strings for both simulator rehearsals and hardware dry-runs. Keep the studio-notebook vibe, but treat every checkbox like a safety latch.

---

## OSC → ROS 2 mapping (what the bridge actually listens for)

| OSC address | ROS 2 subscriber/behavior | Notes |
| --- | --- | --- |
| `/pd/alt` | [`pd_swarm_bridge` subscribes as `Float32` and translates into per-drone takeoff+height targets](../software/swarm/ros2_nodes/pd_swarm_bridge.py#L80-L135) | Normalized `0.0–1.0` becomes `altitude_floor_m + scale` meters; consent gate blocks calls. |
| `/pd/lat` | [`pd_swarm_bridge` subscribes as `Float32` and offsets the swarm laterally](../software/swarm/ros2_nodes/pd_swarm_bridge.py#L80-L136) | Uses `lateral_axis` (`x` or `y`) and spreads the formation so center drones stay centered. |
| `/pd/yaw` | [`pd_swarm_bridge` subscribes as `Float32` and rotates the formation](../software/swarm/ros2_nodes/pd_swarm_bridge.py#L84-L139) | Expects `-1..1`; scaled to degrees via `yaw_scale_deg`. |
| `/pd/consent` | [`pd_swarm_bridge` subscribes as `Bool` and locks/unlocks all commands](../software/swarm/ros2_nodes/pd_swarm_bridge.py#L86-L118) | When false: lands and stops every craft, ignores new targets until consent returns. |
| `/pd/<craft>/lat`, `/pd/<craft>/alt` | [`swarm_demo.py` maps per-craft overrides into CrazySwarm2 GoTo/Takeoff calls](../software/swarm/swarm_demo.py#L17-L111) | Optional per-drone routes; still honor consent and formation offsets. |

Everything above rides on your existing OSC mappings (`config/mapping.yaml` or recipes), so you can aim Processing/TouchDesigner at the same address space you use for the single-drone bridge.

---

## ROS 2 + CrazySwarm2 prerequisites (don’t skip)

- **ROS 2 workspace sourced** (Foxy/Humble). The bridge nodes are ROS 2 nodes; run `source /opt/ros/<distro>/setup.bash` and your CrazySwarm2 `install/setup.bash` before any commands.
- **CrazySwarm2 built and talking to your trackers.** `ros2 service call /cf1/takeoff crazyflie_interfaces/srv/Takeoff` should work with props off before you layer OSC in.
- **`crazyflie_interfaces` available.** `pd_swarm_bridge` and `swarm_demo.py` call `GoTo`, `Takeoff`, `Land`, and `Stop` services; make sure they exist in your workspace.
- **python-osc installed** in the environment running `swarm_demo.py` (for receiving OSC) or whatever OSC sender you’re using.
- **Safety rails set**: netted space, props off for rehearsals, consent choreography drilled. Novelty < utility; muscle memory beats heroics.

---

## Simulated rehearsal (Ignition + ROS 2 stack)

1. **Source your workspace** that contains CrazySwarm2 + `ros_ign_gazebo`:
   ```bash
   source /opt/ros/<distro>/setup.bash
   source ~/colcon_ws/install/setup.bash
   ```
2. **Run the launch helper** (drives Ignition + both ROS 2 nodes):
   ```bash
   ./software/swarm/swarm_rehearsal.py --headless
   ```
   Add `--dry-run` to print the composed launch and bail if you’re just verifying paths.
3. **Point your OSC sender** (Processing tracker, TouchDesigner, etc.) at `udp://<bridge-host>:9010` and drive `/pd/alt`, `/pd/lat`, `/pd/yaw`, and `/pd/consent`.
4. **Watch simulated telemetry** on `/pd/sim/<craft>/pose` or in Ignition. When consent drops, the bridge auto-lands and freezes inputs; confirm that behavior before moving to hardware.

---

## Hardware dry-run (props off, radios on)

1. **Power the Crazyflies + Crazyswarm2 backend**, props removed or ducts taped for safety. Confirm `ros2 topic list` shows `/cf*/...` topics.
2. **Start the OSC→CrazySwarm2 bridge** from this repo:
   ```bash
   python3 software/swarm/swarm_demo.py --base-mapping config/mapping.yaml
   ```
   - Leave the defaults (`0.0.0.0:9010`) unless your OSC sender expects a different port.
   - If you run it inside your ROS 2 workspace as an installed package, swap the command for `ros2 run perceptual_drift_swarm swarm_demo`.
3. **Send OSC gestures** from your tracker or a quick CLI poke:
   ```bash
   python3 -m pythonosc.udp_client 127.0.0.1 9010 /pd/consent 1
   python3 -m pythonosc.udp_client 127.0.0.1 9010 /pd/alt 0.4
   python3 -m pythonosc.udp_client 127.0.0.1 9010 /pd/lat -0.2
   ```
   Swap in `/pd/cf1/lat` or `/pd/cf2/alt` if you want per-drone overrides. You should see takeoff/GoTo service calls in the ROS log but the props stay still because you removed them.
4. **Kill and relaunch**: drop consent back to `0`, confirm every craft lands/locks, then restart the bridge and re-run the three messages above. No surprises? You’re cleared to reinstall props and move into the netted cage.

This quickstart is the bridge between “one drone hello world” and “swarm dress rehearsal.” Treat it like a lab notebook you can scribble in; add your own venue-specific notes next time you rehearse.
