# Swarm Safety + Simulation Cheatsheet

Welcome to the chaos lab. This doc explains how we keep the simulated swarm tame
and how those guardrails should translate when we swap Ignition for actual
rotors. Read it like a zine, treat it like a checklist.

## Safety interlocks

| Interlock | Simulation behavior | Real-world translation |
| --------- | ------------------ | ---------------------- |
| **Consent latch** (`/pd/consent`) | `pd_swarm_bridge` refuses to issue new service calls. `pd_sim_pose_driver` keeps the models frozen at their last pose. | Map the same boolean to the CrazySwarm2 `/emergency` or `/all/emergency` service. Hardware version should also trip a physical estop (power strip or deck kill) when the bit flips low. |
| **Altitude floor + ceiling** | `ALTITUDE_FLOOR_M` / `ALTITUDE_SCALE_M` parameters limit commands to a 0.3–1.3 m bubble. The fake telemetry echoes the clamp so overlays never lie. | Match the envelope to your net height. If your venue ceiling is lower, change the params before the show. Never let the crowd stretch it past what your mocap volume supports. |
| **Lateral bounding box** | `lateral_range` + `lateral_scale_m` restrict crowd pushes to ±1.6 m. Formation offsets keep each quad from trying to occupy the same air. | Mirror the same limits inside your CrazySwarm2 YAML (per-drone `motion_constraints`). Tape the actual floor footprint and rehearse inside it. |
| **Kill zone** | If Ignition reports a pose outside the configured world bounds, `pd_sim_pose_driver` will fail to move the model (service rejects it) and you see the freeze instantly. | Use motion capture monitoring to slam an estop when tracking is lost or a drone exits the “safe cage.” CrazySwarm2 has watchdog timers—set them aggressively. |
| **Manual overrides** | You can `Ctrl+C` the `swarm_rehearsal.py` script or smash the Ignition GUI stop button. Both immediately tear down the ROS graph. | Keep a human pilot with an NRF dongle + Commander ready. Someone else owns the wall power. No single point of failure. |

## How it maps to hardware

1. **World file ≈ stage layout.** The Ignition world puts three X500 stand-ins over a
   flat floor. Swap in venue meshes or place `collision` boxes to mimic risers.
   Anything you build here should be mirrored into CrazySwarm2’s `crazyswarm.yaml`
   so the real drones expect the same geometry.
2. **Service names stay the same.** The bridge hits `/{drone}/takeoff`,
   `/{drone}/go_to`, and `/{drone}/land`. As long as your ROS 2 workspace exposes
   those endpoints, you can swap the simulated swarm for hardware without touching
   the messaging layer.
3. **Telemetry loop = your HUD.** The fake `PoseStamped` publishers mimic mocap
   updates. In hardware, subscribe to the CrazySwarm2 localization topics instead
   so your visualizers / lighting rig ingest the truth, not the predictions.
4. **Consent = estop.** We treat `/pd/consent` as the master arm/disarm flag.
   In sim it just prevents new service calls. With real drones hook it to
   `/{drone}/emergency` and, crucially, a physical kill circuit that drops power.
5. **Logs or it didn’t happen.** Capture rosbag files during rehearsal. When you
   move to hardware, those same bags become your “black box” for R&D and insurance.

## Preflight ritual (sim and real)

1. **Check your params.** Open `software/swarm/ros2_nodes/pd_swarm_bridge.py` and
   confirm the formation offsets, altitude scales, and durations make sense for
   today’s venue. Commit the config before the show.
2. **Run the rehearsal script headless.**

   ```bash
   ./software/swarm/swarm_rehearsal.py --headless
   ```

   The script boots Ignition, the ROS bridge, and the pose driver. Watch the
   console for consent warnings or service failures.
3. **Drive it with OSC.** Feed `/pd/*` data from the tracker. Verify the drones
   stay in their lanes and the GUI shows the same motion as the raw telemetry
   topics.
4. **Trigger every interlock.** Drop consent, slam the OSC inputs, and confirm
   the simulated swarm stops cold. Repeat in hardware until it’s muscle memory.
5. **Document weirdness.** Treat this README like a lab notebook—add quirks,
   postmortems, and per-venue tweaks so the next gig launches smoother.

Stay loud, stay safe.
