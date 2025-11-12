# Ignition rehearsal assets

This folder carries the bare essentials for a Perceptual Drift swarm rehearsal.
Drop it into a ROS 2 workspace so `ros2 launch` can find everything, or run the
helper script at `../swarm_rehearsal.py` which wires it up automatically.

## Contents

| Path | Purpose |
| ---- | ------- |
| `worlds/pd_swarm.world` | Three X500 stand-ins over a flat floor with a theatrical spotlight. Use it as the sandbox baseline. |
| `launch/pd_swarm_sim.launch.py` | Boots Ignition Gazebo, the PD swarm bridge, and the pose driver in one go. |

## Customization tips

* Swap the `model://x500` includes for your own URDF/SDF assets to match the
  craft you fly in the real show.
* Use Ignition’s GUI to drop collision walls or stage props. Save the world file
  and commit it so the rehearsal space evolves with your tour.
* If you package these nodes inside a ROS 2 overlay, change the `package=` values
  in the launch file to match your install target.
