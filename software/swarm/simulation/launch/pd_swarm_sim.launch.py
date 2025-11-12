"""Launch file to run the Perceptual Drift swarm rehearsal in Ignition."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("ros_ign_gazebo"))
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share / "launch" / "ign_gazebo.launch.py"
        ),
        launch_arguments={
            "gz_args": [world, " -r"],
            "headless": headless,
        }.items(),
    )

    swarm_bridge = Node(
        package="perceptual_drift_swarm",
        executable="pd_swarm_bridge",
        output="screen",
        emulate_tty=True,
    )

    pose_driver = Node(
        package="perceptual_drift_swarm",
        executable="pd_sim_pose_driver",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=str(
                    Path(__file__).resolve().parent.parent
                    / "worlds"
                    / "pd_swarm.world"
                ),
                description="Path to the Ignition SDF world file.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Ignition Gazebo without a GUI.",
            ),
            ignition_launch,
            swarm_bridge,
            pose_driver,
        ]
    )
