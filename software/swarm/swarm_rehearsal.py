#!/usr/bin/env python3
"""Spin up the full Perceptual Drift swarm sim for a stage-friendly demo."""

from __future__ import annotations

import argparse
import asyncio
from pathlib import Path
from typing import Dict

from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Launch the Ignition + ROS2 rehearsal stack without hardware.",
    )
    parser.add_argument(
        "--world",
        type=Path,
        default=None,
        help="Optional path to an alternate SDF world. Defaults to the built-in pd_swarm.world.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run Ignition Gazebo without the GUI window.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the resolved launch arguments instead of executing the launch service.",
    )
    return parser.parse_args()


async def launch_sim(world: Path | None, headless: bool) -> int:
    repo_root = Path(__file__).resolve().parent
    launch_file = repo_root / "simulation" / "launch" / "pd_swarm_sim.launch.py"
    if world is None:
        world = repo_root / "simulation" / "worlds" / "pd_swarm.world"

    launch_arguments: Dict[str, str] = {
        "world": str(world),
        "headless": "true" if headless else "false",
    }

    launch_service = LaunchService()
    launch_description = PythonLaunchDescriptionSource(str(launch_file))
    launch_service.include_launch_description(
        launch_description,
        launch_arguments=launch_arguments.items(),
    )

    return await launch_service.run_async()


def main() -> None:
    args = parse_args()
    if args.dry_run:
        repo_root = Path(__file__).resolve().parent
        default_world = repo_root / "simulation" / "worlds" / "pd_swarm.world"
        world = args.world or default_world
        print("Launch would load world", world)
        print("Headless:", args.headless)
        return

    exit_code = asyncio.run(launch_sim(args.world, args.headless))
    if exit_code != 0:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
