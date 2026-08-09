"""
fire_demo.launch.py
────────────────────
Launches Gazebo Harmonic with the fire plugin demo world.

Usage:
  ros2 launch gz_fire_plugin_demo fire_demo.launch.py
  ros2 launch gz_fire_plugin_demo fire_demo.launch.py world:=demo_cube_fire
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_demo  = FindPackageShare("gz_fire_plugin_demo")
    pkg_plugin = FindPackageShare("gz_fire_plugin")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="demo_cube_fire",
        description="World stem name (without .sdf)"
    )

    world_file = PathJoinSubstitution([
        pkg_demo, "worlds",
        [LaunchConfiguration("world"), ".sdf"]
    ])

    # Make sure Gazebo can find the plugin .so
    set_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=PathJoinSubstitution([pkg_plugin, "lib", "gz_fire_plugin"])
    )

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
    )

    return LaunchDescription([
        world_arg,
        set_plugin_path,
        gz_sim,
    ])
