"""Launch shm_to_image with a stream config: params_file:=<yaml>, streams:=a,b runs a subset of it."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_node(context):
    params = [LaunchConfiguration("params_file").perform(context)]
    streams = [
        s for s in LaunchConfiguration("streams").perform(context).split(",") if s
    ]
    if streams:
        params.append({"streams": streams})
    return [
        Node(
            package="shm_ros",
            executable="shm_to_image",
            name="shm_to_image",
            output="screen",
            parameters=params,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", description="stream config yaml"),
            DeclareLaunchArgument(
                "streams", default_value="", description="comma list, empty = all"
            ),
            OpaqueFunction(function=launch_node),
        ]
    )
