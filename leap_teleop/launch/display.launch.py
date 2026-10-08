"""
Step 1: show the LEAP Hand URDF (dex-urdf) in RViz2.

    ros2 launch leap_teleop display.launch.py                 # with slider GUI
    ros2 launch leap_teleop display.launch.py gui:=false      # joint_states come from another node
    ros2 launch leap_teleop display.launch.py hand:=left
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "leap_teleop"


def _nodes(context):
    share = get_package_share_directory(PKG)
    hand = LaunchConfiguration("hand").perform(context)
    urdf_file = os.path.join(share, "assets", "leap_hand", f"leap_hand_{hand}.urdf")
    rviz_file = os.path.join(share, "rviz", "leap_hand.rviz")

    with open(urdf_file, "r") as f:
        urdf = f.read()
    # dex-urdf uses mesh paths relative to the URDF file; RViz needs a resolvable URI.
    urdf = urdf.replace('filename="meshes/', f'filename="package://{PKG}/assets/leap_hand/meshes/')

    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[{"robot_description": urdf}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             output="screen", condition=IfCondition(LaunchConfiguration("gui"))),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", rviz_file]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hand", default_value="right", choices=["right", "left"]),
        DeclareLaunchArgument("gui", default_value="true",
                              description="start joint_state_publisher_gui sliders"),
        OpaqueFunction(function=_nodes),
    ])
