"""
Launch file that starts the LibraryManager node.
Run it after gazebo.launch.py.

  ros2 launch agibot_x2_pkg library_manager.launch.py [detector:=sam3|depth|yolo|none] [use_llm:=true] [depth_mode:=rgbd]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Declare the detector arguments and start library_manager_node
    """
    return LaunchDescription([
        DeclareLaunchArgument("detector",    default_value="sam3"),
        DeclareLaunchArgument("yolo_model",  default_value="yolov8n.pt"),
        DeclareLaunchArgument("depth_mode",  default_value="midas"),
        DeclareLaunchArgument("use_llm",     default_value="false"),

        Node(
            package="agibot_x2_pkg",
            executable="library_manager_node",
            name="library_manager",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "detector":    LaunchConfiguration("detector"),
                "yolo_model":  LaunchConfiguration("yolo_model"),
                "depth_mode":  LaunchConfiguration("depth_mode"),
                "use_llm":     LaunchConfiguration("use_llm"),
                "ocr_languages": ["it", "en"],
            }],
        ),
    ])
