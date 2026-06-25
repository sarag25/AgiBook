"""
Avvia il nodo LibraryManager.
Da usare dopo gazebo.launch.py.

  ros2 launch x2_description library_manager.launch.py [use_llm:=true] [depth_mode:=rgbd]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("yolo_model",  default_value="yolov8n.pt"),
        DeclareLaunchArgument("depth_mode",  default_value="midas"),
        DeclareLaunchArgument("use_llm",     default_value="false"),

        Node(
            package="x2_description",
            executable="library_manager_node",
            name="library_manager",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "yolo_model":  LaunchConfiguration("yolo_model"),
                "depth_mode":  LaunchConfiguration("depth_mode"),
                "use_llm":     LaunchConfiguration("use_llm"),
                "ocr_languages": ["it", "en"],
            }],
        ),
    ])
