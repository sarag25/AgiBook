"""
Launch file per avviare il nodo pick_book_node sul robot X2 in Gazebo.
Da eseguire DOPO che gazebo.launch.py è già partito e i controller sono attivi.

  ros2 launch agibot_x2_pkg gazebo.launch.py    # terminale 1
  ros2 launch agibot_x2_pkg pick_books.launch.py  # terminale 2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('agibot_x2_pkg')
    script = os.path.join(pkg, 'scripts', 'pick_book_node.py')

    pick_node = Node(
        package='agibot_x2_pkg',
        executable='pick_book_node',
        name='pick_book_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([pick_node])
