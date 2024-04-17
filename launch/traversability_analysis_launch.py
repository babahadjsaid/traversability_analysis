import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('traversability_analysis')
    

    config = os.path.join(
        share_dir,
        'config',
        'param.yaml'
        )

    return LaunchDescription([
        Node(
            package='traversability_analysis',
            executable='traversability_analysis',
            name='traversability_analysis',
            parameters=[config],
            output='screen'
        )
    ])
