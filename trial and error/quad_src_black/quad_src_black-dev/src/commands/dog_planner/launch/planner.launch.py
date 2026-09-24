import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('dog_planner')
    config_file = os.path.join(pkg_dir, 'config', 'planner.yaml')

    global_planner_node = Node(
        package='dog_planner',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[config_file]
    )

    local_planner_node = Node(
        package='dog_planner',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        global_planner_node,
        local_planner_node
    ])
