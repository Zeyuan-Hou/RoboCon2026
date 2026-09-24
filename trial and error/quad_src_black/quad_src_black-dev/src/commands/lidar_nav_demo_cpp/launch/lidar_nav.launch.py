import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_nav_demo_cpp')
    config_file = os.path.join(
        pkg_share,
        'config',
        'waypoints.yaml'
    )
    prepath_file = os.path.join(
        pkg_share,
        'config',
        'prepath.yaml'
    )
    lidar_nav_node = Node(
        package='lidar_nav_demo_cpp',
        executable='lidar_nav_demo_node',
        name='lidar_nav_demo_node',
        output='screen',
        parameters=[config_file, prepath_file]
    )

    # sim_tf_node = Node(
    #     package='lidar_nav_demo_cpp',
    #     executable='sim_tf_pub_node',
    #     name='sim_tf_pub_node',
    #     output='screen',
    #     parameters=[{
    #         'target_frame': 'camera_init',
    #         'child_frame': 'aft_mapped',
    #         'update_rate': 50.0
    #     }]
    # )

    return LaunchDescription([
        # sim_tf_node,
        lidar_nav_node
    ])