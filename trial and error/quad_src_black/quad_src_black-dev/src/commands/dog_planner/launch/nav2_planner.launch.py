import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('dog_planner')
    config_file = os.path.join(pkg_dir, 'config', 'planner.yaml')
    nav2_config_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # 我们自定义的全局规划器 (A*)
    global_planner_node = Node(
        package='dog_planner',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[config_file]
    )

    # 官方 Nav2 DWB 的桥接节点 (代替了我们手写的 local_planner_node)
    local_planner_node_nav2 = Node(
        package='dog_planner',
        executable='local_planner_node_nav2',
        name='local_planner_node_nav2',
        output='screen'
    )

    # 启动官方的 Nav2 Controller Server (它内部加载了 dwb_core::DWBLocalPlanner)
    nav2_controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config_file]
    )

    # 启动 Nav2 Lifecycle Manager 来激活 Controller Server
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['controller_server']}]
    )

    return LaunchDescription([
        global_planner_node,
        local_planner_node_nav2,
        nav2_controller_server,
        nav2_lifecycle_manager
    ])
