"""仅启动 lidar_pose 桥接：配合 point_lio 使用，无需 task_race / 遥控器任务模式。"""
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    config = '/home/cat/hitcrt_quad2026_ws/src/config/quad_run_cfg.yaml'

    bridge_node = Node(
        package='task_manager',
        executable='lidar_pose_bridge_node',
        name='lidar_pose_bridge_node',
        namespace='quad',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        LogInfo(msg=(
            'lidar_pose_bridge: 发布 /quad/lidar_pose_xyyaw、/quad/odom、map->odom->base_link，'
            '无需遥控器进入任务/导航模式。'
            '请先启动: ros2 launch point_lio mapping_mid360.launch.py'
        )),
        bridge_node,
    ])
