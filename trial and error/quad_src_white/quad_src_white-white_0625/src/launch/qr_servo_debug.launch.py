import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    base_config = '/home/cat/hitcrt_quad2026_ws/src/config/quad_run_cfg.yaml'
    obstacle_config = (
        '/home/cat/hitcrt_quad2026_ws/src/commands/task_manager/config/obstacle_race_params.yaml'
    )

    imu_config = os.path.join(
        get_package_share_directory('hipnuc_imu'),
        'config',
        'hipnuc_config.yaml',
    )

    aruco_node = Node(
        package='aruco',
        executable='aruco_processor_node',
        name='aruco',
        namespace='quad',
        output='screen',
        parameters=[base_config],
    )

    task_manager_node = Node(
        package='task_manager',
        executable='task_manager_node',
        name='task_manager_node',
        namespace='quad',
        output='screen',
        parameters=[
            base_config,
            obstacle_config,
            {'mission_mode': 'obstacle'},
        ],
        remappings=[
            ('task_done', 'task_done_debug_ignored'),
        ],
    )

    task_executor_node = Node(
        package='task_manager',
        executable='task_executor_node',
        name='task_executor_node',
        namespace='quad',
        output='screen',
        parameters=[base_config, obstacle_config],
    )

    joystick_input_node = Node(
        package='joystick_input',
        executable='joystick_input_node',
        name='joy_pub',
        namespace='quad',
        output='screen',
    )

    joystick_handler_node = Node(
        package='joystick_input',
        executable='joystick_handler_node',
        name='joy_handler',
        namespace='quad',
        output='screen',
    )

    state_machine_node = Node(
        package='state_machine',
        executable='state_machine',
        name='sm',
        namespace='quad',
        output='screen',
    )

    pos_control_node = Node(
        package='pos_controller',
        executable='pos_control_node',
        name='pos_control_node',
        namespace='quad',
        output='screen',
        parameters=[base_config],
    )

    rl_control_node = Node(
        package='rl_gait_controller',
        executable='rl_control_node',
        name='rl_control_node',
        namespace='quad',
        output='screen',
        parameters=[base_config],
    )

    inference_node = Node(
        package='rknn_quad_node',
        executable='rknn_quad_node',
        name='rknn_quad_node',
        namespace='quad',
        output='screen',
        parameters=[base_config],
    )

    motor_node = Node(
        package='unitree_motor_node',
        executable='unitree_motor_node',
        name='unitree_motor_node',
        namespace='quad',
        output='screen',
    )

    imu_node = Node(
        package='hipnuc_imu',
        executable='talker',
        name='IMU_publisher',
        output='screen',
        parameters=[imu_config],
    )

    return LaunchDescription([
        aruco_node,
        task_manager_node,
        task_executor_node,
        joystick_input_node,
        joystick_handler_node,
        state_machine_node,
        pos_control_node,
        rl_control_node,
        inference_node,
        motor_node,
        imu_node,
    ])
