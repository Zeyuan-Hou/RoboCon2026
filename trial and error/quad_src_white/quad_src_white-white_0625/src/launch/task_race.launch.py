import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = '/home/cat/hitcrt_quad2026_ws/src/config/quad_run_cfg.yaml'

    imu_config = os.path.join(
        get_package_share_directory('hipnuc_imu'),
        'config',
        'hipnuc_config.yaml',
    )

    default_waypoints_yaml = (
        '/home/cat/hitcrt_quad2026_ws/src/commands/lidar_nav_demo_cpp/config/waypoints.yaml'
    )
    prepath_yaml = '/home/cat/hitcrt_quad2026_ws/src/commands/lidar_nav_demo_cpp/config/prepath.yaml'

    declare_waypoints_file = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints_yaml,
        description=(
            'Lidar nav waypoints yaml. Set use_field_layout_generated_points in waypoints.yaml '
            'to load manip_points_generated.yaml at startup.'
        ),
    )
    declare_arm_interpolate_time = DeclareLaunchArgument(
        'arm_interpolate_time',
        default_value='1.2',
        description='Manipulator interpolation time in seconds.'
    )
    declare_manipulator_control_freq = DeclareLaunchArgument(
        'manipulator_control_freq_hz',
        default_value='400.0',
        description='Manipulator motor control frequency.'
    )
    declare_anchor_recorder_enable = DeclareLaunchArgument(
        'anchor_recorder_enable',
        default_value='false',
        description='Start record_anchor_points.py for LB+RB / LT+RT anchor logging.',
    )

    anchor_recorder_script = (
        '/home/cat/hitcrt_quad2026_ws/src/pythontool/record_anchor_points.py'
    )
    task_manager_node = Node(
        package='task_manager',
        executable='task_manager_node',
        name='task_manager_node',
        namespace='quad',
        output='screen',
        parameters=[
            config,
            LaunchConfiguration('waypoints_file'),
            {
                'mission_mode': 'logistics',
                'manipulation.command_topic': '/manipulator/cmd',
            },
        ],
    )

    task_executor_node = Node(
        package='task_manager',
        executable='task_executor_node',
        name='task_executor_node',
        namespace='quad',
        output='screen',
        parameters=[config],
    )

    lidar_nav_demo_node = Node(
        package='lidar_nav_demo_cpp',
        executable='lidar_nav_demo_node',
        name='lidar_nav_demo_node',
        namespace='quad',
        output='screen',
        parameters=[LaunchConfiguration('waypoints_file'), prepath_yaml],
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
        parameters=[config],
    )

    rl_control_node = Node(
        package='rl_gait_controller',
        executable='rl_control_node',
        name='rl_control_node',
        namespace='quad',
        output='screen',
        parameters=[config],
    )

    inference_node = Node(
        package='rknn_quad_node',
        executable='rknn_quad_node',
        name='rknn_quad_node',
        namespace='quad',
        output='screen',
        parameters=[config],
    )

    quad_motor_node = Node(
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

    # ---- Manipulator stack ----
    manipulator_motor_node = Node(
        package='3508_motor_node',
        executable='3508_motor_node',
        name='motor_driver',
        namespace='quad',
        output='screen',
        parameters=[{
            'control_freq_hz': LaunchConfiguration('manipulator_control_freq_hz'),
        }],
    )

    arm_node = Node(
        package='arm_node',
        executable='arm_node_td',
        name='arm_controller',
        namespace='quad',
        output='screen',
        parameters=[{
            'control_freq_hz': LaunchConfiguration('manipulator_control_freq_hz'),
            'arm_interpolate_time': LaunchConfiguration('arm_interpolate_time'),
            'grab_motor0_pos': 0.5,
            'grab_motor1_pos': 2.5,
            'lift_motor0_pos': 3.5,
            'lift_motor1_pos': 0.0,
            'place_low_motor0_pos': 1.2,
            'place_low_motor1_pos': 1.5,
            'lay_motor0_pos': -0.5,
            'lay_motor1_pos': 0.0,
            'place_high_motor0_pos': 2.5,
            'place_high_motor1_pos': 0.5,
            'motor0_Kp': 8000.0,
            'motor0_Ki': 0.2,
            'motor0_Kd': 0.2,
            'motor1_Kp': 3000.0,
            'motor1_Ki': 0.1,
            'motor1_Kd': 0.2,
            'soft_pos_limit_enabled': True,
            'soft_pos_limit_threshold': 6.0,
            'td_r': 50.0,
            'td_h': 0.15,
            'trajectory_speed_scale': 10.0,
            'tracking_td_r': 50.0,
            'tracking_td_h': 0.1,
            'shake_amplitude': 1.0,
            'shake_frequency': 1.0,
            'shake_duration': 1.0,
            'shake_td_r': 50.0,
            'shake_td_h': 0.05,
            'tracking_wait_timeout_s': 10.0,
            'approach_ready_topic': '/quad/logistics_pickup_reached',
        }],
    )

    pump_node = Node(
        package='pump_node',
        executable='pump_node',
        name='pump_controller',
        namespace='quad',
        output='screen',
        parameters=[{
            'pump_chip': 'gpiochip1',
            'pump_line': 14,
            'valve_chip': 'gpiochip1',
            'valve_line': 15,
            'cmd_interval_ms': 500,
        }],
    )

    manipulator_manager_node = Node(
        package='manipulator_manager_node',
        executable='manipulator_manager_node',
        name='manipulator_manager',
        output='screen',
        parameters=[{
            'control_freq_hz': 50.0,
            # GRAB任务参数（基于信号同步）
            'arm_arrival_timeout_ms': 3000,
            'approach_ready_timeout_ms': 10000,
            'trajectory_complete_timeout_ms': 50000,
            'grab_pump_time_ms': 500,
            'grab_lift_time_ms': 1000,
            # PLACE_LOW任务
            'place_low_arm_time_ms': 1500,
            'place_low_release_time_ms': 400,
            'place_low_lift_time_ms': 1000,
            # PLACE_HIGH任务
            'place_high_arm_time_ms': 1000,
            'place_high_release_time_ms': 600,
            'place_high_lift_time_ms': 1000,
            # 待机任务
            'init_standby_time_ms': 1000,
            'lift_standby_time_ms': 1000,
        }],
    )

    anchor_point_recorder = ExecuteProcess(
        cmd=['python3', anchor_recorder_script],
        output='screen',
        condition=IfCondition(LaunchConfiguration('anchor_recorder_enable')),
    )

    return LaunchDescription([
        declare_waypoints_file,
        declare_arm_interpolate_time,
        declare_manipulator_control_freq,
        declare_anchor_recorder_enable,
        anchor_point_recorder,
        task_manager_node,
        lidar_nav_demo_node,
        joystick_input_node,
        joystick_handler_node,
        state_machine_node,
        pos_control_node,
        rl_control_node,
        inference_node,
        quad_motor_node,
        imu_node,
        task_executor_node, 
        manipulator_motor_node,
        arm_node,
        pump_node,
        manipulator_manager_node,
    ])
