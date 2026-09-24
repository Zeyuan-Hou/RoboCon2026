"""
Launch file for manipulator system
同时启动:
- 3508_motor_node: 电机驱动节点
- arm_node: 机械臂控制节点
- pump_node: 气泵控制节点
- manipulator_manager_node: 机械臂管理节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明参数
    declare_arm_interpolate_time = DeclareLaunchArgument(
        'arm_interpolate_time',
        default_value='1.2',
        description='机械臂插值过渡时间(s)'
    )

    declare_control_freq = DeclareLaunchArgument(
        'control_freq_hz',
        default_value='400.0',
        description='控制频率(Hz)'
    )

    # 3508电机驱动节点
    motor_node = Node(
        package='3508_motor_node',
        executable='3508_motor_node',
        name='motor_driver',
        namespace='quad',
        output='screen',
        parameters=[{
            'control_freq_hz': LaunchConfiguration('control_freq_hz'),
        }]
    )

    # 机械臂控制节点
    arm_node = Node(
        package='arm_node',
        executable='arm_node',
        name='arm_controller',
        namespace='quad',
        output='screen',
        parameters=[{
            'control_freq_hz': LaunchConfiguration('control_freq_hz'),
            'arm_interpolate_time': LaunchConfiguration('arm_interpolate_time'),
            # 各状态目标位置
            'grab_motor0_pos': 0.6,
            'grab_motor1_pos': 2.5,
            'lift_motor0_pos': 3.5,
            'lift_motor1_pos': -3.5,
            'place_low_motor0_pos': -0.3,
            'place_low_motor1_pos': -0.5,
            'place_high_motor0_pos': 2.0,
            'place_high_motor1_pos': -1.5,
            # PID参数
            'motor0_Kp': 3000.0,
            'motor0_Ki': 0.2,
            'motor0_Kd': 0.2,
            'motor1_Kp': 2000.0,
            'motor1_Ki': 0.1,
            'motor1_Kd': 0.2,
            # 软限位
            'soft_pos_limit_enabled': True,
            'soft_pos_limit_threshold': 6.0,
        }]
    )

    # 气泵控制节点
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
            'cmd_interval_ms': 500,  # 指令最小间隔
        }]
    )

    # 机械臂管理节点
    manager_node = Node(
        package='manipulator_manager_node',
        executable='manipulator_manager_node',
        name='manipulator_manager',
        output='screen',
        parameters=[{
            'control_freq_hz': 50.0,
            # GRAB任务时间
            'grab_arm_time_ms': 1500,
            'grab_pump_time_ms': 800,
            'grab_lift_time_ms': 1500,
            # PLACE_LOW任务时间
            'place_low_arm_time_ms': 2000,
            'place_low_release_time_ms': 500,
            'place_low_lift_time_ms': 1500,
            # PLACE_HIGH任务时间
            'place_high_arm_time_ms': 2000,
            'place_high_release_time_ms': 500,
            'place_high_lift_time_ms': 1500,
        }]
    )

    return LaunchDescription([
        # 参数声明
        declare_arm_interpolate_time,
        declare_control_freq,
        # 节点
        motor_node,
        arm_node,
        pump_node,
        manager_node,
    ])
