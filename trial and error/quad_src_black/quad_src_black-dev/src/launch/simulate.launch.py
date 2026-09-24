import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    # config = os.path.join(
    #     get_package_share_directory("quad"),
    #     'config',
    #     'simulate_cfg.yaml'
    # )
    config = "/home/cat/hitcrt_quad_ws/src/config/quad_simulate_cfg.yaml"

    return LaunchDescription([
        # Node(
        #     package="simulate",
        #     executable="simulate_node",
        #     name="sim",
        #     namespace="quad",
        #     parameters=[
        #         config
        #     ]
        # ),

        Node(
            package="aruco",
            executable="aruco_processor_node",
            name="aruco",
            namespace="quad",
        ),
        Node(
            package="joystick_input",
            executable="joystick_input_node",
            name="joy_input",
            namespace="quad",
        ),
        Node(
            package="joystick_input",
            executable="joystick_handler_node",
            name="joy_handler",
            namespace="quad",
        ),
        Node(
            package="task_manager",
            executable="task_manager_node",
            name="task_manager_node",
            namespace="quad",
            parameters=[
                config
            ]
        ),
        Node(
            package="state_machine",
            executable="state_machine",
            name="sm",
            namespace="quad",
        ),
        Node(
            package="pos_controller",
            executable="pos_control_node",
            name="pos_control_node",
            namespace="quad",
            parameters=[
                config
            ]
        ),
        Node(
            package="rl_gait_controller",
            executable="rl_control_node",
            name="rl_control_node",
            namespace="quad",
            parameters=[
                config
            ]
        ),        
        Node(
            package="rknn_quad_node",
            executable="rknn_quad_node",
            name="rknn_quad_node",
            namespace="quad",
            parameters=[
                config
            ]
        ),

        Node(
            package="joint_pd_controller",
            executable="joint_control_node",
            name="joint_control_node",
            namespace="quad",
        )

    ])
