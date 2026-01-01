"""
Демо походки на реальных сервах.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('crab_description')
    xacro_file = os.path.join(pkg_share, 'models', 'crab_model.xacro')
    config_file = os.path.join(pkg_share, 'config', 'robot_geometry.yaml')

    with open(config_file, 'r') as f:
        geometry_config = yaml.safe_load(f)

    xacro_args = [
        ' coxa_length:=' + str(geometry_config['leg']['coxa_length']),
        ' femur_length:=' + str(geometry_config['leg']['femur_length']),
        ' tibia_length:=' + str(geometry_config['leg']['tibia_length']),
        ' joint_lower_limit:=' + str(geometry_config['joint_limits']['lower']),
        ' joint_upper_limit:=' + str(geometry_config['joint_limits']['upper']),
        ' use_primitives:=true',
    ]

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, *xacro_args], on_stderr='ignore'),
        value_type=str
    )

    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyS5',
        description='Serial port for Maestro'
    )

    # Путь к demo_gait.py
    demo_script = os.path.join(pkg_share, '..', '..', 'lib', 'crab_description', 'demo_gait.py')
    # Альтернативный путь если симлинк
    if not os.path.exists(demo_script):
        demo_script = '/home/orangepi/ros2_ws/src/OrangePi_hexapod/src/crab_description/scripts/demo_gait.py'

    return LaunchDescription([
        port_name_arg,
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='crab_leg_kinematics',
            executable='leg_ik_service',
            name='crab_leg_kinematics',
            parameters=[
                {'robot_description': robot_description},
                {'joint_lower_limit': geometry_config['joint_limits']['lower']},
                {'joint_upper_limit': geometry_config['joint_limits']['upper']},
            ],
        ),
        Node(
            package='crab_body_kinematics',
            executable='body_kinematics',
            parameters=[
                {'robot_description': robot_description},
                {'leg_radius': geometry_config['workspace']['leg_radius']},
                {'clearance': geometry_config['body_positions']['clearance']},
                {'seat_height': geometry_config['body_positions']['seat_height']},
                {'stand_step': geometry_config['body_positions']['stand_step']},
            ],
        ),
        Node(
            package='crab_gait',
            executable='gait_kinematics',
            parameters=[
                {'robot_description': robot_description},
                {'leg_radius': geometry_config['workspace']['leg_radius']},
                {'clearance': geometry_config['body_positions']['clearance']},
                {'trapezoid_low_radius': geometry_config['gait']['trapezoid']['low_radius']},
                {'trapezoid_high_radius': geometry_config['gait']['trapezoid']['high_radius']},
                {'trapezoid_h': geometry_config['gait']['trapezoid']['height']},
                {'duration_ripple': geometry_config['gait']['timing']['ripple_duration']},
                {'duration_tripod': geometry_config['gait']['timing']['tripod_duration']},
            ],
        ),
        Node(
            package='maestro_driver',
            executable='servo_node',
            parameters=[{
                'port_name': LaunchConfiguration('port_name'),
                'baud_rate': 115200,
            }],
            output='screen',
        ),
        # Запускаем demo скрипт через python3 с задержкой
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', demo_script],
                    output='screen',
                ),
            ],
        ),
    ])
