"""
Тестовый launch для проверки походки без джойстика.
Запускает минимальный набор нод + gait_test_node который автоматически:
1. Встаёт (STAND_UP)
2. Идёт вперёд 10 секунд (RIPPLE gait)
3. Останавливается и садится (SEAT_DOWN)
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
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

    return LaunchDescription([
        port_name_arg,
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        
        # Leg IK Service
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
        
        # Body Kinematics
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
        
        # Gait Generator
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
        
        # Maestro Servo Driver
        Node(
            package='maestro_driver',
            executable='servo_node',
            parameters=[{
                'port_name': LaunchConfiguration('port_name'),
                'baud_rate': 115200,
            }],
        ),
        
        # Gait Test Node - автоматически запускает тест
        Node(
            package='crab_gait',
            executable='gait_test_node',
            output='screen',
        ),
    ])
