import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('crab_description')
    xacro_file = os.path.join(pkg_share, 'models', 'crab_model.xacro')
    config_file = os.path.join(pkg_share, 'config', 'robot_geometry.yaml')
    
    robot_description = ParameterValue(
        Command(['xacro', xacro_file], on_stderr='ignore'),
        value_type=str
    )
    
    # Загрузка конфига геометрии
    with open(config_file, 'r') as f:
        geometry_config = yaml.safe_load(f)
    
    # Аргумент для порта Maestro
    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyS5',
        description='Serial port for Maestro servo controller'
    )

    return LaunchDescription([
        port_name_arg,
        
        # Robot State Publisher (публикует URDF)
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
            parameters=[{'robot_description': robot_description}],
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
                {'path_tolerance': geometry_config['gait']['path']['tolerance']},
                {'rounded_radius': geometry_config['gait']['path']['rounded_radius']},
                {'duration_ripple': geometry_config['gait']['timing']['ripple_duration']},
                {'duration_tripod': geometry_config['gait']['timing']['tripod_duration']},
            ],
        ),
        
        # Joint Publisher (для визуализации)
        Node(
            package='crab_joint_publisher',
            executable='joint_publisher',
        ),
        
        # IMU Control
        Node(
            package='crab_imu',
            executable='imu_control',
        ),
        
        # Joystick Teleop
        Node(
            package='crab_teleop_joy',
            executable='teleop_joy',
        ),
        
        # DS4 IMU Publisher (reads gyroscope from DualShock 4)
        Node(
            package='crab_teleop_joy',
            executable='ds4_imu_publisher',
        ),
        
        # Maestro Servo Driver
        Node(
            package='maestro_driver',
            executable='servo_node',
            parameters=[{
                'port_name': LaunchConfiguration('port_name'),
                'baud_rate': 115200
            }],
        ),
        
        # Joy node (драйвер джойстика)
        Node(
            package='joy',
            executable='joy_node',
        ),
    ])
