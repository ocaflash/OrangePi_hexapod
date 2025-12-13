import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('crab_description')
    xacro_file = os.path.join(pkg_share, 'models', 'crab_model.xacro')
    
    robot_description = Command(['xacro ', xacro_file], on_stderr='ignore')
    
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
            parameters=[{'robot_description': robot_description}],
        ),
        
        # Gait Generator
        Node(
            package='crab_gait',
            executable='gait_kinematics',
            parameters=[{'robot_description': robot_description}],
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
