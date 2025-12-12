import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('crab_description')
    xacro_file = os.path.join(pkg_share, 'models', 'crab_model.xacro')
    
    robot_description = Command(['xacro ', xacro_file], on_stderr='ignore')

    return LaunchDescription([
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
        
        # Joy node (драйвер джойстика)
        Node(
            package='joy',
            executable='joy_node',
        ),
    ])
