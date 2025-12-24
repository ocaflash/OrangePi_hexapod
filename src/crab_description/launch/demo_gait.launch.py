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
    rviz_config = os.path.join(pkg_share, 'rviz', 'urdf.rviz')

    with open(config_file, 'r') as f:
        geometry_config = yaml.safe_load(f)

    use_primitives_arg = DeclareLaunchArgument(
        'use_primitives',
        default_value='false',
        description='Use primitive geometry instead of STL meshes'
    )

    xacro_args = [
        ' coxa_length:=' + str(geometry_config['leg']['coxa_length']),
        ' femur_length:=' + str(geometry_config['leg']['femur_length']),
        ' tibia_length:=' + str(geometry_config['leg']['tibia_length']),
        ' joint_lower_limit:=' + str(geometry_config['joint_limits']['lower']),
        ' joint_upper_limit:=' + str(geometry_config['joint_limits']['upper']),
        PythonExpression([
            "' use_primitives:=' + '", LaunchConfiguration('use_primitives'), "'"
        ]),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, *xacro_args]),
        value_type=str
    )

    return LaunchDescription([
        use_primitives_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='crab_leg_kinematics',
            executable='leg_ik_service',
            name='crab_leg_kinematics',
            parameters=[{'robot_description': robot_description}],
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
                {'path_tolerance': geometry_config['gait']['path']['tolerance']},
                {'rounded_radius': geometry_config['gait']['path']['rounded_radius']},
                {'duration_ripple': geometry_config['gait']['timing']['ripple_duration']},
                {'duration_tripod': geometry_config['gait']['timing']['tripod_duration']},
            ],
        ),
        Node(
            package='crab_joint_publisher',
            executable='joint_publisher',
        ),
        Node(
            package='crab_description',
            executable='demo_gait.py',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
