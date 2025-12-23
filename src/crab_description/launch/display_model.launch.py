import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
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
        'coxa_length:=' + str(geometry_config['leg']['coxa_length']),
        'femur_length:=' + str(geometry_config['leg']['femur_length']),
        'tibia_length:=' + str(geometry_config['leg']['tibia_length']),
        'joint_lower_limit:=' + str(geometry_config['joint_limits']['lower']),
        'joint_upper_limit:=' + str(geometry_config['joint_limits']['upper']),
        PythonExpression([
            "'use_primitives:=' + '", LaunchConfiguration('use_primitives'), "'"
        ]),
    ]

    robot_description = Command([
        'xacro', xacro_file,
        *xacro_args
    ])

    return LaunchDescription([
        use_primitives_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
