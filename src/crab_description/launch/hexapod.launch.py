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

    # --- Joystick device selection (for joy_node) ---
    # Many controllers appear as /dev/input/js0, but can also be js1/js2 depending on boot order.
    # Allow override via environment variable JOY_DEV, and expose as launch argument joy_dev.
    joy_dev_default = os.environ.get('JOY_DEV', '').strip()
    if not joy_dev_default:
        for candidate in ('/dev/input/js0', '/dev/input/js1', '/dev/input/js2', '/dev/input/js3'):
            if os.path.exists(candidate):
                joy_dev_default = candidate
                break
    if not joy_dev_default:
        joy_dev_default = '/dev/input/js0'
    
    # Загрузка конфига геометрии
    with open(config_file, 'r') as f:
        geometry_config = yaml.safe_load(f)

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
        Command(['xacro ', xacro_file, *xacro_args], on_stderr='ignore'),
        value_type=str
    )
    
    # Аргументы
    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyS5',
        description='Serial port for Maestro servo controller'
    )
    maestro_protocol_arg = DeclareLaunchArgument(
        'maestro_protocol',
        default_value=os.environ.get('MAESTRO_PROTOCOL', 'mini_ssc'),
        description='Maestro serial protocol: compact or mini_ssc'
    )
    imu_autostart_arg = DeclareLaunchArgument(
        'imu_autostart',
        default_value='false',
        description='Automatically enable IMU stabilization at startup'
    )
    use_primitives_arg = DeclareLaunchArgument(
        'use_primitives',
        default_value='false',
        description='Use primitive geometry instead of STL meshes'
    )
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value=joy_dev_default,
        description='Joystick device path for joy_node (e.g. /dev/input/js0)'
    )
    allow_partial_ik_arg = DeclareLaunchArgument(
        'allow_partial_ik',
        default_value='false',
        description='Allow best-effort IK (do not block all joints if a leg IK fails) - useful for testing'
    )

    return LaunchDescription([
        port_name_arg,
        maestro_protocol_arg,
        use_primitives_arg,
        imu_autostart_arg,
        joy_dev_arg,
        allow_partial_ik_arg,
        
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
            parameters=[
                {'robot_description': robot_description},
                {'allow_partial_ik': LaunchConfiguration('allow_partial_ik')},
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
            parameters=[{'auto_start': LaunchConfiguration('imu_autostart')}],
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
                'baud_rate': 115200,
                'protocol': LaunchConfiguration('maestro_protocol'),
            }],
        ),
        
        # Joy node (драйвер джойстика)
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
            }],
        ),
    ])
