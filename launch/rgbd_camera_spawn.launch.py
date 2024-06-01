from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'package',
            default_value='sensor_moveit_perception',
            description='Package name'
        ),
        DeclareLaunchArgument(
            'camera_urdf_path',
            default_value='rgbd_camera_world.urdf.xacro',
            description='URDF/XACRO path.'
        ),
        DeclareLaunchArgument(
            'ns', 
            default_value='rgbd',
            description='Specify namespace.'
        ),
        DeclareLaunchArgument(
            'x', 
            default_value='3.0',
            description='Specify x pose.'
        ),
        DeclareLaunchArgument(
            'y', 
            default_value='0.0',
            description='Specify y pose.'
        ),
        DeclareLaunchArgument(
            'z', 
            default_value='0.5',
            description='Specify z pose.'
        ),
        DeclareLaunchArgument(
            'roll', 
            default_value='0.0',
            description='Specify roll pose.'
        ),
        DeclareLaunchArgument(
            'pitch', 
            default_value='0.0',
            description='Specify pitch pose.'
        ),
        DeclareLaunchArgument(
            'yaw', 
            default_value='3.14159',
            description='Specify yaw pose.'
        ),
    ]

    # Initialize Arguments
    package = LaunchConfiguration('package')
    camera_urdf_path = LaunchConfiguration('camera_urdf_path')
    ns = LaunchConfiguration('ns')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # Construct path to the camera URDF file
    camera_file = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(package), 'urdf', camera_urdf_path]
            ),
            ' ',
            'namespace:=', ns,
            ' ',
            'x:=', x,
            ' ',
            'y:=', y,
            ' ',
            'z:=', z,
            ' ',
            'roll:=', roll,
            ' ',
            'pitch:=', pitch,
            ' ',
            'yaw:=', yaw,
        ]
    )

    camera_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        output='both',
        parameters=[
            {'robot_description': ParameterValue(camera_file, value_type=str)},
            {'use_sim_time': True},
        ],
        remappings=[
            ('robot_description', 'robot_description'),
        ]
    )

    # Load and spawn the camera description in Gazebo
    spawn_camera = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_camera',
        namespace=ns,
        arguments=[
            '-topic', 'robot_description',
            '-entity', [ns,'_camera'],
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.0', 
            '-R', '0.0', 
            '-P', '0.0', 
            '-Y', '0.0',
        ],
        output='screen'
    )

    nodes = [
        camera_state_publisher,
        spawn_camera,
    ]

    # Create the move_group node with parameters and namespace (if provided)
    return LaunchDescription(declared_arguments + nodes)
