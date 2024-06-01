from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # DECLARE Gazebo WORLD file:
    gazebo_world = PathJoinSubstitution(
        [FindPackageShare('sensor_moveit_perception'), 'worlds', 'test_empty.world']
    )

    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'),'launch', 'gazebo.launch.py'])]
        ),
        launch_arguments={'verbose': 'false', 'world': gazebo_world}.items(),
    )

    nodes = [
        gazebo,
    ]

    # Create the move_group node with parameters and namespace (if provided)
    return LaunchDescription(
        nodes
    )
