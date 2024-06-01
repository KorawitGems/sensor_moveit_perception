from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    spawn_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare('sensor_moveit_perception'),'launch', 'rgbd_camera_spawn.launch.py']
            )]
        ),
        launch_arguments={
            'ns': 'rgbd2',
            'x': '0.0',
            'y': '3.0',
            'z': '1.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '4.7123',
        }.items(),
    )

    return LaunchDescription([spawn_camera])
