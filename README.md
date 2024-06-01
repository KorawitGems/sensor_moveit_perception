## sensor_moveit_perception

# Installation

1. Install ROS2 Humble, MoveIt2, moveit_ros_perception and gazebo_ros_pkgs.

2. Clone sensor_moveit_perception in ROS2 workspace

```bash
git clone https://github.com/KorawitGems/sensor_moveit_perception.git -b humble
```

# Run

Test camera spawn
```bash
ros2 launch sensor_moveit_perception test_camera_spawn.launch.py
```

# Integrated with robot

1. Include sensor_moveit_perception/moveit2/sensors_pointcloud.yaml in move_group node parameters
2. Include sensor_moveit_perception/launch/rgbd_camera_spawn.launch.py in robot launch.py by set arguments like example test_camera_spawn.launch.py

# Example

Clone my modified packages
```bash
git clone https://github.com/KorawitGems/elfin_robot_ros2.git
```

Run

```bash
ros2 launch elfin3_ros2_moveit2 elfin3_rgbd.launch.py
```