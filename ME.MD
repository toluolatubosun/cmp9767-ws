# CONNECT TO PHYSICAL ROB0T
zenoh-bridge-ros2dds -e tcp/10.82.0.97:7447

# CONNECT TO SIMULATION
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py
## inspect
rviz2 -d /opt/ros/lcas/install/limo_gazebosim/share/limo_gazebosim/rviz/urdf.rviz

# Movement
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Move in cricle
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"