# CONNECT TO PHYSICAL ROB0T
zenoh-bridge-ros2dds -e tcp/10.82.0.97:7447

# CONNECT TO SIMULATION
## Default environment
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py
## Custom environment
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=/workspaces/cmp9767-ws/src/worlds/basic_environment.world

# Inspect sensors
rviz2 -d /opt/ros/lcas/install/limo_gazebosim/share/limo_gazebosim/rviz/urdf.rviz
rviz2 -d /opt/ros/lcas/install/limo_gazebosim/share/limo_gazebosim/rviz/urdf.rviz world:=/workspaces/cmp9767-ws/src/worlds/basic_environment.world

# Movement
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Move in cricle
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# -------------------------------------

# Command to view frames (Generates PDF to view sensors)
ros2 run tf2_tools view_frames

# Install Transfrom
sudo apt update
sudo apt install ros-humble-tf2-ros

# View Transform Between Target and Source
ros2 run tf2_ros tf2_echo <target> <source>
ros2 run tf2_ros tf2_echo odom depth_camera_link

# Get the frame the laser scanner is in
ros2 topic echo /scan

# -------------------------------------

# ROS2 Package
For `my_robot_package` you can use any name

```bash
# Create a package
ros2 pkg create --build-type ament_python my_robot_package --dependencies rclpy geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs

# Build workspace
colcon build --symlink-install

# Set enviroment so ros2 can find new packages
source install/setup.bash

# Run the package
ros2 run my_robot_package move_circle
```

# -------------------------------------

# Install RQT Image View
## Inspect image topics
sudo apt-get install ros-humble-rqt-image-view
## Save image topics
sudo apt-get install ros-humble-image-view

# Image stream coming from the robot
ros2 run image_view image_view --ros-args -r image:=/limo/depth_camera_link/image_raw
OR 
ros2 run rqt_image_view rqt_image_view (Then you select the topic)

# ROS Bags
## Record topic
ros2 bag record -o <filename> <topic(s)>
> ros2 bag record -o my_bag /cmd_vel /scan
## Play ros bag
ros2 bag play <filename>
> ros2 bag play my_bag

# Stream Video/Camera to pic
## Install image publisher
sudo apt-get install ros-humble-image-publisher
## Stream to topic
ros2 run image_publisher image_publisher_node test_video.mp4
ros2 run image_publisher image_publisher_node /dev/video0 (device camere, name may differ)
## Inspect image with RQT
ros2 run rqt_image_view rqt_image_view

# -------------------------------------

# Install navigation package for autonomous movement
sudo apt install ros-humble-nav2-*

# Run autonomius movement
ros2 launch limo_navigation limo_navigation.launch.py
ros2 launch limo_navigation limo_navigation.launch.py world:=/workspaces/cmp9767-ws/src/worlds/basic_environment.world