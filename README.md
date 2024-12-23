# RobLab_hw4
README
This repository contains the packages required for the homework 4.
Follow the instructions to run the packages properly.

## Build and source the packages
Clone the repository in your ROS2 workspace and build the packages.

build
```bash
colcon build
```
then source

```bash
source install/setup.bash
```

Visualize on Rviz

```bash
ros2 launch rl_fra2mo_description display_fra2mo.launch.py
```

Start Gazebo simulation
```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```

To visualize the aruco
```bash
rqt
```

FOR GOALS EXERCISE, after running the previous commands, open 2 terminals and then: 

```bash
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```

To run the node and select the desired option

```bash
ros2 run rl_fra2mo_description follow_waypoints.py
```

FOR EXERCISE 4 open 5 terminals in this order:

```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```

To launch detection node
```bash
ros2 launch rl_fra2mo_description detection.launch.py
```

To detect aruco

```bash
ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=115 -p marker_size:=0.1 -p reference_frame:=camera_link_optical -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical
```

(For point c)

```bash
ros2 topic echo /tf_static
```

To run detect_aruco
```bash
ros2 run rl_fra2mo_description detect_aruco.py
```

