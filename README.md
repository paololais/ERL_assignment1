# Experimental Robotics Laboratory – Assignment 1

## Overview
This package has been developed for the *Experimental Robotics Laboratory* course  
(Master’s Degree in Robotics Engineering, University of Genoa).

It implements a ROS2 node that enables a mobile robot to autonomously:

1. Rotate 360° to detect all ArUco markers in the environment.  
2. Keep track of the IDs and angular positions of the markers.  
3. Move toward the marker with the **lowest ID**.  
4. Center the marker in the camera image, annotate it and publish the result.  
5. Repeat the behavior for all remaining markers in **ascending ID order**.

It contains two robot models:
- mogi_bot.urdf – a 2-wheel differential-drive robot
- mogi_bot_4wheels.urdf – a 4-wheel robot using mecanum wheels

The 4-wheel model uses a different motion plugin (mecanum-drive) instead of the standard diff-drive used in the 2-wheel version.
Additionally, an odometry publisher was integrated into the 4-wheel model to compensate for known odometry issues with the mecanum plugin in ROS2 Humble.
To switch between the robots, simply change the model_arg parameter inside spawn_robot_aruco.launch.py to either model name.

Moreover, it was also tested on a physical ROSBot, successfully obtaining the desired behaviour. The implementation, which can be found on branch "rosbot", was slightly modified to get a more robust solution to the real world environment and conditions.

The package was originally tested using **ROS2 Humble** on **Ubuntu 22.04**, but it can be easily adapted to other ROS2 distributions.

---

### World
The file `aruco_world.sdf` defines an environment containing **five ArUco markers**.

### Marker Generation
Markers are generated using the package:

https://github.com/SaxionMechatronics/ros2-gazebo-aruco

Run:

```bash
ros2 run ros2_aruco aruco_generate_marker
```

Before generating the markers, minor modifications were applied to ensure the use of
DICT_ARUCO_ORIGINAL.

The generated marker (a cube with 6 faces) replaces the texture of the aruco_box model
from the same repository (gz-world/aruco_box).

### Adding Markers in Gazebo

1. Launch an empty world:
```bash
gz sim empty.sdf
```
2. Add 5 ArUco boxes using the Resource Spawner.
3. Place the boxes in a circle, each rotated so a different marker faces the robot at the center.
4. Save the world and place it inside the package’s worlds/ directory.
5. Ensure your Gazebo model path is set correctly in your .bashrc.

### Robot

The simulated robot used is the mogi_bot (as introduced in the course).

A standard camera sensor is used for image acquisition.

### Launch File

A custom launch file is provided to:
- Load the aruco_world.sdf world.
- Spawn the mogi_bot at the center of the 5 aruco boxes.
- Initialize all required ROS2 interfaces for the simulation.

Launch file location:
assignment1/launch/spawn_robot_aruco.launch.py

### Node: detect_aruco_node
This ROS2 node autonomously detects and processes ArUco markers using a simple finite state machine (FSM) with three states: rotating → centering → done.

The node first performs a 360° rotation to detect all markers and record their approximate angles. It then processes them in ascending ID order: for each marker, it rotates to bring it into view, uses a proportional controller to horizontally center it in the camera image, and publishes an annotated image on /aruco_centered_image. 

After all markers are centered and published, the node stops.

## Installation
- Clone the package inside your ROS2 workspace src folder:
```bash
cd ~/ros2_ws/src/
git clone https://github.com/paololais/ERL_assignment1.git
```

- Build the package and source:
```bash
cd ~/ros2_ws
colcon build
source install/local_setup.bash
```

## Usage
- Launch the Gazebo simulation and Rviz:
```bash
ros2 launch assignment1 spawn_robot_aruco.launch.py
```

- Run the node:
```bash
ros2 run assignment1 detect_aruco_node
```