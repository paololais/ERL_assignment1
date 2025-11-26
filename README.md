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
A wide-angle camera plugin was avoided due to compatibility issues with ROS2 Humble and Gazebo 6.

### Launch File

A custom launch file is provided to:
- Load the aruco_world.sdf world.
- Spawn the mogi_bot at the center of the 5 aruco boxes.
- Initialize all required ROS2 interfaces for the simulation.

Launch file location:
assignment1/launch/spawn_robot_aruco.launch.py

### Node: detect_aruco_node

This node implements the full assignment requirements.

#### Features

- Performs a 360° rotation while detecting ArUco markers.

- Stores each marker’s ID and the relative angle at which it was detected

- Once all markers are found:

    - Moves toward the marker with the lowest ID

    - Centers the marker in the camera view

    - Draws a circle over the detected marker

    -  Publishes the annotated image on /aruco_centered_image topic

- Repeats the same behavior for the other markers in ascending ID order

- Stops when all markers are processed

The node is written in Python and uses cv2.aruco for detection.

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
