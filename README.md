# ROS2 TurtleBot Explorer

A ROS2-based autonomous exploration robot developed using TurtleBot3 and LiDAR sensor processing in Gazebo simulation.

---

# Project Overview

This project implements an autonomous mobile robot capable of:

* Obstacle avoidance
* Autonomous exploration
* Dynamic turning behaviour
* Recovery from wall collisions
* LiDAR-based environment sensing

The robot was developed and tested using ROS2 and TurtleBot3 in a Gazebo simulation environment.

---

# Features

## Autonomous Navigation

The robot can move independently inside unknown environments.

## LiDAR Obstacle Avoidance

LaserScan data is processed in real time to detect nearby obstacles and avoid collisions.

## Finite State Machine (FSM)

The navigation logic is based on multiple robot states such as:

* Forward movement
* Turning
* Escape / recovery behaviour

## Recovery Behaviour

The robot can reverse and rotate when trapped near walls or corners.

---

# Technologies Used

* ROS2
* Python
* TurtleBot3
* Gazebo
* LiDAR Sensor Processing

---

# Project Structure

```text
launch/
    explore.launch.py

scripts/
    explorer.py
    explorer_verz.py

package.xml
CMakeLists.txt
```

---

# File Description

| File              | Description                         |
| ----------------- | ----------------------------------- |
| explorer.py       | Stable safer exploration controller |
| explorer_verz.py  | Improved faster exploration version |
| explore.launch.py | ROS2 launch configuration           |
| package.xml       | ROS2 package configuration          |
| CMakeLists.txt    | Build configuration                 |

---

# How to Run

```bash
colcon build
source install/setup.bash
ros2 launch ele434_team20_2026 explore.launch.py
```

---

# Future Improvements

* Improved path planning
* SLAM integration
* Dynamic obstacle tracking
* Multi-robot coordination

---

# Author

WU YICHENG
University of Sheffield
MSc Robotics
