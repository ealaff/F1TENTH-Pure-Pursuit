# F1TENTH Pure Pursuit Navigation

This repository contains the implementation of the Pure Pursuit algorithm for autonomous path following in the F1TENTH simulator. It includes two versions: **ROS 2 Humble (Ubuntu 22.04)** and **ROS 2 Jazzy (Ubuntu 24.04)**, each with its own Python controller and launch file. Both versions use odometry feedback, Ackermann steering, and predefined waypoints to navigate a racing track, with visualization through RViz.

### Launch Commands

**ROS 2 Humble:**
```bash
ros2 launch ~/f1tenth_ws/start_f1tenth.launch.py
```

**ROS 2 Jazzy:**
```bash
ros2 launch ~/f1tenth_ws/start_f1tenth_jazzy.launch.py
```

**Note:** The F1TENTH simulator and required ROS 2 dependencies must be installed before running either version.
