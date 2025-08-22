# Robotic Arm Simulation · ROS 2 Humble

A complete ROS 2 Humble workspace for simulating a 3-DOF robotic arm.  
RViz is used for visualisation and interactive joint control; Gazebo (optional) provides physics simulation.

---

## 1 Project Structure

![Comparison Chart](screenshots/robotic_arm_control_methods.png)

## 2 Quick Start

clone & build
git clone <repo-url>
cd robotic_arm_simulation_ros2
colcon build --symlink-install
source install/setup.bash

visualise in RViz with slider control
ros2 launch my_robotic_arm display.launch.py

text

---

## 3 Available Control Options

| Method                            | Command / Script                                                | Real-time | Notes                                    |
|----------------------------------|-----------------------------------------------------------------|-----------|------------------------------------------|
| Joint-state sliders (RViz GUI)    | `ros2 launch my_robotic_arm display.launch.py`                  | Yes       | Drag sliders in RViz                     |
| Keyboard — instant response       | `ros2 run my_robotic_arm instant_keyboard_control.py`           | Yes       | No **Enter** key required                |
| Keyboard — press Enter            | `ros2 run my_robotic_arm keyboard_control.py`                   | No        | Simpler code example                     |
| Pre-programmed cycle              | `ros2 run my_robotic_arm control_arm.py`                        | Yes       | Loops through poses                      |
| Direct topic publish              | `ros2 topic pub /joint_states …`                                | No        | Full manual control (example below)      |
| Python API (write your own)       | Import `rclpy`, publish `JointState`                            | Yes       | Integrate into larger projects           |

Example topic publish:

ros2 topic pub /joint_states sensor_msgs/msg/JointState "
header: {stamp: {sec: 0, nanosec: 0}}
name: ['joint_1','joint_2','joint_3']
position: [1.57, 0.50, -0.50]
velocity: [0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0]" --once

text

---

## 4 Screenshots

| RViz (sliders)                                   | Gazebo (physics)                                  |
|--------------------------------------------------|---------------------------------------------------|
| ![RViz](screenshots/rviz_screenshot.png)         | ![Gazebo](screenshots/gazebo_simulation.png)      |

---

## 5 Control-Method Comparison

![Comparison Chart](screenshots/robotic_arm_control_methods.png)

| Method                    | Ease of Use (1-5) | Real-time Response (1-5) |
|---------------------------|-------------------|--------------------------|
| Joint sliders             | 5                 | 5                        |
| Keyboard instant          | 4                 | 4                        |
| Keyboard Enter            | 3                 | 3                        |
| Topic commands            | 2                 | 1                        |
| Custom scripts            | 3                 | 4                        |
| Python API                | 2                 | 5                        |
| Joystick                  | 1                 | 5                        |

---

## 6 File Catalogue

| File / Folder                       | Purpose |
|------------------------------------|---------|
| `launch/display.launch.py`         | RViz + sliders |
| `launch/gazebo_sim.launch.py`      | Gazebo + controllers |
| `urdf/my_robot_arm*.xacro`         | Robot description |
| `config/my_controllers.yaml`       | `ros2_control` parameters |
| `scripts/control_arm.py`           | Automated demo motion |
| `scripts/instant_keyboard_control.py` | Real-time keyboard teleop |
| `scripts/keyboard_control.py`      | Enter-key keyboard teleop |

---

## 7 Build & Run

build after cloning
colcon build --symlink-install
source install/setup.bash

launch RViz or Gazebo as required
text

---

## 8 Contributing & License

Pull requests are welcome; please follow ROS 2 coding conventions.  
Released under the Apache 2.0 license.
