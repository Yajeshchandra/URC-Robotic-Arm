# Robotic Arm Control Repository

## Overview

This repository contains the essential packages for simulating and controlling a robotic arm for **Team Deimos Mars Rover 2024** . The setup includes:

- **URDF Launch Package**: For launching the robot model in a simulation environment.
- **MoveIt Package**: For planning and executing motions of the robotic arm.
- **Teleoperation (Teleop) Package**: For controlling the robotic arm using key bindings.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
  - [1. Launch the Robotic Arm in Gazebo](#1-launch-the-robotic-arm-in-gazebo)
  - [2. Simulate Motion Planning with MoveIt](#2-simulate-motion-planning-with-moveit)
  - [3. Teleoperate the Robotic Arm](#3-teleoperate-the-robotic-arm)
- [File Structure](#file-structure)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

## Prerequisites

Before you begin, ensure you have met the following requirements:

- **ROS Noetic** installed on Ubuntu 20.04.
- **Gazebo** installed for simulation.
- **MoveIt** for motion planning.
- Basic understanding of ROS packages and launch files.

## Installation

1. Clone the repository:

    ```bash
    git clone https://github.com/rishang19dx/URC-Robotic-Arm.git
    cd URC-Robotic-Arm
    ```

2. Install necessary dependencies:

    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build the workspace:

    ```bash
    catkin build
    source devel/setup.bash
    ```

## Usage

### 1. Launch the Robotic Arm in Gazebo

To launch the robotic arm in the Gazebo simulation:

```bash
roslaunch arm_urdf arm_urdf.launch
```

### 2. Simulate Motion Planning with MoveIt

To plan and execute motions using MoveIt:

```bash
roslaunch arm_moveit full_robot_arm_sim.launch
```

### 3. Teleoperate the Robotic Arm

To control the robotic arm using key bindings:

```bash
rosrun arm_teleop teleop.py
```

  ![image](https://github.com/user-attachments/assets/58a98093-fb87-4899-b2ab-bd99611cd9a8)


## File Structure

<div style="display: flex; align-items: flex-start;">

  <!-- Text on the right -->
  <pre>
  URC-Robotic-Arm/
  ├── arm_urdf/
  │   ├── urdf/
  │   │   └── arm_urdf.urdf                     # URDF model of the robotic arm
  │   ├── launch/
  │   │   └── arm_urdf.launch                   # Launch file for Gazebo simulation
  ├── arm_moveit/
  │   ├── config/
  │   │   └── ...                               # MoveIt configuration files
  │   ├── launch/
  │   │   └── full_robot_arm_sim.launch         # Launch file for MoveIt
  ├── arm_teleop/
  │   ├── src/
  │   │   └── teleop.py                         # Script for key binding control
  └── README.md                                 # You are here!
  </pre>

</div>


## Troubleshooting

If you encounter issues, consider the following:

- Ensure all dependencies are installed and sourced correctly.
- Check the URDF file for any syntax errors if the robot doesn't load in Gazebo.
- Verify that the correct ROS nodes are running if MoveIt or teleoperation doesn't work.

## Contributing

Contributions are welcome! If you have suggestions, find a bug, or want to add a feature, feel free to create an issue or submit a pull request.

1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Make your changes and commit them (`git commit -am 'Add some feature'`).
4. Push to the branch (`git push origin feature-branch`).
5. Create a pull request.

---
