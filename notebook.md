# ROS2 Personal Notes Template

## Table of Contents
- [Overview](#overview)
- [Installation](#installation)
- [Workspace Setup](#workspace-setup)
- [Core Concepts](#core-concepts)
- [Common Commands](#common-commands)
- [Useful Resources](#useful-resources)

---

## Overview
_Short summary of ROS2 and its purpose._

---

## Installation
- OS requirements:
- ROS2 distribution:
- Key commands:
    ```bash
    # Example:
    sudo apt update
    sudo apt install ros-<distro>-desktop
    ```

---

## Workspace Setup
- Create workspace:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
- Notes:

---

## Core Concepts
- **Nodes:**  
- **Topics:**  
- **Services:**  
- **Actions:**  
- **Parameters:**  
- **Launch files:**  

---

## Common Commands
| Task                | Command Example                      |
|---------------------|--------------------------------------|
| Build workspace     | `colcon build`                       |
| Source environment  | `source install/setup.bash`          |
| Run node            | `ros2 run <package> <executable>`    |
| List topics         | `ros2 topic list`                    |
| Echo topic          | `ros2 topic echo <topic_name>`       |

---

## Useful Resources
- [ROS2 Documentation](https://docs.ros.org/en/rolling/index.html)
- [ROS2 Tutorials](https://docs.ros.org/en/rolling/Tutorials.html)
- [ROS Answers](https://answers.ros.org/questions/)

---

## Personal Notes
- _Add your own notes, issues, and solutions here._