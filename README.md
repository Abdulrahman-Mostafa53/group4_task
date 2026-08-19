## 🤖 Autonomous Maze Navigation — ROS 2

An autonomous maze navigation project built with **ROS 2**, **Gazebo Sim**, and **TurtleBot3**.

The project focuses on building a modular robot-control system using ROS 2 Actions, Services, Topics, and Odometry feedback to navigate a grid-based maze efficiently.

# 🚀 Overview

The robot navigates the simulated maze by combining:

- Custom **ROS 2 Actions** for linear and rotational movement.
- `/cmd_vel` for robot velocity control.
- `/odom` for position and orientation feedback.
- A **ROS 2 Service** for controlling maze walls.
- A high-level controller responsible for coordinating the robot's movements.

The movement system uses feedback from the robot's odometry to achieve accurate navigation.

# 🧩 Project Structure

The project is organized into three main ROS 2 packages:

- `maze_control` — Handles the Gazebo maze environment and wall-control functionality.
- `maze_interfaces` — Contains the custom ROS 2 interfaces used by the project, including `MoveX.action`, `MoveYaw.action`, and `Error.srv`.
- `maze_solver` — Contains the robot-control logic, including the movement Action Servers and high-level control components.

# ⚙️ Key Features

- TurtleBot3 simulation in Gazebo.
- Autonomous maze navigation.
- Custom ROS 2 movement Actions.
- Odometry-based movement tracking.
- Wall control through ROS 2 Services.
- Action cancellation and robot stopping.
- Modular ROS 2 package architecture.
- Git-based collaborative development.

# 🛠️ Technologies

- ROS 2
- Gazebo Sim Harmonic
- TurtleBot3
- Python
- rclpy
- ROS 2 Actions & Services
- Git & GitHub

# 📹 Demonstration

A demonstration of the final maze-navigation system will be provided here.

**[Demo Video](https://drive.google.com/drive/u/1/folders/1FKFmDAdRw6ilQXBaE2jsGNd-ZGbqqZ9M)**

# 👥 Team

This project was developed collaboratively as a team, with different members contributing to the ROS 2 packages, robot control, simulation, testing, and maze-navigation logic.
