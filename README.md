UAV LTL Planner - ROS2 Package

This document provides setup and usage instructions for the UAV LTL Planner ROS2 package, which converts natural language mission commands into Linear Temporal Logic (LTL) formulas and executes them using a Gazebo-based drone simulator.

Overview

The UAV LTL Planner is a ROS2 package that provides:

    Natural language to LTL translation using a fine-tuned GGUF language model

    LTL formula validation and feasibility checking

    Drone execution via the ros_gz_crazyflie simulation stack

    ROS2 topic-based communication for mission commands and status

Prerequisites

System Requirements

    Ubuntu 22.04 (recommended)

    Python 3.10+

    ROS2 Jazzy (or compatible version)

Dependencies

    rclpy - ROS2 Python client library

    std_msgs, geometry_msgs, nav_msgs - ROS2 message types

    langchain-core, langgraph - AI/ML libraries

    llama-cpp-python - GGUF model inference

    PyYAML - Configuration file parsing

    ros_gz_crazyflie - Gazebo simulation packages

Usage

These instructions assume your workspace is located at ~/Desktop/crazyflie_ws/ros2_ws/.

Terminal 1: Launch the Simulator

First, launch the Crazyflie Gazebo simulation.
Bash

# Set up the environment
cd ~/Desktop/crazyflie_ws/ros2_ws/
source ~/Desktop/crazyflie_ws/ros2_ws/venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Set Gazebo simulation paths
export CRAZYFLIE_SIM_PATH=~/Desktop/crazyflie_ws/simulation_ws/crazyflie-simulation/simulator_files/gazebo
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$CRAZYFLIE_SIM_PATH

# Launch the simulator
ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py

Terminal 2: Launch the LTL Planner

In a new terminal, launch the uav_ltl_planner nodes. This will start the mission_executor and state_monitor.
Bash

# Set up the environment
cd ~/Desktop/crazyflie_ws/ros2_ws/
source ~/Desktop/crazyflie_ws/ros2_ws/venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch the LTL nodes
ros2 launch uav_ltl_planner ltl_mission.launch.py

Terminal 3: Send a Command

In a third terminal, you can publish a natural language command to the /mission_command topic.
Bash

# Set up the environment
cd ~/Desktop/crazyflie_ws/ros2_ws/
source ~/Desktop/crazyflie_ws/ros2_ws/venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Send the mission command
echo "--- SENDING COMMAND ---"
ros2 topic pub --once /mission_command std_msgs/String "{data: 'go to waypoint_a'}"

Terminal 4: Monitor Results

In a separate terminal (or in Terminal 3), you can monitor the output from the LTL planner.
Bash

# Monitor LTL formula output
ros2 topic echo /ltl_formula

# Monitor mission status
ros2 topic echo /mission_status

# Monitor system status
ros2 topic echo /mission_monitor

ROS2 Topics

Subscribed Topics

    /mission_command (std_msgs/String) - Natural language mission commands

    /crazyflie/odom (nav_msgs/Odometry) - Used by the CrazyflieExecutor to get the drone's current position.

Published Topics

    /ltl_formula (std_msgs/String) - Generated LTL formulas or final plan status.

    /mission_status (std_msgs/String) - High-level mission execution status.

    /mission_monitor (std_msgs/String) - System monitoring information.

    /crazyflie/cmd_vel (geometry_msgs/Twist) - Velocity commands sent to the simulated drone.

Status Values

    TRANSLATED - LTL formula generated successfully

    EXECUTED - Mission plan generated and sent to executor

    NOT_FEASIBLE - Mission not feasible with current constraints

    ERROR - Error occurred during processing