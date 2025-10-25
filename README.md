# UAV LTL Planner - ROS2 Package

This document provides setup and usage instructions for the UAV LTL Planner ROS2 package, which converts natural language mission commands into Linear Temporal Logic (LTL) formulas and executes them using a mock drone simulator.

## Overview

The UAV LTL Planner is a ROS2 package that provides:
- Natural language to LTL translation using a fine-tuned language model
- LTL formula validation and feasibility checking
- Mock drone execution simulation
- ROS2 topic-based communication for mission commands and status

## Prerequisites

### System Requirements
- Ubuntu 22.04 (recommended) or macOS
- Python 3.10+
- ROS2 Humble (or compatible version)

### Dependencies
- `rclpy` - ROS2 Python client library
- `std_msgs`, `geometry_msgs`, `nav_msgs`, `trajectory_msgs` - ROS2 message types
- `langchain-core`, `langchain-ollama`, `langgraph` - AI/ML libraries
- `llama-cpp-python` - GGUF model inference
- `PyYAML` - Configuration file parsing

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd temporal-logic-agent
```

### 2. Set Up Python Environment
```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install Python dependencies
pip install -r requirements.txt
```

### 3. Set Up ROS2 Workspace
```bash
# Install ROS2 dependencies (if not already installed)
sudo apt update
sudo apt install ros-humble-rclpy ros-humble-std-msgs ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-trajectory-msgs

# Build the package
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 4. Download the Model (Optional)
The package includes a placeholder for the GGUF model. For full functionality, you'll need to:
1. Obtain the `translation_model.gguf` file
2. Place it in `src/uav_ltl_planner/models/`
3. The model will be automatically copied to the install directory during build

## Usage

### 1. Launch the System
```bash
# Source the workspace
source install/setup.bash

# Launch the mission execution system
ros2 launch uav_ltl_planner ltl_mission.launch.py
```

This will start:
- `mission_executor` node - Handles mission command processing
- `state_monitor` node - Monitors execution status

### 2. Send Mission Commands
```bash
# Send a natural language mission command
ros2 topic pub /mission_command std_msgs/String "data: 'fly to waypoint_a'"

# Send another command
ros2 topic pub /mission_command std_msgs/String "data: 'go to waypoint_b then waypoint_c'"

# Send a constraint-based command
ros2 topic pub /mission_command std_msgs/String "data: 'stay above 2 meters while going to waypoint_a'"
```

### 3. Monitor Results
```bash
# Monitor LTL formula output
ros2 topic echo /ltl_formula

# Monitor mission status
ros2 topic echo /mission_status

# Monitor system status
ros2 topic echo /mission_monitor
```

### 4. Example Commands
The system supports various types of mission commands:

**Basic Navigation:**
- `"fly to waypoint_a"`
- `"go to waypoint_b"`
- `"navigate to waypoint_c"`

**Sequential Missions:**
- `"go to waypoint_a then waypoint_b"`
- `"fly to waypoint_a then waypoint_b then waypoint_c"`

**Constraints:**
- `"stay above 2 meters"`
- `"maintain altitude above 1.5 meters"`
- `"go to waypoint_a while avoiding obstacle_1"`

**Actions:**
- `"hover for 10 seconds"`
- `"scan area_1"`

## ROS2 Topics

### Subscribed Topics
- `/mission_command` (std_msgs/String) - Natural language mission commands

### Published Topics
- `/ltl_formula` (std_msgs/String) - Generated LTL formulas
- `/mission_status` (std_msgs/String) - Mission execution status
- `/mission_monitor` (std_msgs/String) - System monitoring information

### Status Values
- `TRANSLATED` - LTL formula generated successfully
- `EXECUTED` - Mission executed successfully
- `NOT_FEASIBLE` - Mission not feasible with current constraints
- `ERROR` - Error occurred during processing

## Configuration

### Environment Configuration
The system uses `src/uav_ltl_planner/config/environment.yaml` for configuration:

```yaml
flight_zone:
  min: [-5.0, -5.0, 0.3]
  max: [5.0, 5.0, 3.0]

waypoints:
  waypoint_a: [2.0, 2.0, 1.0]
  waypoint_b: [-2.0, 2.0, 1.0]
  waypoint_c: [0.0, 4.0, 2.0]

obstacles:
  obstacle_1:
    center: [1.0, 1.0, 0.0]
    radius: 0.5

safety:
  min_altitude: 0.3
  max_altitude: 3.0
  emergency_battery_level: 20

drone:
  start_position: [0.0, 0.0, 0.0]
  max_speed: 2.0
  battery_capacity: 100
```

### Launch Parameters
```bash
# Launch with custom config file
ros2 launch uav_ltl_planner ltl_mission.launch.py config_file:=/path/to/custom/config.yaml

# Launch with debug logging
ros2 launch uav_ltl_planner ltl_mission.launch.py log_level:=debug
```

## Development

### Running Individual Nodes
```bash
# Run mission executor only
ros2 run uav_ltl_planner mission_executor

# Run state monitor only
ros2 run uav_ltl_planner state_monitor
```

### Testing
```bash
# Test the ROS2 package structure
python -c "import sys; sys.path.insert(0, 'src'); from uav_ltl_planner.ros2_nodes.mission_executor_node import MissionExecutorNode; print('Package structure is correct')"
```

### Building
```bash
colcon build --symlink-install --packages-select uav_ltl_planner
```

## Troubleshooting

### Common Issues

1. **Model not found error:**
   - Ensure `translation_model.gguf` is in the models directory
   - Check that the model file is not gitignored

2. **Import errors:**
   - Verify all Python dependencies are installed
   - Check that the ROS2 workspace is properly sourced

3. **Config file not found:**
   - Ensure `config/environment.yaml` exists
   - Check file permissions

4. **Node startup failures:**
   - Check ROS2 installation
   - Verify package dependencies are met
   - Check logs with `ros2 run uav_ltl_planner mission_executor --ros-args --log-level debug`

### Debug Mode
```bash
# Launch with debug logging
ros2 launch uav_ltl_planner ltl_mission.launch.py log_level:=debug

# Check node logs
ros2 node list
ros2 node info /mission_executor
```

## Integration with Real UAV Systems

This package is designed to be easily integrated with real UAV systems:

1. **Replace Mock Executor:** The `MockExecutor` class can be replaced with real UAV control interfaces
2. **Add Real Sensors:** Integrate with actual sensor data for obstacle detection and positioning
3. **Connect to Flight Controllers:** Interface with PX4, ArduPilot, or other flight control systems
4. **Add Safety Systems:** Implement real-time safety monitoring and emergency procedures

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test with both standalone and ROS2 modes
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review the logs and error messages
3. Open an issue on the repository
4. Contact the development team
