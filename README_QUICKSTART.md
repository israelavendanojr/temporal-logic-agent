# Quickstart Guide - UAV LTL Planner

This guide provides the simplest way to build, launch, and control the UAV LTL Planner system.

## One-Time Setup

### Prerequisites

Install `tmux` if not already installed:

```bash
sudo apt install tmux
```

### Initial Build

Navigate to your workspace and build the package:

```bash
cd ~/Desktop/crazyflie_ws/ros2_ws/
make build
```

Or use the Makefile directly:

```bash
cd ~/Desktop/crazyflie_ws/ros2_ws/
source venv/bin/activate
source /opt/ros/jazzy/setup.bash
colcon build --packages-select uav_ltl_planner
```

## Daily Workflow

### Launch Everything with One Command

```bash
cd ~/Desktop/crazyflie_ws/ros2_ws/
make all
```

This will:
1. Build the package
2. Launch the simulator in one tmux pane
3. Launch the LTL planner nodes in another pane
4. Open an interactive CLI in the third pane

You'll be dropped into the interactive CLI where you can send commands directly.

### Using the CLI

Once launched, you'll see the CLI prompt:

```
UAV Mission Control CLI
Type 'exit' to quit.

UAV>
```

Simply type your natural language commands:

```
UAV> go to waypoint_a
Mission sent: "go to waypoint_a"
[LTL] F(at(waypoint_a))
[STATUS] TRANSLATED
[STATUS] EXECUTED

UAV> exit
Exiting...
```

### Switching Between Panes

While in the tmux session, use these keyboard shortcuts:

- `Ctrl+b` then `n` - Cycle to next pane (Simulator -> Nodes -> CLI)
- `Ctrl+b` then `p` - Cycle to previous pane
- `Ctrl+b` then `q` - Quit the tmux session

You can watch the output from the simulator and nodes by switching panes.

## Shutdown

### Exit the CLI

Simply type `exit` in the CLI:

```
UAV> exit
Exiting...
```

This will close the CLI but keep the simulator and nodes running.

### Stop Everything

To completely stop the system, exit the CLI (type `exit`), then from any terminal:

```bash
make stop
```

Or:

```bash
cd ~/Desktop/crazyflie_ws/ros2_ws/
tmux kill-session -t uav
```

## Makefile Commands

For reference, here are all available Makefile targets:

| Command | Description |
|---------|-------------|
| `make build` | Build the uav_ltl_planner package |
| `make clean` | Remove build artifacts |
| `make launch` | Launch simulator + nodes + CLI |
| `make stop` | Stop the tmux session |
| `make all` | Build and launch (recommended) |

## Example Commands

Try these commands in the CLI:

```
UAV> go to waypoint_a
UAV> patrol between waypoint_a and waypoint_b
UAV> avoid obstacle_1 and go to waypoint_c
UAV> return to starting position
```

The system will automatically:
1. Translate your natural language to LTL formulas
2. Validate the mission
3. Execute the mission in the Gazebo simulation
4. Display real-time feedback in your CLI

## Troubleshooting

### CLI Not Showing Messages

If you don't see LTL or status messages, check that the nodes are running:

1. Switch to the "Nodes" pane using `Ctrl+b` then `n`
2. Look for error messages
3. Ensure the model is loaded (may take 30 seconds on first launch)

### Simulator Not Starting

Check the "Sim" pane:
1. Switch to the first pane using `Ctrl+b` then `p` (twice to go to first pane)
2. Look for error messages
3. Verify Gazebo is properly installed and sourced

### Build Errors

```bash
make clean
make build
```

## Advanced Usage

### Manual Control (3 Terminals)

If you prefer manual control over the automated system:

**Terminal 1 - Simulator:**
```bash
cd ~/Desktop/crazyflie_ws/ros2_ws/
source venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export CRAZYFLIE_SIM_PATH=~/Desktop/crazyflie_ws/simulation_ws/crazyflie-simulation/simulator_files/gazebo
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$CRAZYFLIE_SIM_PATH
ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py
```

**Terminal 2 - Nodes:**
```bash
cd ~/Desktop/crazyflie_ws/ros2_ws/
source venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch uav_ltl_planner ltl_mission.launch.py
```

**Terminal 3 - CLI:**
```bash
cd ~/Desktop/crazyflie_ws/ros2_ws/
source venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/uav_ltl_planner/scripts/mission_cli.py
```

## ROS2 Topics

The system uses the following ROS2 topics:

- **Subscribed:**
  - `/mission_command` (std_msgs/String) - Mission commands from CLI

- **Published:**
  - `/ltl_formula` (std_msgs/String) - Generated LTL formulas
  - `/mission_status` (std_msgs/String) - Mission execution status
  - `/crazyflie/cmd_vel` (geometry_msgs/Twist) - Velocity commands to drone
  - `/crazyflie/odom` (nav_msgs/Odometry) - Drone position feedback

## Status Values

- `TRANSLATED` - LTL formula generated successfully
- `EXECUTED` - Mission plan executed
- `NOT_FEASIBLE` - Mission not feasible with current constraints  
- `ERROR` - Error occurred during processing

## Support

For detailed information, see [README.md](README.md).

