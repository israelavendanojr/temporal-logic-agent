#!/bin/bash
# Launch script for UAV LTL Planner using tmux
# This creates a 3-pane layout: Simulator | Nodes | CLI

set -e

# Define paths - update these for your system
WORKSPACE_ROOT="$HOME/Desktop/crazyflie_ws/ros2_ws"
VENV_PATH="$WORKSPACE_ROOT/venv"
CRAZYFLIE_SIM_PATH="$HOME/Desktop/crazyflie_ws/simulation_ws/crazyflie-simulation/simulator_files/gazebo"
ROS2_DISTRO="jazzy"

# Session name
SESSION_NAME="uav"

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "Error: tmux is not installed. Install with: sudo apt install tmux"
    exit 1
fi

# Kill existing session if it exists
tmux kill-session -t $SESSION_NAME 2>/dev/null || true

# Create new detached session
tmux new-session -d -s $SESSION_NAME

# Create panes: 3 horizontal panes
tmux split-window -h -t $SESSION_NAME
tmux split-window -h -t $SESSION_NAME

# Set up Pane 0 (Simulator)
tmux select-pane -t 0
tmux send-keys -t $SESSION_NAME "cd $WORKSPACE_ROOT" C-m
tmux send-keys -t $SESSION_NAME "source $VENV_PATH/bin/activate" C-m
tmux send-keys -t $SESSION_NAME "source /opt/ros/$ROS2_DISTRO/setup.bash" C-m
tmux send-keys -t $SESSION_NAME "source $WORKSPACE_ROOT/install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME "export CRAZYFLIE_SIM_PATH=$CRAZYFLIE_SIM_PATH" C-m
tmux send-keys -t $SESSION_NAME "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:\$CRAZYFLIE_SIM_PATH" C-m
tmux send-keys -t $SESSION_NAME "ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py" C-m

# Set up Pane 1 (LTL Nodes)
tmux select-pane -t 1
tmux send-keys -t $SESSION_NAME "cd $WORKSPACE_ROOT" C-m
tmux send-keys -t $SESSION_NAME "source $VENV_PATH/bin/activate" C-m
tmux send-keys -t $SESSION_NAME "source /opt/ros/$ROS2_DISTRO/setup.bash" C-m
tmux send-keys -t $SESSION_NAME "source $WORKSPACE_ROOT/install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME "ros2 launch uav_ltl_planner ltl_mission.launch.py" C-m

# Set up Pane 2 (CLI)
tmux select-pane -t 2
tmux send-keys -t $SESSION_NAME "cd $WORKSPACE_ROOT" C-m
tmux send-keys -t $SESSION_NAME "source $VENV_PATH/bin/activate" C-m
tmux send-keys -t $SESSION_NAME "source /opt/ros/$ROS2_DISTRO/setup.bash" C-m
tmux send-keys -t $SESSION_NAME "source $WORKSPACE_ROOT/install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME "sleep 2" C-m
tmux send-keys -t $SESSION_NAME "python3 $WORKSPACE_ROOT/src/uav_ltl_planner/scripts/mission_cli.py" C-m

# Wait a moment for everything to start, then attach to CLI pane
sleep 1

# Attach to the session, selecting the CLI pane
tmux attach-session -t $SESSION_NAME

echo "Launched UAV LTL Planner in tmux session '$SESSION_NAME'"
echo "Layout: [Simulator | Nodes | CLI]"
echo ""
echo "Keyboard shortcuts:"
echo "  Ctrl+b then n  - Next pane"
echo "  Ctrl+b then p  - Previous pane"
echo "  Ctrl+b then q  - Quit session"
echo "  Type 'exit' in CLI to quit"

