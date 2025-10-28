# Deployment Notes

This document provides instructions for deploying the streamlined launch and control system to your ROS2 workspace.

## File Locations

The following files have been created and need to be placed in the correct locations:

### Files to Copy to Your Workspace

#### 1. Makefile
**Source:** `/Users/israelavendanojr./Desktop/project-repos/temporal-logic-agent/Makefile`  
**Destination:** `~/Desktop/crazyflie_ws/ros2_ws/Makefile`

This file should be placed at the root of your ROS2 workspace.

#### 2. CLI Script
**Already created at:** `src/uav_ltl_planner/scripts/mission_cli.py`  
**No action needed** - this file is already in the correct location and will be installed with the package when you run `colcon build`.

#### 3. Launch Script
**Already created at:** `src/uav_ltl_planner/scripts/launch.sh`  
**No action needed** - this file is already in the correct location and will be installed with the package when you run `colcon build`.

#### 4. README_QUICKSTART.md
**Already created at:** `README_QUICKSTART.md`  
This is in the project repository. You may want to copy it to your workspace root or reference it there.

## Quick Deployment Steps

1. **Copy the Makefile:**
```bash
cp ~/Desktop/project-repos/temporal-logic-agent/Makefile ~/Desktop/crazyflie_ws/ros2_ws/Makefile
```

2. **Update paths in launch.sh (if needed):**
The paths in `src/uav_ltl_planner/scripts/launch.sh` are already configured for:
- Workspace: `~/Desktop/crazyflie_ws/ros2_ws/`
- Virtual environment: `~/Desktop/crazyflie_ws/ros2_ws/venv/`
- ROS2 distro: `jazzy`

If your paths are different, edit the file before building.

3. **Build and install the package:**
```bash
cd ~/Desktop/crazyflie_ws/ros2_ws/
make build
```

4. **Test the system:**
```bash
make all
```

## Path Customization

If your workspace is at a different location, update these files:

### launch.sh
Edit the paths at the top of the file:
```bash
WORKSPACE_ROOT="$HOME/Desktop/crazyflie_ws/ros2_ws"  # Change this
VENV_PATH="$WORKSPACE_ROOT/venv"  # Change this if venv is elsewhere
CRAZYFLIE_SIM_PATH="$HOME/Desktop/crazyflie_ws/simulation_ws/crazyflie-simulation/simulator_files/gazebo"  # Change this
```

### Makefile
Edit the paths at the top of the file:
```makefile
WORKSPACE_ROOT := $(HOME)/Desktop/crazyflie_ws/ros2_ws  # Change this
VENV_PATH := $(WORKSPACE_ROOT)/venv  # Change this
ROS2_DISTRO := jazzy  # Change this if using a different distro
```

## Verification

After deploying, verify the setup:

1. **Check that scripts are executable:**
```bash
ls -l ~/Desktop/crazyflie_ws/ros2_ws/src/uav_ltl_planner/scripts/
```

You should see both scripts have execute permissions.

2. **Test the Makefile:**
```bash
cd ~/Desktop/crazyflie_ws/ros2_ws/
make help
```

You should see help text with available commands.

3. **Build the package:**
```bash
make build
```

This should build the package including the new CLI script.

## Troubleshooting

### "tmux: command not found"
Install tmux:
```bash
sudo apt install tmux
```

### "No such file or directory" errors
Check that all paths in `launch.sh` match your actual workspace structure.

### Makefile not found
Make sure you copied the Makefile to the workspace root (`~/Desktop/crazyflie_ws/ros2_ws/Makefile`), not inside the package.

### Scripts not found after build
Check that `setup.py` has been updated to include scripts. The scripts should be installed to:
```bash
ls ~/Desktop/crazyflie_ws/ros2_ws/install/uav_ltl_planner/share/uav_ltl_planner/scripts/
```

## Next Steps

Once deployed, follow the instructions in `README_QUICKSTART.md` to use the system.

