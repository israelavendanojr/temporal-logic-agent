# Makefile for UAV LTL Planner ROS2 Package
# Workspace: ~/Desktop/crazyflie_ws/ros2_ws/

WORKSPACE_ROOT := $(HOME)/Desktop/crazyflie_ws/ros2_ws
VENV_PATH := $(WORKSPACE_ROOT)/venv
ROS2_DISTRO := jazzy

.PHONY: build clean launch stop all help

help:
	@echo "UAV LTL Planner - Makefile Targets"
	@echo ""
	@echo "  make build    - Build the uav_ltl_planner package"
	@echo "  make clean    - Remove build artifacts"
	@echo "  make launch   - Launch the UAV system (simulator + nodes + CLI)"
	@echo "  make stop     - Stop the tmux session"
	@echo "  make all      - Build and launch the system"
	@echo ""

build:
	@echo "Building uav_ltl_planner package..."
	cd $(WORKSPACE_ROOT) && \
	source $(VENV_PATH)/bin/activate && \
	source /opt/ros/$(ROS2_DISTRO)/setup.bash && \
	colcon build --packages-select uav_ltl_planner
	@echo "Build complete"

clean:
	@echo "Cleaning build artifacts..."
	cd $(WORKSPACE_ROOT) && \
	rm -rf build/ install/ log/ && \
	find . -type d -name __pycache__ -exec rm -r {} + 2>/dev/null || true && \
	find . -type f -name "*.pyc" -delete
	@echo "Clean complete"

launch:
	@echo "Launching UAV LTL Planner system..."
	$(WORKSPACE_ROOT)/src/uav_ltl_planner/scripts/launch.sh

stop:
	@echo "Stopping tmux session..."
	tmux kill-session -t uav 2>/dev/null || echo "No session found"
	@echo "Stopped"

all: build launch
	@echo "Build and launch complete"

