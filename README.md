# Temporal Logic Agent

A ROS2-based system for translating natural language drone mission commands into Linear Temporal Logic (LTL) formulas and executing them in a Gazebo simulation environment. This project combines fine-tuned large language models with LTL planning to enable intuitive high-level control of autonomous unmanned aerial vehicles.

## Quick Start

Build and launch the complete system:

```
make build
make launch
make cli
```

Monitor system status:

```
make status
```

Stop all services:

```
make stop
```

## Architecture

```
Natural Language Command
         |
         v
  Mission CLI Node
         |
         v
   /mission_command (ROS2 Topic)
         |
         v
  Mission Executor Node
         |
         v
  LangGraph Agent Core
         |
         v
  [GGUF Translation Model] --> LTL Formula
         |                            |
         |                            v
         |                   LTL Parser & Validator
         |                            |
         |                            v
         |                   Feasibility Checker
         |                            |
         |                            v
         |                   LTL Executor
         |                            |
         +--------------------------> Plan Generator
                                         |
                                         v
                              /crazyflie/cmd_vel (ROS2 Topic)
                                         |
                                         v
                              Gazebo Simulator (Crazyflie Drone)
```

## Project Structure

```
temporal-logic-agent/
├── Makefile                          # Build and launch orchestration
├── requirements.txt                  # Python dependencies
├── models/
│   ├── lifted_data.jsonl            # Training dataset
│   ├── ltl-notebook.ipynb          # Model training notebook
│   ├── pack_dataset.py             # Dataset preprocessing utilities
│   └── run_qlora.py                # QLoRA fine-tuning script
├── scripts/
│   └── mission_cli.py              # Interactive mission control CLI
└── src/
    └── uav_ltl_planner/
        ├── config/
        │   └── environment.yaml
        ├── launch/
        │   └── ltl_mission.launch.py
        ├── package.xml
        ├── setup.cfg
        ├── setup.py
        ├── scripts/
        │   ├── launch.sh
        │   └── mission_cli.py
        └── uav_ltl_planner/
            ├── __init__.py
            ├── agent/
            │   ├── __init__.py
            │   ├── config_loader.py
            │   ├── core.py               # LangGraph agent workflow
            │   ├── model_server.py       # GGUF model inference
            │   ├── tools.py              # Agent tool functions
            │   └── ltl/
            │       ├── __init__.py
            │       ├── exceptions.py
            │       ├── grammar.py        # LTL grammar definition
            │       └── parser.py         # LTL syntax validator
            ├── ros2_nodes/
            │   ├── __init__.py
            │   ├── mission_executor_node.py  # Main ROS2 node
            │   └── state_monitor_node.py     # System state monitor
            └── services/
                ├── __init__.py
                ├── crazyflie_executor.py     # Crazyflie integration
                └── ltl_executor.py           # LTL plan execution
```
