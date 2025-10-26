#!/usr/bin/env python3
"""
Mission Executor ROS2 Node for UAV LTL Planner

This node subscribes to natural language mission commands and publishes
LTL formulas and execution status through ROS2 topics.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from langchain_core.messages import HumanMessage
import os
import sys
import logging
import time

# Add the package path to sys.path for imports
try:
    from ament_index_python.packages import get_package_share_directory
    package_share_dir = get_package_share_directory('uav_ltl_planner')
    sys.path.insert(0, os.path.join(package_share_dir, '..', '..', 'src', 'uav_ltl_planner'))
except ImportError:
    # Fallback for standalone mode
    package_share_dir = None

# Import the existing agent components
from uav_ltl_planner.agent.core import get_compiled_graph
from uav_ltl_planner.agent.config_loader import get_config
from uav_ltl_planner.services.ltl_executor import LTLExecutor


class MissionExecutorNode(Node):
    """
    ROS2 Node that handles mission command processing and LTL translation.
    """

    def __init__(self):
        super().__init__('mission_executor')
        
        # Initialize session state
        self.session_state = None
        self.app = None
        
        # Initialize publishers
        self.ltl_publisher = self.create_publisher(
            String,
            '/ltl_formula',
            10
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/mission_status',
            10
        )
        
        # Initialize subscriber
        self.mission_subscriber = self.create_subscription(
            String,
            '/mission_command',
            self.mission_callback,
            10
        )
        
        # Initialize LTL executor
        self.ltl_executor = LTLExecutor()

        try:
            from uav_ltl_planner.services.crazyflie_executor import CrazyflieExecutor
            self.crazyflie_executor = CrazyflieExecutor(self)
        except Exception as e:
            self.get_logger().warn(f"Crazyflie executor unavailable: {e}")
            self.crazyflie_executor = None
        
        # Compile the agent graph
        try:
            self.get_logger().info("Compiling agent graph...")
            self.app = get_compiled_graph()
            self.get_logger().info("Agent graph compiled successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to compile agent graph: {e}")
            raise
        
        # Initialize session state
        self._initialize_session_state()

        try:
            self.get_logger().info("Pre-loading LTL translation model...")
            from uav_ltl_planner.agent.model_server import get_gguf_model
            get_gguf_model()  # This will load and cache the model
            self.get_logger().info("✓ Model pre-loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to pre-load model: {e}")
        
        self.get_logger().info("Mission Executor Node initialized and ready")

    def _initialize_session_state(self):
        """Initialize the session state with configuration."""
        try:
            config = get_config()
            drone_state = config.get_drone_state()
            self.session_state = {
                'conversation_log': [],
                'spatial_memory': {
                    'current_position': tuple(drone_state['start_position']),
                    'start_position': tuple(drone_state['start_position']),
                    'objects': config.get_waypoints(),
                    'flight_zone': config.get_flight_zone(),
                    'obstacles': config.get_obstacles()
                }
            }
            self.get_logger().info("Session state initialized")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize session state: {e}")
            raise

    def mission_callback(self, msg):
        """
        Callback function for mission command messages.
        
        Args:
            msg: std_msgs/String containing the natural language mission command
        """
        query = msg.data.strip()
        if not query:
            self.get_logger().warn("Received empty mission command")
            return
        
        # Wait for model to be ready (up to 30 seconds)
        if not hasattr(self, '_model_ready_logged'):
            self.get_logger().info("Waiting for model to be ready...")
            timeout = 30
            start = time.time()
            while time.time() - start < timeout:
                try:
                    from uav_ltl_planner.agent.model_server import get_gguf_model
                    model = get_gguf_model()
                    if model.model is not None:
                        self.get_logger().info("✓ Model is ready")
                        self._model_ready_logged = True
                        break
                except:
                    pass
                time.sleep(0.5)

        self.get_logger().info(f"Received mission command: {query}")
        
        try:
            # Process the mission command through the agent - NOW RETURNS 3 VALUES
            result, self.session_state, ltl_formula = self._run_agent(query, self.session_state)
            
            # Publish the LTL formula (or execution result)
            ltl_msg = String()
            ltl_msg.data = result
            self.ltl_publisher.publish(ltl_msg)
            self.get_logger().info(f"Published LTL formula: {result}")
            
            # Publish status
            status_msg = String()
            if result.startswith("ERROR:") or result.startswith("INVALID_"):
                status_msg.data = "ERROR"
            elif "NOT FEASIBLE" in result:
                status_msg.data = "NOT_FEASIBLE"
            elif "Execution complete" in result:
                status_msg.data = "EXECUTED"
            else:
                status_msg.data = "TRANSLATED"
            
            self.status_publisher.publish(status_msg)
            self.get_logger().info(f"Published status: {status_msg.data}")

            # Execute on Crazyflie using captured LTL formula!
            if self.crazyflie_executor and ltl_formula:
                self.get_logger().info(f"Sending to Crazyflie: {ltl_formula}")
                plan = self.ltl_executor.parse_formula(ltl_formula)
                self.crazyflie_executor.run(plan)
            
        except Exception as e:
            self.get_logger().error(f"Error processing mission command: {e}")
            
            # Publish error status
            error_msg = String()
            error_msg.data = f"ERROR: {str(e)}"
            self.ltl_publisher.publish(error_msg)
            
            status_msg = String()
            status_msg.data = "ERROR"
            self.status_publisher.publish(status_msg)

    def _run_agent(self, query: str, session_state: dict = None):
        """
        Run the agent with session state tracking.
        
        Args:
            query: Natural language query
            session_state: Current session state
            
        Returns:
            Tuple of (result, updated_session_state, ltl_formula_if_available)
        """
        if self.app is None:
            return "ERROR: Agent graph not compiled.", session_state, None

        # Initialize session state on first run
        if session_state is None:
            config = get_config()
            drone_state = config.get_drone_state()
            session_state = {
                'conversation_log': [],
                'spatial_memory': {
                    'current_position': tuple(drone_state['start_position']),
                    'start_position': tuple(drone_state['start_position']),
                    'objects': config.get_waypoints(),
                    'flight_zone': config.get_flight_zone(),
                    'obstacles': config.get_obstacles()
                }
            }
        
        # Prepare inputs for LangGraph
        inputs = {
            "messages": [HumanMessage(content=query)],
            "conversation_log": session_state['conversation_log'],
            "spatial_memory": session_state['spatial_memory']
        }
        
        # Execute LangGraph workflow and capture LTL formula
        final_answer = ""
        final_state = None
        captured_ltl_formula = None
        
        for output in self.app.stream(inputs):
            for key, value in output.items():
                messages = value.get("messages", [])
                if messages and hasattr(messages[-1], 'content'):
                    final_answer = messages[-1].content
                    
                # Capture the LTL formula from the state before execution
                if 'ltl_formula' in value and value['ltl_formula']:
                    ltl = value['ltl_formula']
                    # Only capture if it's a valid LTL formula (not error messages)
                    if ltl.startswith('F(') or ltl.startswith('G('):
                        captured_ltl_formula = ltl
                        
                final_state = value
        
        # Update session state from results
        if final_state and 'conversation_log' in final_state:
            session_state['conversation_log'] = final_state['conversation_log']
            session_state['spatial_memory'] = final_state['spatial_memory']
        
        return final_answer, session_state, captured_ltl_formula


def main(args=None):
    """Main function to start the ROS2 node."""
    rclpy.init(args=args)
    
    try:
        node = MissionExecutorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error starting mission executor node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
