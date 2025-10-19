#!/usr/bin/env python3
"""
State Monitor ROS2 Node for UAV LTL Planner

This node monitors the state of the mission execution and provides
status updates and logging capabilities.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class StateMonitorNode(Node):
    """
    ROS2 Node that monitors mission execution state and provides logging.
    """

    def __init__(self):
        super().__init__('state_monitor')
        
        # Initialize subscribers to monitor mission execution
        self.ltl_subscriber = self.create_subscription(
            String,
            '/ltl_formula',
            self.ltl_callback,
            10
        )
        
        self.status_subscriber = self.create_subscription(
            String,
            '/mission_status',
            self.status_callback,
            10
        )
        
        # Initialize publisher for monitoring data
        self.monitor_publisher = self.create_publisher(
            String,
            '/mission_monitor',
            10
        )
        
        # State tracking
        self.current_ltl = ""
        self.current_status = ""
        self.mission_count = 0
        self.start_time = time.time()
        
        # Create a timer for periodic status updates
        self.timer = self.create_timer(5.0, self.publish_monitor_status)
        
        self.get_logger().info("State Monitor Node initialized")

    def ltl_callback(self, msg):
        """
        Callback for LTL formula messages.
        
        Args:
            msg: std_msgs/String containing the LTL formula
        """
        self.current_ltl = msg.data
        self.get_logger().info(f"Received LTL formula: {self.current_ltl}")
        
        # Log the LTL formula for monitoring
        monitor_msg = String()
        monitor_msg.data = f"LTL_RECEIVED: {self.current_ltl}"
        self.monitor_publisher.publish(monitor_msg)

    def status_callback(self, msg):
        """
        Callback for mission status messages.
        
        Args:
            msg: std_msgs/String containing the mission status
        """
        self.current_status = msg.data
        self.get_logger().info(f"Mission status: {self.current_status}")
        
        # Increment mission count for successful translations
        if self.current_status in ["TRANSLATED", "EXECUTED"]:
            self.mission_count += 1
        
        # Log the status for monitoring
        monitor_msg = String()
        monitor_msg.data = f"STATUS_UPDATE: {self.current_status}"
        self.monitor_publisher.publish(monitor_msg)

    def publish_monitor_status(self):
        """Publish periodic monitoring status."""
        uptime = time.time() - self.start_time
        
        monitor_msg = String()
        monitor_msg.data = f"MONITOR_STATUS: uptime={uptime:.1f}s, missions={self.mission_count}, current_status={self.current_status}"
        self.monitor_publisher.publish(monitor_msg)
        
        self.get_logger().debug(f"Monitor status: {monitor_msg.data}")


def main(args=None):
    """Main function to start the ROS2 node."""
    rclpy.init(args=args)
    
    try:
        node = StateMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error starting state monitor node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
