#!/usr/bin/env python3
"""
Interactive Mission Command Line Interface for UAV LTL Planner

This CLI allows users to send mission commands and receive real-time feedback
from the ROS2 mission system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import signal
import threading


class MissionCLI(Node):
    """
    Interactive CLI that publishes commands and subscribes to status updates.
    """
    
    def __init__(self):
        super().__init__('mission_cli')
        
        # Create publisher for mission commands
        self.command_publisher = self.create_publisher(
            String,
            '/mission_command',
            10
        )
        
        # Subscribe to LTL formulas
        self.ltl_subscription = self.create_subscription(
            String,
            '/ltl_formula',
            self.ltl_callback,
            10
        )
        
        # Subscribe to mission status
        self.status_subscription = self.create_subscription(
            String,
            '/mission_status',
            self.status_callback,
            10
        )
        
        self.get_logger().info("UAV Mission Control CLI initialized")
        
    def ltl_callback(self, msg):
        """Callback for LTL formula messages."""
        print(f"[LTL] {msg.data}")
        
    def status_callback(self, msg):
        """Callback for mission status messages."""
        print(f"[STATUS] {msg.data}")
        
    def publish_command(self, command):
        """Publish a mission command."""
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Published command: {command}")
        print(f"Mission sent: \"{command}\"")
    
    def start_spinning(self):
        """Start ROS2 spinning in a separate thread."""
        def spin_loop():
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        self.spin_thread = threading.Thread(target=spin_loop, daemon=True)
        self.spin_thread.start()


def main():
    """Main function for the CLI."""
    # Initialize ROS2
    rclpy.init()
    
    # Create CLI node
    cli = MissionCLI()
    cli.start_spinning()
    
    def signal_handler(sig, frame):
        """Handle Ctrl+C gracefully."""
        print("\n\nShutting down CLI...")
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Print welcome message
    print("\nUAV Mission Control CLI")
    print("Type 'exit' to quit.\n")
    
    try:
        # Main input loop - blocking input while ROS2 spins in background
        while rclpy.ok():
            try:
                user_input = input("UAV> ").strip()
                
                if user_input.lower() in ['exit', 'quit']:
                    print("\nExiting...")
                    break
                
                if user_input:
                    cli.publish_command(user_input)
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                signal_handler(None, None)
                break
            except Exception as e:
                print(f"Error: {e}")
                
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

