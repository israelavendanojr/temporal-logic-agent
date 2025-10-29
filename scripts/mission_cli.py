#!/usr/bin/env python3
"""
UAV LTL Planner - Interactive Mission CLI
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import time
from nav_msgs.msg import Odometry

# --- ANSI Color Codes (Reduced to essential debug colors) ---
class bcolors:
    # Essential for Debugging
    OKGREEN = '\033[92m'  # Success / Command Sent
    OKBLUE = '\033[94m'   # Info / LTL Formula
    FAIL = '\033[91m'     # Error / Not Feasible
    WARNING = '\033[93m'  # Exiting / Warnings
    
    # Utility
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    
# --- The CLI Node ---
class MissionCLI(Node):
    """
    An all-in-one ROS2 node for sending mission commands and
    receiving real-time status feedback.
    """
    def __init__(self):
        super().__init__('mission_cli_node')
        
        # Publisher for sending commands
        self.command_publisher = self.create_publisher(
            String,
            '/mission_command',
            10
        )
        
        # Subscribers for feedback
        self.status_subscriber = self.create_subscription(
            String,
            '/mission_status',
            self.status_callback,
            10
        )
        
        self.ltl_subscriber = self.create_subscription(
            String,
            '/ltl_formula',
            self.ltl_callback,
            10
        )
        
        # NEW: Subscribe to odometry for live position
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/crazyflie/odom',
            self.odom_callback,
            10
        )
        
        self.current_position = None
        self.get_logger().info("Mission CLI Node initialized.")
        self.prompt_ready = threading.Event()

    def status_callback(self, msg):
        """Prints status messages from the /mission_status topic."""
        self.prompt_ready.wait(timeout=5.0)
        status = msg.data
        
        # Track if mission is executing
        if status == "TRANSLATED":
            self._mission_executing = True
        elif status in ["NOT_FEASIBLE", "ERROR"]:
            self._mission_executing = False
        
        # Use Green for Success, Red for Failure, Blue for In-Progress/Other
        if status == "EXECUTED" or status == "TRANSLATED":
            color = bcolors.OKGREEN
        elif status == "NOT_FEASIBLE" or status == "ERROR":
            color = bcolors.FAIL
        else:
            color = bcolors.OKBLUE
        
        print(f"\r{color}↳ STATUS: {status}{bcolors.ENDC}")
        self.show_prompt()

    def ltl_callback(self, msg):
        """Prints LTL formula results from the /ltl_formula topic."""
        self.prompt_ready.wait(timeout=5.0)
        # LTL is critical debug info, use BOLD BLUE
        print(f"\r{bcolors.BOLD}{bcolors.OKBLUE}↳ LTL: {msg.data}{bcolors.ENDC}")
        self.show_prompt()

    def send_command(self, command: str):
        """Publishes a new mission command."""
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        # Confirmation that the command was successfully published
        print(f"{bcolors.OKGREEN}✓ Sent: \"{command}\"{bcolors.ENDC}")

    def show_prompt(self):
        """Utility to redraw the input prompt."""
        # Simple, bold prompt for clear input
        print(f"\n{bcolors.BOLD}> {bcolors.ENDC}", end="", flush=True)

    def odom_callback(self, msg):
        """Track drone position - only show during execution"""
        pos = msg.pose.pose.position
        self.current_position = (pos.x, pos.y, pos.z)
        
        # Only print if we have an active mission executing
        if not hasattr(self, '_mission_executing'):
            self._mission_executing = False
        
        if not hasattr(self, '_last_pos_print'):
            self._last_pos_print = 0
        
        # Only show position updates when mission is executing
        if self._mission_executing and time.time() - self._last_pos_print > 1.0:
            print(f"\r{bcolors.OKBLUE}[{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]{bcolors.ENDC}", end="", flush=True)
            self._last_pos_print = time.time()

    def run_cli_loop(self):
        """
        A separate thread to handle user input, so rclpy.spin() doesn't block it.
        """
        time.sleep(1)  # Wait for ROS node to fully initialize
        
        # Simplified and cleaner startup header
        print("\n" + "="*60)
        print(f"{bcolors.BOLD}UAV MISSION CONTROL CLI{bcolors.ENDC}")
        print("="*60)
        print(f"Waypoints: landing_pad, waypoint_a, waypoint_b, waypoint_c")
        print(f"{bcolors.WARNING}Note:{bcolors.ENDC} Type 'exit' to quit.")
        print("="*60)
        
        self.show_prompt()
        self.prompt_ready.set()
        
        try:
            for line in sys.stdin:
                command = line.strip()
                if not command:
                    self.show_prompt()
                    continue
                
                if command.lower() in ["exit", "quit"]:
                    print(f"\n{bcolors.WARNING}Shutting down CLI...{bcolors.ENDC}")
                    break
                
                self.send_command(command)
                
        except KeyboardInterrupt:
            print(f"\n{bcolors.WARNING}Ctrl+C detected. Shutting down...{bcolors.ENDC}")
        finally:
            rclpy.shutdown()


def main(args=None):
    """Entry point for the CLI."""
    rclpy.init(args=args)
    
    cli_node = MissionCLI()
    
    # Run the input loop in a separate daemon thread
    cli_thread = threading.Thread(target=cli_node.run_cli_loop, daemon=True)
    cli_thread.start()
    
    try:
        rclpy.spin(cli_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"CLI error: {e}")
    finally:
        cli_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cli_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()