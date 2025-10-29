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

# --- ANSI Color Codes ---
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

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
        
        self.get_logger().info("Mission CLI Node initialized.")
        self.prompt_ready = threading.Event()

    def status_callback(self, msg):
        """Prints status messages from the /mission_status topic."""
        self.prompt_ready.wait(timeout=5.0)
        status = msg.data
        color = bcolors.OKBLUE
        if status == "EXECUTED":
            color = bcolors.OKGREEN
        elif status == "NOT_FEASIBLE" or status == "ERROR":
            color = bcolors.FAIL
        
        print(f"\r{color}↳ STATUS: {status}{bcolors.ENDC}")
        self.show_prompt()

    def ltl_callback(self, msg):
        """Prints LTL formula results from the /ltl_formula topic."""
        self.prompt_ready.wait(timeout=5.0)
        print(f"\r{bcolors.OKCYAN}↳ LTL: {msg.data}{bcolors.ENDC}")
        self.show_prompt()

    def send_command(self, command: str):
        """Publishes a new mission command."""
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        print(f"{bcolors.OKGREEN}✓ Mission sent: \"{command}\"{bcolors.ENDC}")

    def show_prompt(self):
        """Utility to redraw the input prompt."""
        print(f"\n{bcolors.BOLD}🚁 > {bcolors.ENDC}", end="", flush=True)

    def run_cli_loop(self):
        """
        A separate thread to handle user input, so rclpy.spin() doesn't block it.
        """
        time.sleep(1)  # Wait for ROS node to fully initialize
        
        print("\n" + "="*60)
        print(f"{bcolors.BOLD}{bcolors.HEADER}UAV MISSION CONTROL CLI{bcolors.ENDC}")
        print("="*60)
        print(f"{bcolors.OKCYAN}Available waypoints:{bcolors.ENDC} landing_pad, waypoint_a, waypoint_b, waypoint_c")
        print(f"{bcolors.WARNING}Commands:{bcolors.ENDC} Type your mission and press Enter. Type 'exit' to quit.")
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