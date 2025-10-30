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

class bcolors:
    OKGREEN = '\033[92m'
    OKBLUE = '\033[94m'
    FAIL = '\033[91m'
    WARNING = '\033[93m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'

class MissionCLI(Node):
    def __init__(self):
        super().__init__('mission_cli_node')
        
        self.command_publisher = self.create_publisher(String, '/mission_command', 10)
        self.status_subscriber = self.create_subscription(String, '/mission_status', self.status_callback, 10)
        self.ltl_subscriber = self.create_subscription(String, '/ltl_formula', self.ltl_callback, 10)
        
        self.last_ltl = None
        self.last_status = None
        self.get_logger().info("Mission CLI Node initialized.")
        self.prompt_ready = threading.Event()

    def status_callback(self, msg):
        self.prompt_ready.wait(timeout=5.0)
        status = msg.data
        
        # Filter duplicate status messages
        if status == self.last_status:
            return
        self.last_status = status
        
        if status == "EXECUTED" or status == "TRANSLATED":
            color = bcolors.OKGREEN
        elif status == "NOT_FEASIBLE" or status == "ERROR":
            color = bcolors.FAIL
        else:
            color = bcolors.OKBLUE
        
        print(f"\r{color}↳ STATUS: {status}{bcolors.ENDC}")
        self.show_prompt()

    def ltl_callback(self, msg):
        self.prompt_ready.wait(timeout=5.0)
        ltl = msg.data
        
        # Filter duplicates
        if ltl == self.last_ltl:
            return
        self.last_ltl = ltl
        
        # Show actual LTL formulas separately from execution results
        if ltl.startswith("F(") or ltl.startswith("G("):
            print(f"\r{bcolors.BOLD}{bcolors.OKBLUE}↳ LTL Translation: {ltl}{bcolors.ENDC}")
        else:
            print(f"\r{bcolors.OKBLUE}↳ Execution: {ltl}{bcolors.ENDC}")
        
        self.show_prompt()

    def send_command(self, command: str):
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        print(f"{bcolors.OKGREEN}✓ Sent: \"{command}\"{bcolors.ENDC}")

    def show_prompt(self):
        print(f"\n{bcolors.BOLD}> {bcolors.ENDC}", end="", flush=True)

    def run_cli_loop(self):
        time.sleep(1)
        
        print("\n" + "="*60)
        print(f"{bcolors.BOLD}UAV MISSION CONTROL CLI{bcolors.ENDC}")
        print("="*60)
        print(f"Waypoints: landing_pad, waypoint_a, waypoint_b, waypoint_c")
        print(f"Note:{bcolors.ENDC} Type 'exit' to quit.")
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
    rclpy.init(args=args)
    cli_node = MissionCLI()
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