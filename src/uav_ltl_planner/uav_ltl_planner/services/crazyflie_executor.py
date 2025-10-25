from geometry_msgs.msg import Twist
from .ltl_executor import ExecutionPlan
from ..agent.config_loader import get_config
import time

class CrazyflieExecutor:
    """Bridge between LTL execution and Crazyflie"""
    
    def __init__(self, node):
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, '/crazyflie/cmd_vel', 10)
        self.waypoints = get_config().get_waypoints()
        node.get_logger().info("CrazyflieExecutor connected to /crazyflie/cmd_vel")
    
    def run(self, plan: ExecutionPlan):
        """Execute LTL plan on Crazyflie"""
        self.node.get_logger().info(f"Executing {len(plan.actions)} actions on Crazyflie")
        
        for action in plan.actions:
            if action.type == "move_to" and action.target:
                self._move_to(action.target)
        
        return {"status": "complete"}
    
    def _move_to(self, waypoint_name: str):
        """Move drone upward (test movement)"""
        self.node.get_logger().info(f"Moving to {waypoint_name}")
        
        msg = Twist()
        msg.linear.z = 0.5  # Move up gently
        
        # Publish for 2 seconds
        for i in range(20):
            self.cmd_pub.publish(msg)
            time.sleep(0.1)
        
        # Stop
        msg.linear.z = 0.0
        self.cmd_pub.publish(msg)
        self.node.get_logger().info("Movement complete")