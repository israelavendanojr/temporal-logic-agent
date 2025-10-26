from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .ltl_executor import ExecutionPlan
from ..agent.config_loader import get_config
import time
import math

class CrazyflieExecutor:
    """Bridge between LTL execution and Crazyflie"""
    
    def __init__(self, node):
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, '/crazyflie/cmd_vel', 10)
        self.waypoints = get_config().get_waypoints()
        
        # Subscribe to odometry to track current position
        self.current_position = None
        self.odom_sub = node.create_subscription(
            Odometry,
            '/crazyflie/odom',
            self.odom_callback,
            10
        )
        
        node.get_logger().info("CrazyflieExecutor connected to /crazyflie/cmd_vel")
    
    def odom_callback(self, msg):
        """Update current position from odometry"""
        pos = msg.pose.pose.position
        self.current_position = (pos.x, pos.y, pos.z)
    
    def run(self, plan: ExecutionPlan):
        """Execute LTL plan on Crazyflie"""
        self.node.get_logger().info(f"Executing {len(plan.actions)} actions on Crazyflie")
        
        for action in plan.actions:
            if action.type == "move_to" and action.target:
                self._move_to(action.target)
        
        return {"status": "complete"}
    
    def _move_to(self, waypoint_name: str):
        """Move drone to specified waypoint using proportional control"""
        if waypoint_name not in self.waypoints:
            self.node.get_logger().error(f"Unknown waypoint: {waypoint_name}")
            return
        
        target = self.waypoints[waypoint_name]
        self.node.get_logger().info(f"Moving to {waypoint_name} at {target}")
        
        # Wait for odometry to be available
        timeout = 5.0
        start_time = time.time()
        while self.current_position is None:
            if time.time() - start_time > timeout:
                self.node.get_logger().error("Timeout waiting for odometry")
                return
            time.sleep(0.1)
        
        # Proportional control parameters
        kp = 0.5  # Proportional gain (adjust based on testing)
        tolerance = 0.1  # Distance tolerance in meters
        max_velocity = 0.5  # Maximum velocity in m/s
        
        rate = 10  # Hz
        
        while True:
            if self.current_position is None:
                self.node.get_logger().warn("Lost odometry")
                break
            
            # Calculate error (distance to target)
            dx = target[0] - self.current_position[0]
            dy = target[1] - self.current_position[1]
            dz = target[2] - self.current_position[2]
            
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # Check if we've reached the target
            if distance < tolerance:
                self.node.get_logger().info(f"Reached {waypoint_name}")
                self._stop()
                break
            
            # Calculate velocity commands (proportional control)
            msg = Twist()
            msg.linear.x = max(min(kp * dx, max_velocity), -max_velocity)
            msg.linear.y = max(min(kp * dy, max_velocity), -max_velocity)
            msg.linear.z = max(min(kp * dz, max_velocity), -max_velocity)
            
            self.cmd_pub.publish(msg)
            time.sleep(1.0 / rate)
        
        self.node.get_logger().info("Movement complete")
    
    def _stop(self):
        """Stop the drone"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        self.cmd_pub.publish(msg)