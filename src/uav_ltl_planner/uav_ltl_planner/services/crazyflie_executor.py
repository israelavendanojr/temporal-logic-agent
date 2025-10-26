from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .ltl_executor import ExecutionPlan
from ..agent.config_loader import get_config
import time
import math
import threading

class CrazyflieExecutor:
    """Simplified Crazyflie executor focused on smooth waypoint navigation"""
    
    def __init__(self, node):
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, '/crazyflie/cmd_vel', 10)
        self.waypoints = get_config().get_waypoints()
        
        # Current position tracking
        self.current_position = None
        self.position_received = threading.Event()
        self.odom_sub = node.create_subscription(
            Odometry,
            '/crazyflie/odom',
            self.odom_callback,
            10
        )
        
        # Control parameters
        self.kp = 0.8
        self.tolerance = 0.15
        self.max_velocity = 0.5
        self.hover_kp = 0.3
        self.rate = 10
        
        # Command interruption
        self.stop_current_action = threading.Event()
        self.action_lock = threading.Lock()
        
        node.get_logger().info("CrazyflieExecutor initialized")
        node.get_logger().info(f"Available waypoints: {list(self.waypoints.keys())}")
        
        # Start action thread
        self.action_thread = None
    
    def odom_callback(self, msg):
        """Update current position from odometry"""
        pos = msg.pose.pose.position
        self.current_position = (pos.x, pos.y, pos.z)
        self.position_received.set()
        # Debug: Log first position update
        if not hasattr(self, '_first_odom_logged'):
            self.node.get_logger().info(f"✓ Odometry callback working! Position: [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]")
            self._first_odom_logged = True
    
    def run(self, plan: ExecutionPlan):
        """Execute LTL plan in a separate thread to avoid blocking callbacks"""
        # Stop any ongoing action
        self.stop_current_action.set()
        if self.action_thread is not None and self.action_thread.is_alive():
            self.action_thread.join(timeout=1.0)
        
        self.stop_current_action.clear()
        
        # Start new action in thread
        self.action_thread = threading.Thread(
            target=self._execute_plan,
            args=(plan,),
            daemon=True
        )
        self.action_thread.start()
        
        return {"status": "started"}
    
    def _execute_plan(self, plan: ExecutionPlan):
        """Execute plan in separate thread"""
        self.node.get_logger().info(f"Executing {len(plan.actions)} actions")
        
        for action in plan.actions:
            if self.stop_current_action.is_set():
                break
                
            if action.type == "move_to" and action.target:
                self._move_to(action.target)
            else:
                self.node.get_logger().warn(f"Ignoring unsupported action: {action.type}")
    
    def _move_to(self, waypoint_name: str):
        """Move drone to absolute waypoint coordinates and hover indefinitely"""
        if waypoint_name not in self.waypoints:
            self.node.get_logger().error(f"Unknown waypoint: {waypoint_name}")
            return
        
        target = self.waypoints[waypoint_name]
        self.node.get_logger().info(f"→ Moving to {waypoint_name} at {target}")
        
        # Wait for odometry with timeout
        if not self.position_received.wait(timeout=5.0):
            self.node.get_logger().error("⚠ Timeout waiting for odometry")
            return
        
        # Navigate to target
        reached = self._navigate_to_target(target, waypoint_name)
        
        if reached:
            # Hover indefinitely
            self._hover_at_position(target, waypoint_name)
    
    def _navigate_to_target(self, target: tuple, waypoint_name: str) -> bool:
        """Navigate to target position using proportional control"""
        self.node.get_logger().info(f"Phase 1: Navigating to {waypoint_name}")
        
        sleep_time = 1.0 / self.rate
        last_log_time = time.time()
        
        while not self.stop_current_action.is_set():
            if self.current_position is None:
                self.node.get_logger().warn("Lost odometry during navigation")
                return False
            
            # Calculate position error
            dx = target[0] - self.current_position[0]
            dy = target[1] - self.current_position[1]
            dz = target[2] - self.current_position[2]
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # Log every 0.5 seconds
            if time.time() - last_log_time > 0.5:
                self.node.get_logger().info(
                    f"Current: [{self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}], "
                    f"Target: [{target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}], "
                    f"Distance: {distance:.2f}m"
                )
                last_log_time = time.time()
            
            # Check if reached target
            if distance < self.tolerance:
                self.node.get_logger().info(f"✓ Reached {waypoint_name} (error: {distance:.3f}m)")
                self._send_zero_velocity()
                return True
            
            # Proportional control
            vx = self._limit_velocity(self.kp * dx)
            vy = self._limit_velocity(self.kp * dy)
            vz = self._limit_velocity(self.kp * dz)
            
            self._publish_velocity(vx, vy, vz)
            time.sleep(sleep_time)
        
        self._send_zero_velocity()
        return False
    
    def _hover_at_position(self, target: tuple, waypoint_name: str):
        """Hover indefinitely at target position"""
        self.node.get_logger().info(f"Phase 2: Hovering at {waypoint_name} - awaiting next command")
        
        sleep_time = 1.0 / self.rate
        
        while not self.stop_current_action.is_set():
            if self.current_position is None:
                break
            
            # Position correction
            dx = target[0] - self.current_position[0]
            dy = target[1] - self.current_position[1]
            dz = target[2] - self.current_position[2]
            
            vx = self._limit_velocity(self.hover_kp * dx, max_vel=0.15)
            vy = self._limit_velocity(self.hover_kp * dy, max_vel=0.15)
            vz = self._limit_velocity(self.hover_kp * dz, max_vel=0.15)
            
            self._publish_velocity(vx, vy, vz)
            time.sleep(sleep_time)
        
        self._send_zero_velocity()
        self.node.get_logger().info(f"Hover at {waypoint_name} stopped")
    
    def _limit_velocity(self, velocity: float, max_vel: float = None) -> float:
        """Limit velocity to maximum value"""
        if max_vel is None:
            max_vel = self.max_velocity
        return max(min(velocity, max_vel), -max_vel)
    
    def _publish_velocity(self, vx: float, vy: float, vz: float):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        self.cmd_pub.publish(msg)
    
    def _send_zero_velocity(self):
        """Send zero velocity command"""
        for _ in range(5):
            self._publish_velocity(0.0, 0.0, 0.0)
            time.sleep(0.02)