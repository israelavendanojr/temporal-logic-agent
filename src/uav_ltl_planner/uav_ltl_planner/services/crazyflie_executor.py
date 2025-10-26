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
        self.kd = 0.5   # <-- NEW: Derivative Gain for PD control
        self.tolerance = 0.15
        self.max_velocity = 0.5
        self.hover_kp = 0.3
        self.rate = 10
        
        # State for PD control
        self.previous_error = (0.0, 0.0, 0.0) # <-- NEW: Store previous error for D-term calculation
        
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
        
        if not self.position_received.wait(timeout=5.0):
             self.node.get_logger().error("Cannot start plan: Timeout waiting for odometry.")
             return
        
        # --- NEW: Check if drone needs to take off before mission starts ---
        if self.current_position[2] < 0.2: # Assumes drone starts near Z=0
            if not self._takeoff(target_z=1.0):
                 self.node.get_logger().error("Takeoff failed. Aborting mission.")
                 self._send_zero_velocity()
                 return
        # -----------------------------------------------------------------

        for action in plan.actions:
            if self.stop_current_action.is_set():
                break
                
            if action.type == "move_to" and action.target:
                self._move_to(action.target)
            else:
                self.node.get_logger().warn(f"Ignoring unsupported action: {action.type}")
        
        # --- NEW: Land after mission completion ---
        self._land()
        # ----------------------------------------
    
    # --- NEW: Takeoff Method ---
    def _takeoff(self, target_z: float) -> bool:
        """Simple ascent to a safe altitude using current X/Y position."""
        if self.current_position is None: return False
        
        x, y, _ = self.current_position
        target = (x, y, target_z)
        self.node.get_logger().info(f"↑ Taking off to Z={target_z:.2f} at current X/Y position.")
        return self._navigate_to_target(target, "Takeoff_Altitude")

    # --- NEW: Land Method ---
    def _land(self) -> bool:
        """Simple controlled descent to Z=0.05m."""
        if self.current_position is None: return False
        
        x, y, _ = self.current_position
        target = (x, y, 0.05) # Land just above ground
        self.node.get_logger().info("↓ Landing...")
        return self._navigate_to_target(target, "Landing_Spot")
    # -----------------------

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
        
        # Reset previous error before starting navigation
        self.previous_error = (0.0, 0.0, 0.0) # <-- IMPORTANT: Reset D-term state
        
        # Navigate to target
        reached = self._navigate_to_target(target, waypoint_name)
        
        if reached:
            # Hover indefinitely
            self._hover_at_position(target, waypoint_name)
    
    def _navigate_to_target(self, target: tuple, waypoint_name: str) -> bool:
        """Navigate to target position using Proportional-Derivative (PD) control"""
        self.node.get_logger().info(f"Phase 1: Navigating to {waypoint_name}")
        
        sleep_time = 1.0 / self.rate
        last_log_time = time.time()
        
        while not self.stop_current_action.is_set():
            if self.current_position is None:
                self.node.get_logger().warn("Lost odometry during navigation")
                return False
            
            # P-TERM: Calculate position error (Error = Target - Current)
            dx = target[0] - self.current_position[0]
            dy = target[1] - self.current_position[1]
            dz = target[2] - self.current_position[2]
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # D-TERM: Calculate change in error over time (Derivative)
            delta_error_x = (dx - self.previous_error[0]) * self.rate # (Error_current - Error_previous) / dt, where dt=1/rate
            delta_error_y = (dy - self.previous_error[1]) * self.rate
            delta_error_z = (dz - self.previous_error[2]) * self.rate
            
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
            
            # PD Control: Velocity = Kp * Error + Kd * d(Error)/dt
            vx = self._limit_velocity(self.kp * dx + self.kd * delta_error_x)
            vy = self._limit_velocity(self.kp * dy + self.kd * delta_error_y)
            vz = self._limit_velocity(self.kp * dz + self.kd * delta_error_z)
            
            self._publish_velocity(vx, vy, vz)
            
            # Update previous error for the next D-term calculation
            self.previous_error = (dx, dy, dz)
            
            time.sleep(sleep_time)
        
        self._send_zero_velocity()
        return False
    
    def _hover_at_position(self, target: tuple, waypoint_name: str):
        """Hover indefinitely at target position using P control for station keeping"""
        self.node.get_logger().info(f"Phase 2: Hovering at {waypoint_name} - awaiting next command")
        
        sleep_time = 1.0 / self.rate
        
        while not self.stop_current_action.is_set():
            if self.current_position is None:
                break
            
            # Position correction (P-term only for slow station keeping)
            dx = target[0] - self.current_position[0]
            dy = target[1] - self.current_position[1]
            dz = target[2] - self.current_position[2]
            
            # Use lower max_vel for small corrections during hover
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
        # Note: Angular Z (Yaw) is left at 0.0, assuming fixed orientation
        self.cmd_pub.publish(msg)
    
    def _send_zero_velocity(self):
        """Send zero velocity command repeatedly to ensure motors stop"""
        for _ in range(5):
            self._publish_velocity(0.0, 0.0, 0.0)
            time.sleep(0.02)