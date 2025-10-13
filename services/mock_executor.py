import time
import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from .ltl_executor import Action, Constraint, ExecutionPlan
from agent.config_loader import get_config


logger = logging.getLogger(__name__)


@dataclass
class DroneState:
    position: Tuple[float, float, float]
    battery: float
    is_moving: bool = False
    history: List[Tuple[str, Any]] = field(default_factory=list)


class MockExecutor:
    """Simulates execution of an ExecutionPlan while validating constraints."""

    def __init__(self):
        cfg = get_config()
        drone_cfg = cfg.get_drone_state()
        start_pos = tuple(drone_cfg.get("start_position", (0.0, 0.0, 0.0)))
        start_batt = float(drone_cfg.get("battery", 100.0))
        self.state = DroneState(position=start_pos, battery=start_batt)
        self.env_waypoints = cfg.get_waypoints()
        self.flight_zone = cfg.get_flight_zone()
        self.obstacles = cfg.get_obstacles()
        self.safety = cfg.get_safety_params()

    def _log(self, msg: str) -> None:
        logger.info(msg)
        print(msg)

    def _validate_constraints(self, constraints: List[Constraint]) -> None:
        for c in constraints:
            if c.type == "altitude":
                z = self.state.position[2]
                min_alt = c.params.get("min")
                max_alt = c.params.get("max")
                if min_alt is not None and z < min_alt:
                    self._log(f"[WARN] Altitude {z:.2f}m below minimum {min_alt:.2f}m")
                if max_alt is not None and z > max_alt:
                    self._log(f"[WARN] Altitude {z:.2f}m above maximum {max_alt:.2f}m")
            elif c.type == "obstacle":
                obs = c.params.get("object")
                if obs and obs not in self.obstacles:
                    self._log(f"[WARN] Referenced obstacle '{obs}' not found in environment")

    def _apply_battery_use(self, amount: float) -> None:
        before = self.state.battery
        self.state.battery = max(0.0, self.state.battery - amount)
        self._log(f"    Battery: {before:.0f}% → {self.state.battery:.0f}%")

    def _move_to(self, waypoint: str) -> None:
        if waypoint not in self.env_waypoints:
            self._log(f"[ERROR] Unknown waypoint '{waypoint}'")
            return
        target = self.env_waypoints[waypoint]
        self._log(f"    Moving to {waypoint} {target}")
        self.state.is_moving = True
        # Simulate time to move proportional to distance
        dx = target[0] - self.state.position[0]
        dy = target[1] - self.state.position[1]
        dz = target[2] - self.state.position[2]
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        travel_time = max(0.5, dist)
        time.sleep(min(travel_time, 2.5))  # cap sleep to keep tests responsive
        self.state.position = (float(target[0]), float(target[1]), float(target[2]))
        self.state.is_moving = False
        self._log("    Arrived")
        self._apply_battery_use(3.0)
        self.state.history.append(("move_to", waypoint))

    def _hover(self, duration: float) -> None:
        self._log(f"    Hovering for {duration:.1f}s")
        time.sleep(min(duration, 2.0))
        self._apply_battery_use(1.0)
        self.state.history.append(("hover", duration))

    def _scan(self, area: str) -> None:
        self._log(f"    Scanning {area}")
        time.sleep(0.5)
        self._apply_battery_use(1.0)
        self.state.history.append(("scan", area))

    def _land(self) -> None:
        self._log("    Landing")
        time.sleep(0.5)
        self.state.position = (self.state.position[0], self.state.position[1], 0.0)
        self._apply_battery_use(0.5)
        self.state.history.append(("land", None))

    def _emergency_return(self) -> None:
        self._log("    Emergency return to start")
        start_x, start_y, start_z = 0.0, 0.0, 0.0
        self.state.position = (start_x, start_y, start_z)
        self._apply_battery_use(5.0)
        self.state.history.append(("emergency_return", None))

    def run(self, plan: ExecutionPlan) -> Dict[str, Any]:
        start_pos = self.state.position
        self._log(f"[00.0s] Starting at position {start_pos}")
        if plan.constraints:
            self._log("Constraints:")
            for c in plan.constraints:
                if c.type == "altitude" and "min" in c.params:
                    self._log(f"  - Maintain altitude > {c.params['min']}m")
                elif c.type == "altitude" and "max" in c.params:
                    self._log(f"  - Maintain altitude < {c.params['max']}m")
                elif c.type == "obstacle":
                    self._log(f"  - Keep clear of {c.params.get('object')}")

        for action in plan.actions:
            self._validate_constraints(plan.constraints)
            self._log(f"[ACTION] {action.type}{'(' + str(action.target) + ')' if action.target else ''}")
            if action.type == "move_to" and action.target:
                self._move_to(action.target)
            elif action.type == "hover":
                self._hover(action.duration or 0.0)
            elif action.type == "scan" and action.target:
                self._scan(action.target)
            elif action.type == "land":
                self._land()
            elif action.type == "emergency_return":
                self._emergency_return()
            else:
                self._log(f"[WARN] Unsupported action {action}")

        self._log("[DONE] Mission completed")
        return {
            "final_position": self.state.position,
            "battery": self.state.battery,
            "history": list(self.state.history),
        }


