"""Mission feasibility service"""
import re
import logging
from typing import Dict, Tuple
from domain.ltl_formula import LTLFormula
from domain.mission_context import Environment

logger = logging.getLogger(__name__)

class FeasibilityService:
    """Service for checking mission feasibility"""
    
    def __init__(self, safety_params: Dict):
        self.safety_params = safety_params
    
    def check_mission_feasibility(self, formula: LTLFormula, environment: Environment) -> str:
        """Check if LTL formula represents a feasible mission"""
        
        # Handle invalid formulas
        if not formula.is_valid:
            if formula.error_message and "Translation failed" in formula.error_message:
                return "NOT FEASIBLE: Translation error"
            return "NOT FEASIBLE: Invalid LTL syntax"
        
        # Check waypoint feasibility
        waypoint_check = self._check_waypoint_feasibility(formula, environment)
        if waypoint_check != "FEASIBLE":
            return waypoint_check
        
        # Check altitude constraints
        altitude_check = self._check_altitude_feasibility(formula)
        if altitude_check != "FEASIBLE":
            return altitude_check
        
        return "FEASIBLE"
    
    def _check_waypoint_feasibility(self, formula: LTLFormula, environment: Environment) -> str:
        """Check if all referenced waypoints exist"""
        waypoint_pattern = r"(?:at|near|move_to)\((\w+)"
        referenced_waypoints = re.findall(waypoint_pattern, formula.raw_formula)
        
        for waypoint in referenced_waypoints:
            if waypoint not in environment.waypoints:
                return f"NOT FEASIBLE: Unknown waypoint '{waypoint}'"
        
        return "FEASIBLE"
    
    def _check_altitude_feasibility(self, formula: LTLFormula) -> str:
        """Check if altitude constraints are within safe limits"""
        altitude_pattern = r"(?:above|below)\(([0-9.]+)\)"
        altitudes = re.findall(altitude_pattern, formula.raw_formula)
        
        for alt_str in altitudes:
            altitude = float(alt_str)
            if altitude < self.safety_params['min_altitude'] or altitude > self.safety_params['max_altitude']:
                return f"NOT FEASIBLE: Altitude {altitude}m outside safe range"
        
        return "FEASIBLE"
