"""Safety constraint definitions and validation"""
import logging
from typing import List, Tuple

logger = logging.getLogger(__name__)

class SafetyConstraints:
    """Mandatory safety constraints for all drone missions"""
    
    MANDATORY = [
        "G(above(0.3))",              # Always above 30cm ground clearance
        "G(in_bounds())",             # Always within flight zone
        "G(clear_of(obstacles))",     # Always avoid obstacles
    ]
    
    CONDITIONAL = [
        "battery_level(20) -> F(emergency_return())",  # Emergency return at 20%
        "battery_level(15) -> F(land())",              # Land at 15%
    ]
    
    @classmethod
    def get_all_constraints(cls):
        """Return all safety constraints"""
        return cls.MANDATORY + cls.CONDITIONAL
    
    @classmethod
    def validate_mission_safety(cls, ltl_formula: str) -> Tuple[bool, List[str]]:
        """Check if formula includes required safety constraints"""
        missing = []
        for constraint in cls.MANDATORY:
            if constraint not in ltl_formula:
                missing.append(constraint)
        
        return len(missing) == 0, missing
