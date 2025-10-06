"""Semantic validation for LTL formulas beyond syntax checking"""
import re
import logging
from typing import Tuple, List, Set
from .grammar import LTLGrammar

logger = logging.getLogger(__name__)

class SemanticValidator:
    """Validates semantic correctness of LTL formulas"""
    
    def __init__(self):
        self.grammar = LTLGrammar()
    
    def validate(self, ltl_formula: str) -> Tuple[bool, str]:
        """
        Comprehensive semantic validation.
        
        Returns:
            (is_valid, error_message)
        """
        # Rule 1: Check for inverted obstacle avoidance
        is_valid, error = self._check_obstacle_negation(ltl_formula)
        if not is_valid:
            return False, error
        
        # Rule 2: Check for conflicting spatial constraints
        is_valid, error = self._check_spatial_conflicts(ltl_formula)
        if not is_valid:
            return False, error
        
        # Rule 3: Check for redundant constraints
        is_valid, warning = self._check_redundancy(ltl_formula)
        if warning:
            logger.warning(f"Semantic warning: {warning}")
        
        return True, "Semantically valid"
    
    def _check_obstacle_negation(self, formula: str) -> Tuple[bool, str]:
        """Rule: !clear_of(obstacle_X) is always semantically wrong"""
        obstacle_negation = re.search(r'!clear_of\((obstacle_\d+|perimeter_\w+)\)', formula)
        if obstacle_negation:
            obj = obstacle_negation.group(1)
            return False, f"Inverted obstacle avoidance: !clear_of({obj}) means 'collide with {obj}'"
        return True, ""
    
    def _check_spatial_conflicts(self, formula: str) -> Tuple[bool, str]:
        """Rule: Cannot be at multiple waypoints simultaneously with G()"""
        # Extract waypoints within G() blocks
        g_blocks = re.findall(r'G\([^)]*at\((\w+)\)[^)]*\)', formula)
        
        if len(set(g_blocks)) > 1:
            return False, f"Conflicting spatial constraint: Cannot always be at multiple waypoints {set(g_blocks)}"
        
        # Check for stationary + moving conflict
        if 'G(stationary())' in formula and 'moving()' in formula:
            return False, "Conflicting constraint: Cannot be stationary and moving"
        
        return True, ""
    
    def _check_redundancy(self, formula: str) -> Tuple[bool, str]:
        """Detect potentially redundant constraints (warning only)"""
        # Check for duplicate predicates
        predicates = re.findall(r'(at|above|below|scan|hover)\([^)]+\)', formula)
        if len(predicates) != len(set(predicates)):
            duplicates = [p for p in predicates if predicates.count(p) > 1]
            return True, f"Redundant predicates detected: {set(duplicates)}"
        
        return True, ""
    
    def extract_waypoints(self, formula: str) -> Set[str]:
        """Extract all referenced waypoints from formula"""
        return set(re.findall(r'at\((\w+)\)', formula))
    
    def extract_actions(self, formula: str) -> List[str]:
        """Extract all actions from formula in order"""
        actions = []
        for match in re.finditer(r'(scan|hover|land|emergency_return|move_to)\([^)]*\)', formula):
            actions.append(match.group(0))
        return actions
