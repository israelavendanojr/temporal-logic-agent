"""LTL validation service"""
from typing import List, Tuple
import logging
from domain.ltl_formula import LTLFormula

logger = logging.getLogger(__name__)

class ValidationService:
    """Service for validating LTL formulas"""
    
    def __init__(self, ltl_parser, safety_constraints: List[str]):
        self.parser = ltl_parser
        self.safety_constraints = safety_constraints
    
    def validate_and_enhance(self, formula: LTLFormula) -> LTLFormula:
        """Validate LTL formula and inject safety constraints"""
        # Skip validation for already invalid formulas
        if not formula.is_valid:
            return formula
        
        # Perform syntax validation
        tokens = self.parser.tokenize(formula.raw_formula)
        is_valid, error_msg = self.parser.validate_syntax(formula.raw_formula)
        
        # Update formula with validation results
        validated_formula = formula.with_validation_result(tokens, is_valid, error_msg)
        
        # Inject safety constraints if valid
        if validated_formula.is_valid:
            return validated_formula.with_safety_constraints(self.safety_constraints)
        
        return validated_formula
    
    def check_safety_compliance(self, formula: LTLFormula) -> Tuple[bool, List[str]]:
        """Check if formula includes required safety constraints"""
        missing_constraints = []
        
        for constraint in self.safety_constraints:
            if constraint not in formula.raw_formula:
                missing_constraints.append(constraint)
        
        return len(missing_constraints) == 0, missing_constraints
