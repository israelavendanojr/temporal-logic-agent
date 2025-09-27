"""LTL Formula domain model"""
from dataclasses import dataclass
from typing import List, Optional, Dict, Any
import logging

logger = logging.getLogger(__name__)

@dataclass(frozen=True)
class LTLFormula:
    """Immutable LTL formula value object with validation state"""
    raw_formula: str
    tokens: List[str]
    is_valid: bool
    error_message: Optional[str] = None
    has_safety_constraints: bool = False
    
    @classmethod
    def create_from_raw(cls, raw_formula: str) -> 'LTLFormula':
        """Create LTL formula from raw string - validation will be done by services"""
        return cls(
            raw_formula=raw_formula,
            tokens=[],
            is_valid=True,  # Assume valid until validated
            error_message=None,
            has_safety_constraints=False
        )
    
    @classmethod
    def create_invalid(cls, raw_formula: str, error_message: str) -> 'LTLFormula':
        """Create invalid LTL formula with error"""
        return cls(
            raw_formula=raw_formula,
            tokens=[],
            is_valid=False,
            error_message=error_message,
            has_safety_constraints=False
        )
    
    def with_validation_result(self, tokens: List[str], is_valid: bool, error_message: Optional[str] = None) -> 'LTLFormula':
        """Return new formula with validation results"""
        return LTLFormula(
            raw_formula=self.raw_formula,
            tokens=tokens,
            is_valid=is_valid,
            error_message=error_message,
            has_safety_constraints=self.has_safety_constraints
        )
    
    def with_safety_constraints(self, safety_constraints: List[str]) -> 'LTLFormula':
        """Return new formula with safety constraints injected"""
        if not self.is_valid:
            return self
        
        if self.has_safety_constraints:
            return self  # Already has safety constraints
        
        safety_ltl = " & ".join(safety_constraints)
        enhanced_formula = f"({self.raw_formula}) & {safety_ltl}"
        
        return LTLFormula(
            raw_formula=enhanced_formula,
            tokens=self.tokens,
            is_valid=self.is_valid,
            error_message=self.error_message,
            has_safety_constraints=True
        )
    
    def __str__(self) -> str:
        return self.raw_formula
