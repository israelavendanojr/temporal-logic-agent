"""Fixed LTL Parser for syntax validation and safety injection"""
import re
from typing import Tuple, Dict, List
from .grammar import LTLGrammar

class LTLParser:
    """Parse and validate LTL formulas with safety constraint injection"""
    
    def __init__(self):
        self.grammar = LTLGrammar()
        
    def tokenize(self, formula: str) -> List[str]:
        """Improved tokenization that handles complex formulas correctly"""
        # Remove all spaces first
        clean_formula = formula.replace(' ', '')
        
        # Tokenize with proper precedence handling
        # First pass: Handle predicates/actions with parentheses
        predicate_pattern = r'\w+\([^)]*\)'
        predicates = re.findall(predicate_pattern, clean_formula)
        
        # Replace predicates with placeholders to avoid interference
        temp_formula = clean_formula
        predicate_map = {}
        for i, pred in enumerate(predicates):
            placeholder = f"PRED{i}"
            predicate_map[placeholder] = pred
            temp_formula = temp_formula.replace(pred, placeholder, 1)
        
        # Second pass: Split on operators while preserving them
        operator_pattern = r'([FGX!&|()])'
        parts = re.split(operator_pattern, temp_formula)
        
        # Clean up and restore predicates
        tokens = []
        for part in parts:
            if part and part.strip():
                if part in predicate_map:
                    tokens.append(predicate_map[part])
                elif part != '':
                    tokens.append(part)
        
        # Filter out empty tokens
        return [token for token in tokens if token.strip()]
        
    def validate_syntax(self, formula: str) -> Tuple[bool, str]:
        """Enhanced syntax validation with better error reporting"""
        if not formula or formula.strip() == "":
            return False, "Empty formula"
            
        # Handle error cases from translation
        if formula.startswith("INVALID_SYNTAX:") or formula.startswith("ERROR:"):
            return False, "Translation error"
            
        try:
            tokens = self.tokenize(formula)
            
            for token in tokens:
                if token in ['(', ')', '&', '|', '!']:
                    continue
                elif token in self.grammar.TEMPORAL_OPS:
                    continue
                elif '(' in token and ')' in token:  # Predicate or action
                    pred_name = token.split('(')[0]
                    if pred_name not in self.grammar.get_all_predicates():
                        return False, f"Unknown predicate: {pred_name}"
                    
                    # Validate predicate parameters
                    param_str = token[token.find('(')+1:token.rfind(')')]
                    if not self._validate_predicate_params(pred_name, param_str):
                        return False, f"Invalid parameters for {pred_name}: {param_str}"
                        
                elif token.isalnum():  # Simple token
                    continue
                else:
                    return False, f"Invalid token: {token}"
                    
            return True, "Valid syntax"
            
        except Exception as e:
            return False, f"Parsing error: {str(e)}"
    
    def _validate_predicate_params(self, pred_name: str, param_str: str) -> bool:
        """Validate predicate parameters against grammar"""
        expected_params = self.grammar.get_all_predicates().get(pred_name, [])
        
        if not param_str and len(expected_params) == 0:
            return True
            
        if not param_str and len(expected_params) > 0:
            return False
            
        # For now, basic validation - can be enhanced later
        params = [p.strip() for p in param_str.split(',') if p.strip()]
        return len(params) <= len(expected_params) if expected_params else True
    
    def inject_safety_constraints(self, formula: str) -> str:
        """Automatically inject mandatory safety constraints"""
        if not formula or formula.startswith("INVALID_SYNTAX:") or formula.startswith("ERROR:"):
            return formula
            
        safety_constraints = [
            "G(above(0.3))",
            "G(in_bounds())", 
            "G(clear_of(obstacles))"
        ]
        
        # Combine original formula with safety constraints
        safety_ltl = " & ".join(safety_constraints)
        return f"({formula}) & {safety_ltl}"
    
    def simplify_for_compatibility(self, formula: str) -> str:
        """Temporary method to handle model output compatibility issues"""
        # Handle common model output patterns that cause parsing issues
        
        # Fix spaced operators
        formula = re.sub(r'\s*U\s*', ' U ', formula)
        formula = re.sub(r'\s*&\s*', ' & ', formula)
        formula = re.sub(r'\s*\|\s*', ' | ', formula)
        
        # Clean up extra spaces
        formula = ' '.join(formula.split())
        
        return formula