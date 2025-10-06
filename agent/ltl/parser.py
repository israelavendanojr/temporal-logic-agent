"""Fixed LTL Parser for syntax validation and safety injection"""
import re
import logging
from typing import Tuple, Dict, List
from .grammar import LTLGrammar

logger = logging.getLogger(__name__)

class LTLParser:
    """Parse and validate LTL formulas with safety constraint injection"""
    
    def __init__(self):
        self.grammar = LTLGrammar()
        
    def tokenize(self, formula: str) -> List[str]:
        """Fixed tokenization that properly handles LTL formulas"""
        if not formula:
            return []
        
        # Clean the formula
        clean_formula = formula.strip()
        
        # More sophisticated tokenizer for LTL formulas
        tokens = []
        i = 0
        
        while i < len(clean_formula):
            char = clean_formula[i]
            
            # Skip whitespace
            if char.isspace():
                i += 1
                continue
            
            # Handle operators and parentheses
            if char in 'FGX!&|()+':
                tokens.append(char)
                i += 1
                continue
            
            # Handle word tokens (predicates/actions starting with letters)
            if char.isalpha() or char == '_':
                start = i
                while i < len(clean_formula) and (clean_formula[i].isalnum() or clean_formula[i] == '_'):
                    i += 1
                
                if i < len(clean_formula) and clean_formula[i] == '(':
                    # This is a predicate/action - capture the whole thing including parameters in parens
                    while i < len(clean_formula) and clean_formula[i] != ')':
                        i += 1
                    if i < len(clean_formula):
                        i += 1  # Include closing paren
                
                tokens.append(clean_formula[start:i])
                continue
            
            # Any other character (numbers, etc.) should be part of parameters
            if char.isdigit() or char == '.' or char == '-' or char == ',':
                start = i
                while (i < len(clean_formula) and 
                       (clean_formula[i].isalnum() or clean_formula[i] in '.,-')):
                    i += 1
                tokens.append(clean_formula[start:i])
                continue
                
            # Skip unrecognized characters
            i += 1
        
        # Clean up tokens - filter empty ones
        return [token for token in tokens if token.strip()]
        
    def validate_syntax(self, formula: str) -> Tuple[bool, str]:
        """Enhanced syntax validation with corrected logic flow"""
        if not formula or formula.strip() == "":
            return False, "Empty formula"
            
        # Handle error cases from translation
        if formula.startswith("INVALID_SYNTAX:") or formula.startswith("ERROR:"):
            return False, "Translation error"
            
        try:
            tokens = self.tokenize(formula)
            logger.debug(f"Tokenized formula '{formula}' into: {tokens}")
            
            i = 0
            while i < len(tokens):
                token = tokens[i]
                
                # Skip parentheses
                if token in ['(', ')']:
                    i += 1
                    continue
                    
                # Handle temporal operators
                elif token in self.grammar.TEMPORAL_OPS:
                    # For F, G, X - expect opening parenthesis next
                    if token in ['F', 'G', 'X']:
                        if i + 1 >= len(tokens) or tokens[i + 1] != '(':
                            return False, f"Temporal operator {token} must be followed by parentheses"
                    # For binary operators &, |, U - just continue
                    elif token in ['&', '|', 'U', '!']:
                        pass
                    i += 1
                    continue
                    
                # Handle predicates and actions
                elif '(' in token and ')' in token:
                    pred_name = token.split('(')[0]
                    if pred_name not in self.grammar.get_all_predicates():
                        return False, f"Unknown predicate: {pred_name}"
                    
                    # Validate predicate parameters
                    param_str = token[token.find('(')+1:token.rfind(')')]
                    if not self._validate_predicate_params(pred_name, param_str):
                        return False, f"Invalid parameters for {pred_name}: {param_str}"
                    i += 1
                    continue
                    
                # Handle simple tokens (should be rare in proper LTL)
                elif token.replace('_', '').replace('-', '').isalnum():
                    # Allow simple alphanumeric tokens (waypoint names, etc.)
                    i += 1
                    continue
                    
                else:
                    return False, f"Invalid token: {token}"
                    
            return True, "Valid syntax"
            
        except Exception as e:
            logger.error(f"Parser exception for formula '{formula}': {str(e)}")
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
    
    # def inject_safety_constraints(self, formula: str) -> str:
    #     """Automatically inject mandatory safety constraints - DISABLED"""
    #     # Safety injection temporarily disabled per professor guidance
    #     if not formula or formula.startswith("INVALID_SYNTAX:") or formula.startswith("ERROR:"):
    #         return formula
    #         
    #     safety_constraints = [
    #         "G(above(0.3))",
    #         "G(in_bounds())", 
    #         "G(clear_of(obstacles))"
    #     ]
    #     
    #     # Combine original formula with safety constraints
    #     safety_ltl = " & ".join(safety_constraints)
    #     return f"({formula}) & {safety_ltl}"
    
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