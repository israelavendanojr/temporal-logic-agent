"""Simplified LangChain tools - direct implementation without service layer"""
from langchain_core.tools import tool
from langchain_core.messages import SystemMessage, HumanMessage
from agent.model_server import get_gguf_model
from agent.ltl.parser import LTLParser
from agent.config_loader import get_config
import re
import logging

logger = logging.getLogger(__name__)

# Module-level singletons - simple and direct
_model = None
_parser = None
_config = None

def get_model():
    """Lazy-load model"""
    global _model
    if _model is None:
        _model = get_gguf_model()
    return _model

def get_parser():
    """Lazy-load parser"""
    global _parser
    if _parser is None:
        _parser = LTLParser()
    return _parser

def get_env_config():
    """Lazy-load config"""
    global _config
    if _config is None:
        _config = get_config()
    return _config

@tool
def translate_to_ltl(natural_language_query: str, conversation_log: list = None, spatial_memory: dict = None) -> str:
    """Translate natural language to LTL with context - combines translation and validation"""
    config = get_env_config()
    waypoints = list(config.get_waypoints().keys())
    
    # Build system prompt with context
    system_prompt = """You translate drone commands to LTL formulas.

Operators: F(eventually), G(always), X(next), U(until), &(and), |(or), !(not)
Predicates: at(loc), above(alt), below(alt), scan(area), hover(dur), land(), move_to(loc)
Actions: emergency_return(), clear_of(obstacle), in_bounds()

Respond only with the LTL formula."""
    
    if waypoints:
        system_prompt += f"\nAvailable waypoints: {', '.join(waypoints)}"
    
    if conversation_log and len(conversation_log) > 0:
        system_prompt += "\nRecent exchanges:"
        for user_q, ltl in conversation_log[-2:]:
            system_prompt += f"\n  Q: {user_q} → {ltl}"
    
    # Generate translation
    try:
        response = get_model().invoke([
            SystemMessage(content=system_prompt),
            HumanMessage(content=natural_language_query)
        ])
        ltl = response.content.strip()
        
        # Validate syntax
        parser = get_parser()
        is_valid, error = parser.validate_syntax(ltl)
        
        if not is_valid:
            logger.warning(f"Invalid LTL syntax: {error}")
            return f"INVALID_SYNTAX: {error}"
        
        # Inject safety constraints
        safe_ltl = parser.inject_safety_constraints(ltl)
        return safe_ltl
        
    except Exception as e:
        logger.error(f"Translation error: {e}")
        return f"ERROR: {str(e)}"

@tool
def sanitize_ltl_formula(ltl_formula: str) -> str:
    """Normalize LTL formula spacing"""
    if ltl_formula.startswith(("INVALID_SYNTAX:", "ERROR:")):
        return ltl_formula
    
    # Normalize operator spacing
    for op in ['&', '|', 'U']:
        ltl_formula = ltl_formula.replace(f' {op} ', f' {op} ')
    
    # Remove extra whitespace
    return ' '.join(ltl_formula.split())

@tool
def validate_ltl_formula(ltl_formula: str) -> bool:
    """Validate LTL syntax"""
    if ltl_formula.startswith(("INVALID_SYNTAX:", "ERROR:")):
        return False
    
    parser = get_parser()
    is_valid, _ = parser.validate_syntax(ltl_formula)
    return is_valid

@tool
def check_feasibility(ltl_formula: str) -> str:
    """Check if mission is feasible given environment constraints"""
    if ltl_formula.startswith("INVALID_SYNTAX:"):
        return "NOT FEASIBLE: Invalid syntax"
    if ltl_formula.startswith("ERROR:"):
        return "NOT FEASIBLE: Translation error"
    
    config = get_env_config()
    waypoints = config.get_waypoints()
    safety_params = config.get_safety_params()
    
    # Check waypoints exist
    waypoint_pattern = r"(?:at|near|move_to)\((\w+)"
    for wp in re.findall(waypoint_pattern, ltl_formula):
        if wp not in waypoints:
            return f"NOT FEASIBLE: Unknown waypoint '{wp}'"
    
    # Check altitude constraints
    altitude_pattern = r"(?:above|below)\(([0-9.]+)\)"
    for alt_str in re.findall(altitude_pattern, ltl_formula):
        alt = float(alt_str)
        if alt < safety_params['min_altitude'] or alt > safety_params['max_altitude']:
            return f"NOT FEASIBLE: Altitude {alt}m outside safe range"
    
    return "FEASIBLE"

@tool
def ask_for_clarification(ambiguous_query: str) -> str:
    """Request user clarification"""
    return "I need more information to process your request. Can you please clarify?"
