import re
import logging
from typing import TypedDict, Annotated, Sequence
from langchain_core.tools import tool
from langchain_core.messages import BaseMessage, HumanMessage, SystemMessage
from .model_server import get_gguf_model
from .ltl.parser import LTLParser
from .config_loader import get_config
from ..services.ltl_executor import LTLExecutor

# Set up logging
logger = logging.getLogger(__name__)

def get_environment_data():
    """Get environment data from configuration"""
    config = get_config()
    return {
        'waypoints': config.get_waypoints(),
        'drone_position': tuple(config.get_drone_state()['start_position']),
        'flight_zone': config.get_flight_zone(),
        'obstacles': config.get_obstacles()
    }

@tool
def translate_with_llm(natural_language_query: str) -> str:
    """Helper function to perform the actual LLM-based translation."""
    try:
        llm = get_gguf_model()
    except Exception as e:
        logger.error(f"Failed to load GGUF model: {e}")
        return "ERROR: Model not available"

    # Get known objects from config instead of hardcoded values
    env_data = get_environment_data()
    known_objects = ", ".join(env_data['waypoints'].keys())
    
    # Updated system prompt to match the fine-tuned model's training format
    system_prompt = (
        f"You are a specialized translator that converts natural language drone commands into Linear Temporal Logic (LTL) formulas. "
        f"Available waypoints: {known_objects}. "
        "Respond only with the LTL formula."
    )

    response = llm.invoke([
        SystemMessage(content=system_prompt),
        HumanMessage(content=natural_language_query)
    ])
    return response.content.strip()

@tool
def translate_to_ltl(natural_language_query: str, conversation_log: list = None, spatial_memory: dict = None) -> str:
    """Minimal translation: query → LTL formula only."""
    env_data = get_environment_data()
    waypoints = list(env_data['waypoints'].keys())
    
    # Ultra-minimal prompt
    system_prompt = f"Convert drone command to LTL formula. Available waypoints: {waypoints}. Output ONLY the formula, nothing else."

    try:
        response = get_gguf_model().invoke([
            SystemMessage(content=system_prompt),
            HumanMessage(content=natural_language_query)
        ])
        
        ltl = response.content.strip()
        ltl = ltl.replace("'", "").replace('"', "")
        
        # Remove any explanatory text
        if ":" in ltl:
            ltl = ltl.split(":")[-1].strip()
        
        # Basic syntax validation only
        parser = LTLParser()
        is_valid, error_msg = parser.validate_syntax(ltl)
        
        if not is_valid:
            logger.warning(f"Invalid syntax: {error_msg}")
            return f"INVALID_SYNTAX: {error_msg}"
        
        return ltl
        
    except Exception as e:
        logger.error(f"Translation error: {e}")
        return "ERROR: Translation failed"
        
@tool
def sanitize_ltl_formula(ltl_formula: str) -> str:
    """Enhanced LTL sanitization for new grammar."""
    if ltl_formula.startswith("INVALID_SYNTAX:") or ltl_formula.startswith("ERROR:"):
        return ltl_formula
        
    # Basic cleanup - normalize spacing around operators
    cleaned = ltl_formula.replace(" & ", " & ").replace("&", " & ")
    cleaned = cleaned.replace(" | ", " | ").replace("|", " | ")
    cleaned = cleaned.replace(" U ", " U ").replace("U", " U ")
    
    # Remove extra whitespace
    cleaned = " ".join(cleaned.split())
    return cleaned

@tool
def validate_ltl_formula(ltl_formula: str) -> bool:
    """Enhanced LTL validation using proper parser."""
    # Handle error cases from translation
    if ltl_formula.startswith("INVALID_SYNTAX:") or ltl_formula.startswith("ERROR:"):
        return False
        
    parser = LTLParser()
    is_valid, error_msg = parser.validate_syntax(ltl_formula)
    
    if not is_valid:
        logger.warning(f"LTL validation failed: {error_msg}")
        return False
    
    # Safety validation disabled per professor guidance
    
    return True

@tool
def check_feasibility(ltl_formula: str) -> str:
    """Enhanced feasibility checking with config-driven environment."""
    # Handle error cases from translation
    if ltl_formula.startswith("INVALID_SYNTAX:"):
        return "NOT FEASIBLE: Invalid LTL syntax"
    if ltl_formula.startswith("INVALID_SEMANTICS:"):
        return "NOT FEASIBLE: Semantic validation error"
    if ltl_formula.startswith("ERROR:"):
        return "NOT FEASIBLE: Translation error"
        
    # Detect model translation failure pattern F(at(unknown)) early
    # This catches cases where the model failed to map a query (e.g., altitude/hover)
    # to a valid predicate and instead produced an invalid placeholder waypoint.
    unknown_pattern = r"\bF\(\s*at\(unknown\)\s*\)"
    if re.search(unknown_pattern, ltl_formula):
        return "NOT FEASIBLE: Translation error - model generated invalid waypoint 'unknown'"

    env_data = get_environment_data()
    waypoints = env_data['waypoints']
    
    # Check for unknown waypoints
    waypoint_pattern = r"(?:at|near|move_to)\((\w+)"
    referenced_waypoints = re.findall(waypoint_pattern, ltl_formula)
    
    for waypoint in referenced_waypoints:
        if waypoint not in waypoints:
            return f"NOT FEASIBLE: Unknown waypoint '{waypoint}'"
    
    # Check altitude constraints
    altitude_pattern = r"(?:above|below)\(([0-9.]+)\)"
    altitudes = re.findall(altitude_pattern, ltl_formula)
    config = get_config()
    safety_params = config.get_safety_params()
    
    for alt in altitudes:
        altitude = float(alt)
        if altitude < safety_params['min_altitude'] or altitude > safety_params['max_altitude']:
            return f"NOT FEASIBLE: Altitude {altitude}m outside safe range"
    
    return "FEASIBLE"

@tool
def execute_ltl_formula(ltl_formula: str) -> str:
    """Generate an execution plan from an LTL formula - no mock execution."""
    try:
        executor = LTLExecutor()
        plan = executor.parse_formula(ltl_formula)
        return f"Plan generated: {len(plan.actions)} actions"
    except Exception as e:
        logger.error(f"Plan generation failed: {e}")
        return f"ERROR: Plan generation failed - {e}"

@tool
def ask_for_clarification(ambiguous_query: str) -> str:
    """Asks the user for clarification on an ambiguous query."""
    return "I need more information to process your request. Can you please clarify?"
