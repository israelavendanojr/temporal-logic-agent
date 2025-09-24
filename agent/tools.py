import re
import logging
from typing import TypedDict, Annotated, Sequence
from langchain_core.tools import tool
from langchain_core.messages import BaseMessage, HumanMessage, SystemMessage
from .model_server import get_gguf_model

# Set up logging
logger = logging.getLogger(__name__)

# MOCK DATA for a stateful environment
MOCK_OBJECTS = {
    "X": (1, 2, 3),
    "Y": (4, 5, 6),
    "Z": (0, 0, 0)
}
MOCK_CRAZYFLIE_LOCATION = (-0.1, 0.2, 0.1)

@tool
def translate_with_llm(natural_language_query: str) -> str:
    """Helper function to perform the actual LLM-based translation."""
    try:
        llm = get_gguf_model()
    except Exception as e:
        logger.error(f"Failed to load GGUF model: {e}")
        return "ERROR: Model not available"

    known_objects = ", ".join(MOCK_OBJECTS.keys())
    
    # Updated system prompt to match the fine-tuned model's training format
    system_prompt = (
        "You are a specialized translator that converts natural language drone commands into Linear Temporal Logic (LTL) formulas. "
        "Respond only with the LTL formula."
    )

    response = llm.invoke([
        SystemMessage(content=system_prompt),
        HumanMessage(content=natural_language_query)
    ])
    return response.content.strip()

@tool
def translate_to_ltl(natural_language_query: str, conversation_log: list = None, spatial_memory: dict = None) -> str:
    """Enhanced translation with full conversation context and spatial awareness."""
    
    # Build context string for system prompt
    context_parts = []
    
    if spatial_memory:
        context_parts.append(f"Current drone position: {spatial_memory.get('current_position')}")
        context_parts.append(f"Start position: {spatial_memory.get('start_position')}")
        context_parts.append(f"Object locations: {spatial_memory.get('objects')}")
    
    if conversation_log:
        context_parts.append("Previous conversation:")
        for user_q, ltl_result in conversation_log[-5:]:  # Last 5 exchanges
            context_parts.append(f"User: {user_q}")
            context_parts.append(f"LTL: {ltl_result}")
    
    # Enhanced system prompt with full context
    base_prompt = "You are a specialized translator that converts natural language drone commands into Linear Temporal Logic (LTL) formulas."
    
    if context_parts:
        context_section = "\n\nContext:\n" + "\n".join(context_parts)
        system_prompt = base_prompt + context_section + "\n\nFor complex multi-step queries, reason through the complete sequence. Respond only with the LTL formula."
    else:
        system_prompt = base_prompt + " Respond only with the LTL formula."

    response = get_gguf_model().invoke([
        SystemMessage(content=system_prompt),
        HumanMessage(content=natural_language_query)
    ])
    
    return response.content.strip()

@tool
def sanitize_ltl_formula(ltl_formula: str) -> str:
    """Cleans up and normalizes the LTL formula string for consistent validation."""
    cleaned = ltl_formula.replace(" ", "")
    cleaned = cleaned.replace("U", " U ")
    return " ".join(cleaned.split())

@tool
def validate_ltl_formula(ltl_formula: str) -> bool:
    """
    Verifies that an LTL formula is syntactically valid and grounded in the environment.
    """
    # Use a more robust approach with multiple regex patterns
    
    # Pattern for single LTL fragments
    fragment_pattern = re.compile(
        r"^(F\(at\((?:X|Y|Z|unknown)\)\)|"
        r"move\((?:forward|backward|up|down),\d+\)|"
        r"wait\(\d+\)|"
        r"return_to_start\(\))$"
    )
    
    # Pattern for sequences with 'U'
    sequence_pattern = re.compile(
        r"^(F\(at\((?:X|Y|Z|unknown)\)\s*U\s*)*"
        r"(move\((?:forward|backward|up|down),\d+\)\s*U\s*)*"
        r"(wait\(\d+\)\s*U\s*)*"
        r"(return_to_start\(\))?$"
    )
    
    # Check if the formula matches either a single fragment or a valid sequence
    if fragment_pattern.match(ltl_formula):
        return True
        
    # Split the formula by the 'U' operator and check each part
    parts = ltl_formula.split(' U ')
    
    if len(parts) > 1:
        for part in parts:
            if not fragment_pattern.match(part):
                return False
        return True
        
    return False

@tool
def check_feasibility(ltl_formula: str) -> str:
    """Checks if the LTL formula is physically possible to execute."""
    if "F(at(unknown))" in ltl_formula:
        return "NOT FEASIBLE"
    
    known_objects = MOCK_OBJECTS.keys()
    
    objects_in_formula = re.findall(r"at\((.*?)\)", ltl_formula)
    
    for obj in objects_in_formula:
        if obj not in known_objects and obj != "unknown":
            return "NOT FEASIBLE: Contains unknown objects."
            
    return "FEASIBLE"

@tool
def ask_for_clarification(ambiguous_query: str) -> str:
    """Asks the user for clarification on an ambiguous query."""
    return "I need more information to process your request. Can you please clarify?"