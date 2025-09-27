"""LangChain tools using services architecture"""
from langchain_core.tools import tool
from langchain_core.messages import BaseMessage
from application.services_factory import get_services_factory
from application.context_builder import ContextBuilder
import logging

logger = logging.getLogger(__name__)

# Get services
factory = get_services_factory()
translation_service = factory.create_translation_service()
validation_service = factory.create_validation_service()
feasibility_service = factory.create_feasibility_service()
context_builder = ContextBuilder()

@tool
def translate_to_ltl(natural_language_query: str, conversation_log: list = None, spatial_memory: dict = None) -> str:
    """Enhanced translation using services architecture"""
    try:
        # Build mission context
        context = None
        if spatial_memory and conversation_log:
            context = context_builder.build_from_session_state(spatial_memory, conversation_log)
        
        # Translate query
        formula = translation_service.translate_query(natural_language_query, context)
        
        # Validate and enhance with safety
        enhanced_formula = validation_service.validate_and_enhance(formula)
        
        # Return result
        if enhanced_formula.is_valid:
            return enhanced_formula.raw_formula
        else:
            return f"INVALID_SYNTAX: {enhanced_formula.error_message}"
            
    except Exception as e:
        logger.error(f"Translation tool error: {e}")
        return f"ERROR: Translation failed - {str(e)}"

@tool
def sanitize_ltl_formula(ltl_formula: str) -> str:
    """Enhanced LTL sanitization - now mostly a pass-through since services handle validation"""
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
    """Enhanced LTL validation using services"""
    # Handle error cases from translation
    if ltl_formula.startswith("INVALID_SYNTAX:") or ltl_formula.startswith("ERROR:"):
        return False
    
    try:
        from domain.ltl_formula import LTLFormula
        
        # Create formula and validate
        formula = LTLFormula.create_from_raw(ltl_formula)
        validated_formula = validation_service.validate_and_enhance(formula)
        
        if not validated_formula.is_valid:
            logger.warning(f"LTL validation failed: {validated_formula.error_message}")
            return False
        
        # Check safety compliance
        safety_valid, missing_constraints = validation_service.check_safety_compliance(validated_formula)
        if not safety_valid:
            logger.warning(f"Missing safety constraints: {missing_constraints}")
            # Note: Don't fail validation for missing safety constraints since they should be auto-injected
        
        return True
        
    except Exception as e:
        logger.error(f"Validation tool error: {e}")
        return False

@tool
def check_feasibility(ltl_formula: str) -> str:
    """Enhanced feasibility checking using services"""
    try:
        from domain.ltl_formula import LTLFormula
        
        # Create formula
        formula = LTLFormula.create_from_raw(ltl_formula)
        
        # Handle error cases
        if ltl_formula.startswith("INVALID_SYNTAX:"):
            formula = LTLFormula.create_invalid(ltl_formula, "Invalid syntax")
        elif ltl_formula.startswith("ERROR:"):
            formula = LTLFormula.create_invalid(ltl_formula, "Translation error")
        
        # Build environment context
        context = context_builder.build_from_session_state({}, [])
        
        # Check feasibility
        return feasibility_service.check_mission_feasibility(formula, context.environment)
        
    except Exception as e:
        logger.error(f"Feasibility tool error: {e}")
        return f"NOT FEASIBLE: Error checking feasibility - {str(e)}"

@tool
def ask_for_clarification(ambiguous_query: str) -> str:
    """Asks the user for clarification on an ambiguous query."""
    return "I need more information to process your request. Can you please clarify?"
