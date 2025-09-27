"""NLP to LTL translation service"""
from typing import Protocol, Optional
import logging
from domain.ltl_formula import LTLFormula
from domain.mission_context import MissionContext

logger = logging.getLogger(__name__)

class ModelProvider(Protocol):
    """Interface for LLM model providers"""
    def invoke(self, messages: list) -> object:
        """Invoke model with messages, return response object with .content"""
        ...

class TranslationService:
    """Service for translating natural language to LTL formulas"""
    
    def __init__(self, model_provider: ModelProvider):
        self.model_provider = model_provider
        self.base_prompt = """You are a specialized translator that converts natural language drone commands into Linear Temporal Logic (LTL) formulas.

Available LTL operators:
- F(φ) = eventually φ will be true
- G(φ) = φ is always true  
- X(φ) = φ is true in next step
- φ & ψ = both φ and ψ are true
- φ | ψ = either φ or ψ is true
- !φ = φ is not true

Available predicates and actions:
- at(location): drone at specific location
- near(location, radius): drone within radius of location
- above(altitude): drone above altitude threshold
- move_to(location): navigate to location
- hover(duration): maintain position for seconds
- scan(area): perform sensor sweep
- emergency_return(): return to start position
- land(): controlled landing

Do not include safety constraints in your response - they will be added automatically.
Respond only with the LTL formula."""
    
    def translate_query(self, query: str, context: Optional[MissionContext] = None) -> LTLFormula:
        """Translate natural language query to LTL formula"""
        try:
            # Build system prompt with context
            system_prompt = self._build_system_prompt(context)
            
            # Create messages for model
            from langchain_core.messages import SystemMessage, HumanMessage
            messages = [
                SystemMessage(content=system_prompt),
                HumanMessage(content=query)
            ]
            
            # Get translation from model
            response = self.model_provider.invoke(messages)
            raw_ltl = response.content.strip()
            
            # Return domain object
            return LTLFormula.create_from_raw(raw_ltl)
            
        except Exception as e:
            logger.error(f"Translation error for query '{query}': {e}")
            return LTLFormula.create_invalid(
                raw_formula="",
                error_message=f"Translation failed: {str(e)}"
            )
    
    def _build_system_prompt(self, context: Optional[MissionContext]) -> str:
        """Build system prompt with context information"""
        if not context:
            return self.base_prompt
        
        context_parts = []
        context_parts.append(f"Available waypoints: {list(context.environment.waypoints.keys())}")
        context_parts.append(f"Current drone position: {context.drone_state.position}")
        context_parts.append(f"Flight zone: min={context.environment.flight_zone_min}, max={context.environment.flight_zone_max}")
        
        if context.conversation_history:
            context_parts.append("Previous conversation:")
            for entry in context.conversation_history[-3:]:
                context_parts.append(f"User: {entry.user_query}, LTL: {entry.ltl_result}")
        
        return self.base_prompt + "\n\nContext:\n" + "\n".join(context_parts)
