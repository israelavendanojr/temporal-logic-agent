"""Factory for creating application services"""
import logging
from agent.model_server import get_gguf_model
from agent.ltl.parser import LTLParser
from agent.config_loader import get_config
from services.translation_service import TranslationService
from services.validation_service import ValidationService
from services.feasibility_service import FeasibilityService

logger = logging.getLogger(__name__)

class ServicesFactory:
    """Factory for creating and configuring application services"""
    
    def __init__(self):
        self._config = get_config()
        self._model_provider = None
        self._parser = None
    
    def get_model_provider(self):
        """Get or create model provider"""
        if self._model_provider is None:
            self._model_provider = get_gguf_model()
        return self._model_provider
    
    def get_parser(self):
        """Get or create LTL parser"""
        if self._parser is None:
            self._parser = LTLParser()
        return self._parser
    
    def create_translation_service(self) -> TranslationService:
        """Create translation service with dependencies"""
        return TranslationService(self.get_model_provider())
    
    def create_validation_service(self) -> ValidationService:
        """Create validation service with dependencies"""
        safety_constraints = [
            "G(above(0.3))",
            "G(in_bounds())",
            "G(clear_of(obstacles))"
        ]
        return ValidationService(self.get_parser(), safety_constraints)
    
    def create_feasibility_service(self) -> FeasibilityService:
        """Create feasibility service with dependencies"""
        safety_params = self._config.get_safety_params()
        return FeasibilityService(safety_params)

# Global factory instance
_factory = None

def get_services_factory() -> ServicesFactory:
    """Get global services factory"""
    global _factory
    if _factory is None:
        _factory = ServicesFactory()
    return _factory
