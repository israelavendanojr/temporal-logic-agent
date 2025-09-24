"""
GGUF Model Server for LTL Translation
Provides a wrapper around the fine-tuned GGUF model that mimics ChatOllama's interface.
"""

import os
import logging
from typing import List, Optional
from langchain_core.messages import BaseMessage, HumanMessage, SystemMessage
from langchain_core.callbacks import CallbackManagerForLLMRun

try:
    from llama_cpp import Llama
except ImportError:
    raise ImportError(
        "llama-cpp-python is required. Install it with: pip install llama-cpp-python"
    )

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class GGUFModelWrapper:
    """
    A wrapper around the fine-tuned GGUF model that provides a ChatOllama-like interface.
    """
    
    def __init__(self, model_path: str = "./models/translation_model.gguf", **kwargs):
        """
        Initialize the GGUF model wrapper.
        
        Args:
            model_path: Path to the GGUF model file
            **kwargs: Additional arguments passed to Llama model
        """
        self.model_path = model_path
        self.model = None
        self._load_model(**kwargs)
    
    def _load_model(self, **kwargs):
        """Load the GGUF model with appropriate settings."""
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model file not found: {self.model_path}")
        
        try:
            logger.info(f"Loading GGUF model from {self.model_path}")
            
            # Default parameters optimized for LTL translation
            default_params = {
                "model_path": self.model_path,
                "n_ctx": 2048,  # Context length for LTL outputs
                "n_threads": None,  # Use all available CPU cores
                "n_gpu_layers": 16,  
                "verbose": False,
                "temperature": 0.0,  # Deterministic output for LTL translation
                "top_p": 1.0,
                "top_k": 40,
                "repeat_penalty": 1.1,
                "stop": ["<|im_end|>", "\n\n"],  # Stop tokens from the chat template
            }
            
            # Merge with any provided kwargs
            default_params.update(kwargs)
            
            self.model = Llama(**default_params)
            logger.info("GGUF model loaded successfully")
            
        except Exception as e:
            logger.error(f"Failed to load GGUF model: {e}")
            raise
    
    def invoke(self, messages: List[BaseMessage], **kwargs) -> "MockResponse":
        """
        Invoke the model with a list of messages, mimicking ChatOllama's interface.
        
        Args:
            messages: List of BaseMessage objects (SystemMessage, HumanMessage, etc.)
            **kwargs: Additional arguments (currently unused but kept for compatibility)
            
        Returns:
            MockResponse object with the generated content
        """
        if self.model is None:
            raise RuntimeError("Model not loaded. Call _load_model() first.")
        
        # Extract system and user messages
        system_content = ""
        user_content = ""
        
        for message in messages:
            if isinstance(message, SystemMessage):
                system_content = message.content
            elif isinstance(message, HumanMessage):
                user_content = message.content
        
        # Format the input using the same chat template as the fine-tuned model
        formatted_input = f"""<|im_start|>system
{system_content}<|im_end|><|im_start|>user
{user_content}<|im_end|><|im_start|>assistant"""

        try:
            # Generate response with deterministic settings
            response = self.model(
                formatted_input,
                max_tokens=64,  # Short responses for LTL formulas
                temperature=0.0,  # Deterministic output
                top_p=1.0,
                top_k=40,
                repeat_penalty=1.1,
                stop=["<|im_end|>", "\n\n"],
                echo=False
            )
            
            # Extract the generated text and clean it
            generated_text = response["choices"][0]["text"].strip()
            
            # Clean up any remaining special tokens
            generated_text = generated_text.replace("<|im_end|>", "").strip()
            
            return MockResponse(content=generated_text)
            
        except Exception as e:
            logger.error(f"Error during model inference: {e}")
            return MockResponse(content="ERROR: Model inference failed")
    
    def __call__(self, messages: List[BaseMessage], **kwargs) -> "MockResponse":
        """Allow the wrapper to be called directly."""
        return self.invoke(messages, **kwargs)


class MockResponse:
    """
    A simple response object that mimics the structure of ChatOllama's response.
    """
    
    def __init__(self, content: str):
        self.content = content
    
    def __str__(self):
        return self.content


# Global model instance for reuse
_global_model: Optional[GGUFModelWrapper] = None


def get_gguf_model(model_path: str = "./models/translation_model.gguf") -> GGUFModelWrapper:
    """
    Get or create the global GGUF model instance.
    
    Args:
        model_path: Path to the GGUF model file
        
    Returns:
        GGUFModelWrapper instance
    """
    global _global_model
    
    if _global_model is None:
        logger.info("Creating new GGUF model instance")
        _global_model = GGUFModelWrapper(model_path)
    
    return _global_model


def clear_global_model():
    """Clear the global model instance (useful for testing)."""
    global _global_model
    _global_model = None
