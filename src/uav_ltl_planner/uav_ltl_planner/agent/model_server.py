"""
GGUF Model Server for LTL Translation - GPU OPTIMIZED
Provides a wrapper around the fine-tuned GGUF model that mimics ChatOllama's interface.
"""

import os
import logging
from typing import List, Optional
from langchain_core.messages import BaseMessage, HumanMessage, SystemMessage

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
        """Load the GGUF model with appropriate settings, prioritizing stable CPU/Metal fallback."""
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model file not found: {self.model_path}")
        
        # --- Stable Parameters (Default for Mac/Unstable GPUs) ---
        default_params = {
            "model_path": self.model_path,
            "n_ctx": 2048,
            "n_threads": 8,  # Maximize CPU core usage
            "n_gpu_layers": 0,  # CRITICAL: Forces CPU processing (or Metal fallback via llama_cpp)
            "verbose": False,
            "temperature": 0.0,
            "top_p": 1.0,
            "top_k": 40,
            "repeat_penalty": 1.1,
            "stop": ["<|im_end|>", "\n\n"],
            "n_batch": 512,
        }
        
        # Merge with any provided kwargs
        default_params.update(kwargs)
        
        try:
            logger.info(f"Loading GGUF model from {self.model_path}")
            
            self.model = Llama(**default_params)
            logger.info("Model loaded successfully in STABLE mode (CPU/Metal fallback)")
            logger.info(f"GPU layers offloaded: {default_params['n_gpu_layers']}")
            
        except Exception as e:
            # If even the stable load fails, raise the error.
            logger.error(f"Failed to load GGUF model in stable mode: {e}")
            raise RuntimeError(f"GGUF model failed to load: {e}")
    
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
        # Try to resolve model path using ROS2 package share directory
        resolved_model_path = _resolve_model_path(model_path)
        logger.info("Creating new GGUF model instance")
        _global_model = GGUFModelWrapper(resolved_model_path)
    
    return _global_model


def _resolve_model_path(model_path: str) -> str:
    """
    Resolve model path using ROS2 package share directory or fallback to relative path.
    
    Args:
        model_path: Original model path
        
    Returns:
        Resolved model path
    """
    # If it's already an absolute path, use it as is
    if os.path.isabs(model_path):
        return model_path
    
    # Try to resolve using ROS2 package share directory
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share_dir = get_package_share_directory('uav_ltl_planner')
        ros2_model_path = os.path.join(package_share_dir, 'models', 'translation_model.gguf')
        
        if os.path.exists(ros2_model_path):
            logger.info(f"Using ROS2 model path: {ros2_model_path}")
            return ros2_model_path
        else:
            logger.warning(f"ROS2 model path not found: {ros2_model_path}")
    except ImportError:
        logger.debug("ament_index_python not available, using fallback path resolution")
    except Exception as e:
        logger.warning(f"Failed to resolve ROS2 model path: {e}")
    
    # Fallback to relative path (for standalone mode)
    logger.info(f"Using fallback model path: {model_path}")
    return model_path


def clear_global_model():
    """Clear the global model instance (useful for testing)."""
    global _global_model
    _global_model = None