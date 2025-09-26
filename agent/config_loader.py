"""Configuration management for UAV Task Planner"""
import yaml
import os
import logging
from typing import Dict, Any
from .ltl.exceptions import ConfigError

logger = logging.getLogger(__name__)

class ConfigLoader:
    """Load and validate configuration from YAML files"""
    
    def __init__(self, config_path: str = "config/environment.yaml"):
        self.config_path = config_path
        self._config = None
        self.load_config()
    
    def load_config(self) -> None:
        """Load configuration from YAML file"""
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Config file not found: {self.config_path}")
            
        try:
            with open(self.config_path, 'r') as file:
                self._config = yaml.safe_load(file)
            
            self.validate_config()
        except Exception as e:
            raise ConfigError(f"Failed to load configuration: {e}")
    
    def validate_config(self) -> None:
        """Validate required configuration sections"""
        required_sections = ['flight_zone', 'waypoints', 'safety', 'drone']
        
        for section in required_sections:
            if section not in self._config:
                raise ConfigError(f"Missing required config section: {section}")
    
    def get_waypoints(self) -> Dict[str, tuple]:
        """Get waypoint locations as tuples"""
        waypoints = {}
        for name, coords in self._config['waypoints'].items():
            waypoints[name] = tuple(coords)
        return waypoints
    
    def get_flight_zone(self) -> Dict[str, tuple]:
        """Get flight zone boundaries"""
        return {
            'min': tuple(self._config['flight_zone']['min']),
            'max': tuple(self._config['flight_zone']['max'])
        }
    
    def get_obstacles(self) -> Dict[str, Dict]:
        """Get obstacle definitions"""
        return self._config.get('obstacles', {})
    
    def get_areas(self) -> Dict[str, Dict]:
        """Get area definitions"""
        return self._config.get('areas', {})
    
    def get_safety_params(self) -> Dict[str, Any]:
        """Get safety parameters"""
        return self._config['safety']
    
    def get_drone_state(self) -> Dict[str, Any]:
        """Get current drone state"""
        return self._config['drone']

# Global config instance
_config_instance = None

def get_config() -> ConfigLoader:
    """Get global config instance"""
    global _config_instance
    if _config_instance is None:
        _config_instance = ConfigLoader()
    return _config_instance
