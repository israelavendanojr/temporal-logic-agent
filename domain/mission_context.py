"""Mission context domain model"""
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

@dataclass(frozen=True)
class DroneState:
    """Current drone state"""
    position: Tuple[float, float, float]
    battery_level: float
    is_moving: bool = False
    
@dataclass(frozen=True)
class Environment:
    """Environment information"""
    waypoints: Dict[str, Tuple[float, float, float]]
    obstacles: Dict[str, Dict]
    areas: Dict[str, Dict]
    flight_zone_min: Tuple[float, float, float]
    flight_zone_max: Tuple[float, float, float]

@dataclass(frozen=True)
class ConversationEntry:
    """Single conversation exchange"""
    user_query: str
    ltl_result: str

@dataclass(frozen=True)
class MissionContext:
    """Complete context for mission planning"""
    drone_state: DroneState
    environment: Environment
    conversation_history: List[ConversationEntry]
    
    def to_model_context(self) -> Dict:
        """Convert to dictionary for model consumption"""
        return {
            'waypoints': list(self.environment.waypoints.keys()),
            'current_position': self.drone_state.position,
            'conversation_history': [
                {'user': entry.user_query, 'ltl': entry.ltl_result}
                for entry in self.conversation_history[-3:]  # Last 3 exchanges
            ]
        }
