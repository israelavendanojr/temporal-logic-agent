"""Builder for creating mission context from session state"""
from typing import Dict, Any, List
from domain.mission_context import MissionContext, DroneState, Environment, ConversationEntry
from agent.config_loader import get_config

class ContextBuilder:
    """Builder for creating MissionContext from LangGraph session state"""
    
    def __init__(self):
        self.config = get_config()
    
    def build_from_session_state(self, spatial_memory: Dict[str, Any], conversation_log: List) -> MissionContext:
        """Build MissionContext from LangGraph session state"""
        
        # Build drone state
        current_pos = spatial_memory.get('current_position', (0.0, 0.0, 0.0))
        drone_state = DroneState(
            position=current_pos,
            battery_level=self.config.get_drone_state().get('current_battery', 85.0),
            is_moving=False  # Could be enhanced with actual movement detection
        )
        
        # Build environment
        flight_zone = self.config.get_flight_zone()
        environment = Environment(
            waypoints=self.config.get_waypoints(),
            obstacles=self.config.get_obstacles(),
            areas=self.config.get_areas(),
            flight_zone_min=flight_zone['min'],
            flight_zone_max=flight_zone['max']
        )
        
        # Build conversation history
        conversation_history = [
            ConversationEntry(user_query=user_q, ltl_result=ltl_result)
            for user_q, ltl_result in conversation_log
        ]
        
        return MissionContext(
            drone_state=drone_state,
            environment=environment,
            conversation_history=conversation_history
        )
