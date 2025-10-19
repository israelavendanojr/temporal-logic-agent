"""LTL Grammar definitions for drone mission planning"""

class LTLGrammar:
    """Comprehensive LTL grammar for UAV operations"""
    
    # Temporal operators
    TEMPORAL_OPS = {
        'F': 'eventually',     # F(φ) - Eventually φ
        'G': 'globally',       # G(φ) - Always φ
        'X': 'next',          # X(φ) - Next φ
        'U': 'until',         # φ U ψ - φ Until ψ
        '&': 'and',           # φ & ψ - Both φ and ψ
        '|': 'or',            # φ | ψ - Either φ or ψ  
        '!': 'not'            # !φ - Not φ
    }
    
    # Spatial predicates
    SPATIAL_PREDICATES = {
        'at': ['location'],                    # at(waypoint_a)
        'near': ['location', 'radius'],        # near(waypoint_a, 2.0)
        'above': ['altitude'],                 # above(1.5)
        'below': ['altitude'],                 # below(3.0)
        'in_bounds': [],                       # in_bounds()
        'clear_of': ['obstacle']               # clear_of(obstacle_1)
    }
    
    # System state predicates
    SYSTEM_PREDICATES = {
        'battery_level': ['threshold'],        # battery_level(50)
        'moving': [],                          # moving()
        'stationary': []                       # stationary()
    }
    
    # Actions
    ACTIONS = {
        'move_to': ['location'],               # move_to(waypoint_a)
        'hover': ['duration'],                 # hover(5)
        'scan': ['area'],                      # scan(area_1)
        'land': [],                            # land()
        'emergency_return': []                 # emergency_return()
    }
    
    @classmethod
    def get_all_predicates(cls):
        """Return all valid predicates and actions"""
        all_preds = {}
        all_preds.update(cls.SPATIAL_PREDICATES)
        all_preds.update(cls.SYSTEM_PREDICATES)
        all_preds.update(cls.ACTIONS)
        return all_preds
