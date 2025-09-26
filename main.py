import argparse
import logging
from langchain_core.messages import HumanMessage
from agent.core import get_compiled_graph
from agent.config_loader import get_config

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(name)s %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

# Global debug flag
DEBUG = False

# -----------------------------
# Main Functions
# -----------------------------
def run_agent(app, query: str, session_state: dict = None):
    """Run agent with config-driven session state."""
    
    config = get_config()
    
    # Initialize session state on first run
    if session_state is None:
        drone_state = config.get_drone_state()
        session_state = {
            'conversation_log': [],
            'spatial_memory': {
                'current_position': tuple(drone_state['start_position']),
                'start_position': tuple(drone_state['start_position']),
                'objects': config.get_waypoints(),
                'flight_zone': config.get_flight_zone(),
                'obstacles': config.get_obstacles()
            }
        }
    
    # Add session state to inputs
    inputs = {
        "messages": [HumanMessage(content=query)],
        "conversation_log": session_state['conversation_log'],
        "spatial_memory": session_state['spatial_memory']
    }
    
    # Run workflow
    final_answer = ""
    final_state = None
    for output in app.stream(inputs):
        for key, value in output.items():
            messages = value.get("messages", [])
            if messages and hasattr(messages[-1], 'content'):
                final_answer = messages[-1].content
            final_state = value
    
    # Update session state with results
    if final_state and 'conversation_log' in final_state:
        session_state['conversation_log'] = final_state['conversation_log']
        session_state['spatial_memory'] = final_state['spatial_memory']
    
    print(final_answer)
    return final_answer, session_state

def run_queries(app, queries):
    """Run a list of queries through the agent."""
    config = get_config()
    drone_state = config.get_drone_state()
    
    # Initialize session state for all queries
    session_state = {
        'conversation_log': [],
        'spatial_memory': {
            'current_position': tuple(drone_state['start_position']),
            'start_position': tuple(drone_state['start_position']),
            'objects': config.get_waypoints()
        }
    }
    
    for i, query in enumerate(queries):
        print(f"\nQuery {i+1}: {query}")
        print("Result:", end=" ")
        _, session_state = run_agent(app, query, session_state)
        print("\n" + "-" * 50)

def run_interact(app):
    """Interactive mode with persistent memory."""
    print("\nInteractive mode with memory - type 'quit' to exit")
    
    config = get_config()
    drone_state = config.get_drone_state()
    
    # Initialize session memory
    session_state = {
        'conversation_log': [],
        'spatial_memory': {
            'current_position': tuple(drone_state['start_position']),
            'start_position': tuple(drone_state['start_position']),
            'objects': config.get_waypoints()
        }
    }
    
    while True:
        try:
            user_input = input("\nEnter your question: ").strip()
            if user_input.lower() in ['quit', 'exit', 'q']:
                print("Goodbye!")
                break
            if user_input:
                _, session_state = run_agent(app, user_input, session_state)
        except KeyboardInterrupt:
            print("\nGoodbye!")
            break

def main():
    """Main function to run the agent."""
    global DEBUG
    
    parser = argparse.ArgumentParser(description='Crazyflie Mission Planner Agent')
    parser.add_argument('--debug', action='store_true', help='Enable debug output')
    parser.add_argument('--model', default='llama3.2', help='Ollama model to use')
    args = parser.parse_args()
    
    DEBUG = args.debug
    
    app = get_compiled_graph()
    
    print("Welcome to the Crazyflie Mission Planner Agent.")
    choice = input("Enter '1' for predefined queries, or '2' for interactive mode: ").strip()
    
    if choice == '1':
        queries = [
            # Test new waypoint system
            "fly to waypoint_a",
            "go to waypoint_b then waypoint_c", 
            "patrol between waypoint_a and waypoint_b",
            
            # Test enhanced LTL with safety
            "hover at waypoint_a for 10 seconds",
            "scan area_1 then return to base",
            "fly above 2 meters to waypoint_b",
            
            # Test safety constraints
            "emergency return to start",
            "land at landing_pad",
            
            # Test complex temporal logic
            "always stay above 1 meter while going to waypoint_a",
            "go to waypoint_b and scan area_1",
            
            # Test error cases
            "fly to unknown_location",
            "fly below ground level",
            
            # Backward compatibility tests
            "fly to X",  # Should still work via config
            "go to Y then Z",
        ]
        run_queries(app, queries)
    elif choice == '2':
        run_interact(app)
    else:
        print("Invalid choice. Exiting.")

if __name__ == "__main__":
    main()