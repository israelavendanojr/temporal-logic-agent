"""Simplified main entry point - no service factories"""
import argparse
import logging
from langchain_core.messages import HumanMessage
from agent.core import get_compiled_graph
from agent.config_loader import get_config

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

def run_agent(app, query: str, session_state: dict = None):
    """Run agent with session state tracking"""
    # Initialize session state on first run
    if session_state is None:
        config = get_config()
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
    
    # Prepare inputs for LangGraph
    inputs = {
        "messages": [HumanMessage(content=query)],
        "conversation_log": session_state['conversation_log'],
        "spatial_memory": session_state['spatial_memory']
    }
    
    # Execute LangGraph workflow
    final_answer = ""
    final_state = None
    for output in app.stream(inputs):
        for key, value in output.items():
            messages = value.get("messages", [])
            if messages and hasattr(messages[-1], 'content'):
                final_answer = messages[-1].content
            final_state = value
    
    # Update session state from results
    if final_state and 'conversation_log' in final_state:
        session_state['conversation_log'] = final_state['conversation_log']
        session_state['spatial_memory'] = final_state['spatial_memory']
    
    print(final_answer)
    return final_answer, session_state

def run_interactive(app):
    """Interactive mode with persistent session"""
    print("\nInteractive mode - type 'quit' to exit")
    session_state = None
    
    while True:
        try:
            query = input("\nQuery: ").strip()
            if query.lower() in ['quit', 'exit', 'q']:
                print("Goodbye!")
                break
            if query:
                _, session_state = run_agent(app, query, session_state)
        except KeyboardInterrupt:
            print("\nGoodbye!")
            break

def run_batch_queries(app):
    """Run predefined test queries"""
    test_queries = [
        "fly to waypoint_A",
        "go to waypoint_B then waypoint_C",
        "maintain altitude 2m until reaching base_station",
        "scan area_1 while avoiding obstacle_1",
        "hover at waypoint_A for 10 seconds",
        "always stay above 1 meter while going to waypoint_B",
    ]
    
    session_state = None
    for i, query in enumerate(test_queries, 1):
        print(f"\n{'='*60}")
        print(f"Query {i}: {query}")
        print(f"{'='*60}")
        _, session_state = run_agent(app, query, session_state)

def main():
    parser = argparse.ArgumentParser(description='UAV Task Planner - LTL Translation Agent')
    parser.add_argument('--interactive', action='store_true', help='Run in interactive mode')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Compile LangGraph workflow
    app = get_compiled_graph()
    
    # Run mode selection
    if args.interactive:
        run_interactive(app)
    else:
        run_batch_queries(app)

if __name__ == "__main__":
    main()