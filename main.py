import argparse
from langchain_core.messages import HumanMessage
from agent.core import get_compiled_graph
from agent.tools import MOCK_CRAZYFLIE_LOCATION, MOCK_OBJECTS

# Global debug flag
DEBUG = False

# -----------------------------
# Main Functions
# -----------------------------
def run_agent(app, query: str, session_state: dict = None):
    """Run agent with persistent session state."""
    
    # Initialize session state on first run
    if session_state is None:
        session_state = {
            'conversation_log': [],
            'spatial_memory': {
                'current_position': MOCK_CRAZYFLIE_LOCATION,
                'start_position': MOCK_CRAZYFLIE_LOCATION,
                'objects': MOCK_OBJECTS
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
    # Initialize session state for all queries
    session_state = {
        'conversation_log': [],
        'spatial_memory': {
            'current_position': MOCK_CRAZYFLIE_LOCATION,
            'start_position': MOCK_CRAZYFLIE_LOCATION,
            'objects': MOCK_OBJECTS
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
    
    # Initialize session memory
    session_state = {
        'conversation_log': [],
        'spatial_memory': {
            'current_position': MOCK_CRAZYFLIE_LOCATION,
            'start_position': MOCK_CRAZYFLIE_LOCATION,
            'objects': MOCK_OBJECTS
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
            "Go to X, then go to Y, then go to Z, then retrace your steps back to the start",
            # Test absolute movements with varied phrasing
            "move to Y",
            "head over to Z",
            "fly to X",
            
            # Test waits with varied phrasing
            "pause for 15 seconds",
            "wait 5 seconds",
            
            # Test relative movements with varied phrasing
            "go forward 10 meters",
            "go up 2 meters",
            
            # Test complex, multi-step sequences
            "go to X then wait 5 seconds then go to Z",
            "fly up 10 meters, then fly to Y, then fly down 10 meters, then fly to Z",
            "fly forward 10 meters, then back to the start",
            
            # Test edge cases from the prompt's negative examples
            "find the treasure",
            "fly to Mars",
            "do a backflip",
            "say hello",
            "fly sideways 5 meters",

            "Go to X, then go to Y, then go to Z, then retrace your steps back",
        ]
        run_queries(app, queries)
    elif choice == '2':
        run_interact(app)
    else:
        print("Invalid choice. Exiting.")

if __name__ == "__main__":
    main()