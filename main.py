import argparse
from langchain_core.messages import HumanMessage
from agent.core import get_compiled_graph

# Global debug flag
DEBUG = False

# -----------------------------
# Main Functions
# -----------------------------
def run_agent(app, query: str):
    """Run the agent with a given query."""
    inputs = {"messages": [HumanMessage(content=query)]}
    final_answer = ""
    for output in app.stream(inputs):
        for key, value in output.items():
            final_answer = value.get("messages", [{}])[-1].content
    print(final_answer)
    return final_answer

def run_queries(app, queries):
    """Run a list of queries through the agent."""
    for i, query in enumerate(queries):
        print(f"\nQuery {i+1}: {query}")
        print("Result:", end=" ")
        run_agent(app, query)
        print("\n" + "-" * 50)

def run_interact(app):
    """Start an interactive session with the agent."""
    print("\nInteractive mode - type 'quit' to exit")
    while True:
        try:
            user_input = input("\nEnter your question: ").strip()
            if user_input.lower() in ['quit', 'exit', 'q']:
                print("Goodbye!")
                break
            if user_input:
                run_agent(app, user_input)
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
            "fly sideways 5 meters"
        ]
        run_queries(app, queries)
    elif choice == '2':
        run_interact(app)
    else:
        print("Invalid choice. Exiting.")

if __name__ == "__main__":
    main()