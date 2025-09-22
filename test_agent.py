"""
LangGraph ReAct Agent with Ollama
This script demonstrates a robust, multi-step pipeline for a mission planner agent.
"""

import operator
import argparse
from typing import TypedDict, Annotated, Sequence

from langchain_core.tools import tool
from langchain_core.messages import BaseMessage, HumanMessage, SystemMessage
from langchain_ollama import ChatOllama
from langgraph.prebuilt import ToolNode
from langgraph.graph import StateGraph, END

# Global debug flag
DEBUG = False

# -----------------------------
# Agent State Definition
# -----------------------------
class AgentState(TypedDict):
    """Represents the state of our agent."""
    messages: Annotated[Sequence[BaseMessage], operator.add]
    # Keep track of the LTL formula to pass between nodes
    ltl_formula: str


# -----------------------------
# Tools
# -----------------------------

MOCK_OBJECTS = {
    "object_1": {"name": "red cube", "location": "(1, 2, 3)"},
    "object_2": {"name": "blue sphere", "location": "(4, 5, 6)"},
    "object_3": {"name": "start location", "location": "(0, 0, 0)"}
}
MOCK_CRAZYFLIE_LOCATION = "(-0.1, 0.2, 0.1)"

def translate_with_llm(natural_language_query: str) -> str:
    """Helper function to perform the actual LLM-based translation."""
    llm = ChatOllama(model="llama3.2", temperature=0, base_url="http://localhost:11434")
    system_prompt = (
        "You translate Crazyflie drone mission commands into Linear Temporal Logic (LTL).\n"
        "- Output ONLY the LTL formula, no other text.\n"
        "- If the request is ambiguous (e.g., 'box'), output exactly AMBIGUOUS_QUERY.\n"
        "- If the request is impossible or unknown (e.g., 'Mars'), output F(at(unknown)).\n"
        "- Known objects: red cube (object_1), blue sphere (object_2), start location (object_3).\n"
        "- Use operators like F(at(object)), U, wait(n), return_to_start().\n"
        "Examples:\n"
        "User: fly to the red cube\n"
        "Output: F(at(object_1))\n"
        "User: fly to the blue sphere then fly to the red cube\n"
        "Output: F(at(object_2)) U F(at(object_1))\n"
    )
    response = llm.invoke([
        SystemMessage(content=system_prompt),
        HumanMessage(content=natural_language_query)
    ])
    return response.content.strip()

@tool
def translate_to_ltl(natural_language_query: str) -> str:
    """Translates a natural language command into a complete LTL formula."""
    return translate_with_llm(natural_language_query)

@tool
def validate_ltl_formula(ltl_formula: str) -> bool:
    """Verifies that an LTL formula is syntactically valid and grounded in the environment."""
    valid_formulas = [
        "F(at(object_1)) U F(at(object_2))", 
        "F(at(object_1)) U return_to_start()", 
        "F(at(object_1)) U wait(5) U F(at(object_2))",
        "F(at(object_2)) U wait(5) U F(at(object_1))",
        "move_up(6) U move_down(12) U P(at(initial_altitude))",
        "F(at(object_1))", 
        "F(at(object_2))", 
        "F(at(object_3))",
        "F(at(mars))",
        "F(at(unknown))"
    ]
    return ltl_formula in valid_formulas

@tool
def ask_for_clarification(ambiguous_query: str) -> str:
    """Asks the user for clarification on an ambiguous query."""
    if "box" in ambiguous_query.lower():
        return "There are multiple objects that could be considered 'a box'. Please specify which box you mean, for example 'the red cube'."
    return "I need more information to process your request. Can you please clarify?"

@tool
def check_feasibility(ltl_formula: str) -> str:
    """Checks if the LTL formula is physically possible to execute."""
    if "F(at(unknown))" in ltl_formula or "F(at(mars))" in ltl_formula:
        return "NOT FEASIBLE"
    return "FEASIBLE"

tools = [translate_to_ltl, validate_ltl_formula, ask_for_clarification, check_feasibility]


# -----------------------------
# Setup & Runner
# -----------------------------
def setup_agent(model_name: str = "llama3.2", temperature: float = 0):
    """Initializes model, tools, and compiles the LangGraph agent."""
    if DEBUG:
        print("Initializing Ollama model...")
    
    # We don't need llm.bind_tools() anymore as the logic is in the graph
    llm = ChatOllama(
        model=model_name,
        temperature=temperature,
        base_url="http://localhost:11434"
    )

    # Define the nodes as simple functions
    def translate_node(state: AgentState):
        """Translates the user's query into an LTL formula."""
        query = state['messages'][-1].content
        ltl = translate_to_ltl(query)
        # Update the state with the new formula
        return {"messages": [HumanMessage(content=ltl)], "ltl_formula": ltl}

    def validate_node(state: AgentState):
        """Validates the LTL formula."""
        ltl = state['ltl_formula']
        is_valid = validate_ltl_formula(ltl)
        return {"messages": [HumanMessage(content=str(is_valid))]}
    
    def check_feasibility_node(state: AgentState):
        """Checks the feasibility of the LTL formula."""
        ltl = state['ltl_formula']
        feasibility = check_feasibility(ltl)
        return {"messages": [HumanMessage(content=feasibility)]}
        
    def clarify_node(state: AgentState):
        """Asks the user for clarification."""
        query = state['messages'][0].content
        clarification_msg = ask_for_clarification(query)
        return {"messages": [HumanMessage(content=clarification_msg)]}
    
    def format_final_answer(state: AgentState):
        """Formats the final answer based on the feasibility check."""
        feasibility_result = state['messages'][-1].content
        if "FEASIBLE" in feasibility_result:
            return {"messages": [HumanMessage(content=state['ltl_formula'])]}
        else:
            return {"messages": [HumanMessage(content=feasibility_result)]}
            
    # Define the new, deterministic router
    def router(state: AgentState) -> str:
        """Determines the next step based on the LTL formula and validation results."""
        # Find the most recent message's content
        last_message_content = state['messages'][-1].content
        
        # Check output from translate_node
        if state.get('ltl_formula') and "AMBIGUOUS_QUERY" in state['ltl_formula']:
            return "clarify"
        elif state.get('ltl_formula') and "F(at(unknown))" in state['ltl_formula']:
            return "check_feasibility"
        elif state.get('ltl_formula') and last_message_content == state['ltl_formula']:
            return "validate"
            
        # Check output from validate_node
        if last_message_content == "True":
            return "check_feasibility"
        if last_message_content == "False":
            return "end"

        # Check output from check_feasibility_node
        if "FEASIBLE" in last_message_content or "NOT FEASIBLE" in last_message_content:
            return "final_answer"
            
        # Default to END if no other condition is met
        return "end"
        
    # --- Define the Graph ---
    workflow = StateGraph(AgentState)
    
    # Add nodes to the graph
    workflow.add_node("translate", translate_node)
    workflow.add_node("validate", validate_node)
    workflow.add_node("check_feasibility", check_feasibility_node)
    workflow.add_node("clarify", clarify_node)
    workflow.add_node("final_answer", format_final_answer)

    # Set the entry point to the translation node
    workflow.set_entry_point("translate")
    
    # Define conditional edges from the router
    workflow.add_conditional_edges("translate", router, {
        "validate": "validate",
        "check_feasibility": "check_feasibility",
        "clarify": "clarify",
        "end": END
    })
    
    # Define the other conditional edges
    workflow.add_conditional_edges("validate", router, {
        "check_feasibility": "check_feasibility",
        "end": "final_answer"
    })
    
    workflow.add_edge("check_feasibility", "final_answer")
    workflow.add_edge("clarify", END)
    workflow.add_edge("final_answer", END)

    return workflow.compile()


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
    
    app = setup_agent(model_name=args.model)
    
    print("Welcome to the Crazyflie Mission Planner Agent.")
    choice = input("Enter '1' for predefined queries, or '2' for interactive mode: ").strip()
    
    if choice == '1':
        queries = [
            "fly to the blue sphere and wait 5 seconds, then fly to the red cube",
            "Fly to object 1, then fly back to the start location",
            "fly to the red cube",
            "fly to the box",
            "Fly to Mars",
            "Find the treasure"
        ]
        run_queries(app, queries)
    elif choice == '2':
        run_interact(app)
    else:
        print("Invalid choice. Exiting.")

if __name__ == "__main__":
    main()