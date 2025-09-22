"""
LangGraph ReAct Agent with Ollama
This script demonstrates a Thought-Action-Observation loop using LangGraph and a local Ollama model.
"""

import operator
from typing import TypedDict, Annotated, Sequence

from langchain_core.tools import tool
from langchain_core.messages import BaseMessage, HumanMessage
from langchain_ollama import ChatOllama
from langgraph.prebuilt import ToolNode
from langgraph.graph import StateGraph, END


# -----------------------------
# Agent State Definition
# -----------------------------
class AgentState(TypedDict):
    """Represents the state of our agent."""
    messages: Annotated[Sequence[BaseMessage], operator.add]
    ltl_formula: str  # Store the translated LTL formula
    environment_objects: str  # Context on known objects
    crazyflie_location: str  # Crazyflie's current location
    needs_clarification: bool  # A flag to signal if more info is needed


# -----------------------------
# Tools
# -----------------------------

# MOCK DATA for a stateful environment
MOCK_OBJECTS = {
    "object_1": {"name": "red cube", "location": "(1, 2, 3)"},
    "object_2": {"name": "blue sphere", "location": "(4, 5, 6)"},
    "object_3": {"name": "start location", "location": "(0, 0, 0)"}
}

MOCK_CRAZYFLIE_LOCATION = "(-0.1, 0.2, 0.1)"

@tool
def translate_to_ltl(natural_language_query: str) -> str:
    """Translates a natural language command into a complete LTL formula.
    
    The tool has knowledge of the following objects and their names:
    - red cube (object_1) at (1, 2, 3)
    - blue sphere (object_2) at (4, 5, 6)
    - start location (object_3) at (0, 0, 0)
    
    The Crazyflie's current location is (-0.1, 0.2, 0.1).
    
    Use the following LTL predicates:
    - F(at(object_id)): 'Eventually' at the object's location.
    - G(at(object_id)): 'Always' at the object's location.
    - U(left_operand, right_operand): 'Until' right_operand is true.
    - P(at(object_id)): 'Previously' at the object's location.
    - wait(seconds): Wait for a specified number of seconds.
    - return_to_start(): Return to the starting location (object_3).
    
    Example input: "Fly to object 1, then fly back to the start location"
    Example output: "F(at(object_1)) U return_to_start()"
    
    If the request is ambiguous or references an unknown object, return "AMBIGUOUS_QUERY"."""
    
    # In a full implementation, this would be an LLM call.
    # For now, it is a mock.
    
    # We will use simple if/else logic for a mock
    if "object 1, then fly back to the start" in natural_language_query:
        return "F(at(object_1)) U return_to_start()"
    if "object 1, wait 5 seconds, then fly to object 2" in natural_language_query:
        return "F(at(object_1)) U wait(5) U F(at(object_2))"
    if "Fly up 6 inches, then fly down 12, then return to your initial altitude" in natural_language_query:
        return "move_up(6) U move_down(12) U P(at(initial_altitude))"
    if "Fly to the box" in natural_language_query:
        return "AMBIGUOUS_QUERY"
    if "red cube" in natural_language_query.lower():
        return "F(at(object_1))"
    if "blue sphere" in natural_language_query.lower():
        return "F(at(object_2))"
    if "start location" in natural_language_query.lower():
        return "F(at(object_3))"
    
    return "F(at(unknown))"

@tool
def validate_ltl_formula(ltl_formula: str) -> bool:
    """Verifies that an LTL formula is syntactically valid and grounded in the environment."""
    
    # Mock validation. In a real system, you'd use a library like SPOT.
    if ltl_formula in ["F(at(object_1)) U return_to_start()", 
                      "F(at(object_1)) U wait(5) U F(at(object_2))",
                      "move_up(6) U move_down(12) U P(at(initial_altitude))",
                      "F(at(object_1))", "F(at(object_2))", "F(at(object_3))"]:
        return True
    
    return False

@tool
def ask_for_clarification(ambiguous_query: str) -> str:
    """Asks the user for clarification on an ambiguous query."""
    
    if "box" in ambiguous_query:
        return "There are multiple objects that could be considered 'a box'. Please specify which box you mean, for example 'the red cube'."
    
    return "I need more information to process your request. Can you please clarify?"

@tool
def check_feasibility(ltl_formula: str) -> bool:
    """Checks if the LTL formula is physically possible to execute."""
    
    # Mock feasibility check.
    if "F(at(unknown))" in ltl_formula:
        return False
    
    return True

tools = [translate_to_ltl, validate_ltl_formula, ask_for_clarification, check_feasibility]


# -----------------------------
# Setup & Runner
# -----------------------------
def setup_agent(model_name: str = "llama3.2", temperature: float = 0):
    """Initializes model, tools, and compiles the LangGraph agent."""
    print("Initializing Ollama model...")
    llm = ChatOllama(
        model=model_name,
        temperature=temperature,
        base_url="http://localhost:11434"
    )

    llm_with_tools = llm.bind_tools(tools)
    tool_node = ToolNode(tools)

    def call_model(state: AgentState):
        messages = state["messages"]
        
        # New System Prompt for the LLM
        system_prompt = f"""
        You are a highly specialized Crazyflie flight mission planner. Your sole purpose is to translate natural language commands into a formal Linear Temporal Logic (LTL) formula and ensure the plan is valid and ready for execution.

        You have access to the following tools:
        - translate_to_ltl: Use this FIRST to convert the user's request into an LTL formula. You must use this tool before any other action.
        - validate_ltl_formula: After translating, you MUST validate the LTL formula to ensure it is correct.
        - ask_for_clarification: If a query is ambiguous or you cannot translate it, you must use this tool to ask the user for more information.
        - check_feasibility: After validating the LTL, you must check if the plan is physically possible to execute.

        Here is the current state of the environment and the drone:
        - Current drone location: {MOCK_CRAZYFLIE_LOCATION}
        - Known objects: {MOCK_OBJECTS}

        Your workflow MUST be:
        1. Use `translate_to_ltl` on the user's request.
        2. If the translation is 'AMBIGUOUS_QUERY' or contains 'unknown', use `ask_for_clarification` with the ambiguous_query parameter set to the original user request.
        3. If the translation is successful, use `validate_ltl_formula`.
        4. If validation is successful, use `check_feasibility`.
        5. If all checks pass, respond with the final, validated LTL formula. Do not provide any other text.
        6. If any step fails, explain why and stop.
        """
        
        # Prepend the system prompt to the user message
        full_prompt = [HumanMessage(content=system_prompt)] + messages
        response = llm_with_tools.invoke(full_prompt)
        return {"messages": [response]}

    def should_continue(state: AgentState) -> str:
        messages = state["messages"]
        last_message = messages[-1]
        if hasattr(last_message, 'tool_calls') and last_message.tool_calls:
            return "tools"
        return "end"

    workflow = StateGraph(AgentState)
    workflow.add_node("agent", call_model)
    workflow.add_node("tools", tool_node)
    workflow.set_entry_point("agent")
    workflow.add_conditional_edges("agent", should_continue, {"tools": "tools", "end": END})
    workflow.add_edge("tools", "agent")

    return workflow.compile()

def run_agent(app, query: str):
    """Run the agent with a given query."""
    print(f"\nStarting agent with query: '{query}'")
    print("=" * 50)

    inputs = {"messages": [HumanMessage(content=query)]}

    step_count = 0
    for output in app.stream(inputs):
        step_count += 1
        print(f"\n--- Step {step_count} ---")
        for key, value in output.items():
            print(f"Node: {key}")
            if "messages" in value:
                last_message = value["messages"][-1]
                if hasattr(last_message, 'content'):
                    print(f"Content: {last_message.content}")
                if hasattr(last_message, 'tool_calls') and last_message.tool_calls:
                    print(f"Tool calls: {last_message.tool_calls}")

    final_state = app.invoke(inputs)
    final_answer = final_state["messages"][-1].content

    print("\n" + "=" * 50)
    print("Final Answer:")
    print(final_answer)
    return final_answer


# -----------------------------
# Main Functions
# -----------------------------
def run_queries(app, queries):
    """Run a list of queries through the agent."""
    for query in queries:
        try:
            run_agent(app, query)
            print("\n" + "-" * 50 + "\n")
        except Exception as e:
            print(f"Error processing query '{query}': {str(e)}")


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
        except Exception as e:
            print(f"Error: {str(e)}")


if __name__ == "__main__":
    app = setup_agent()

    queries = [
        "Fly to object 1, then fly back to the start location",
        "Fly to the box"  # This should trigger the ask_for_clarification tool
    ]

    run_queries(app, queries)
    # run_interact(app)  # You can enable this to test interactively
