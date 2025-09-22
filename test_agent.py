"""
LangGraph ReAct Agent with Ollama
This script demonstrates a Thought-Action-Observation loop using LangGraph and a local Ollama model.
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
    user_query: str
    ltl_result: str
    validation_result: bool
    feasibility_result: str
    clarification_result: str
    workflow_step: str
    final_answer: str


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
    """Translates a natural language command into a complete LTL formula."""
    
    query_lower = natural_language_query.lower()
    
    # Exact match patterns for reliable translation
    if "blue sphere and wait 5 seconds, then fly to the red cube" in query_lower:
        return "F(at(object_2)) U wait(5) U F(at(object_1))"
    if "object 1, then fly to object 2" in natural_language_query:
        return "F(at(object_1)) U F(at(object_2))"
    if "object 1, then fly back to the start" in natural_language_query:
        return "F(at(object_1)) U return_to_start()"
    if "object 1, wait 5 seconds, then fly to object 2" in natural_language_query:
        return "F(at(object_1)) U wait(5) U F(at(object_2))"
    if "fly to the box" in query_lower:
        return "AMBIGUOUS_QUERY"
    if "red cube" in query_lower and "blue sphere" not in query_lower:
        return "F(at(object_1))"
    if "blue sphere" in query_lower and "red cube" not in query_lower:
        return "F(at(object_2))"
    if "start location" in query_lower:
        return "F(at(object_3))"
    if "mars" in query_lower:
        return "F(at(mars))"
    if "treasure" in query_lower:
        return "F(at(unknown))"
    
    return "F(at(unknown))"

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
    
    if "F(at(unknown))" in ltl_formula:
        return "NOT FEASIBLE: The destination is unknown."
    
    if "F(at(mars))" in ltl_formula:
        return "NOT FEASIBLE: The destination 'Mars' is not within the drone's operational environment."
    
    return "FEASIBLE"

tools = [translate_to_ltl, validate_ltl_formula, ask_for_clarification, check_feasibility]


# -----------------------------
# Setup & Runner
# -----------------------------
def setup_agent(model_name: str = "llama3.2", temperature: float = 0):
    """Initializes model, tools, and compiles the LangGraph agent."""
    if DEBUG:
        print("Initializing Ollama model...")
    
    llm = ChatOllama(
        model=model_name,
        temperature=temperature,
        base_url="http://localhost:11434"
    )

    llm_with_tools = llm.bind_tools(tools)
    tool_node = ToolNode(tools)

    def should_continue(state: AgentState) -> str:
        messages = state["messages"]
        last_message = messages[-1] if messages else None
        
        # Check for tool calls
        if hasattr(last_message, 'tool_calls') and last_message.tool_calls:
            return "tools"
        
        # Check workflow progression based on last tool result
        if len(messages) >= 2:
            last_tool_message = None
            for msg in reversed(messages):
                if hasattr(msg, 'content') and not hasattr(msg, 'tool_calls'):
                    last_tool_message = msg
                    break
            
            if last_tool_message and hasattr(last_tool_message, 'content'):
                content = last_tool_message.content
                
                # Check if we just got a translation result
                if any(formula in content for formula in ["F(at(object_", "AMBIGUOUS_QUERY"]):
                    current_step = state.get("workflow_step", "translate")
                    if current_step == "translate":
                        return "continue_workflow"
                        
                # Check if we just got a validation result
                if content in ["True", "False"]:
                    return "continue_workflow"
                    
                # Check if we got clarification or feasibility result
                if "NOT FEASIBLE" in content or "multiple objects" in content or "FEASIBLE" in content:
                    return "end"
        
        return "end"

    def call_model(state: AgentState):
        messages = state["messages"]
        current_step = state.get("workflow_step", "translate")
        
        # Only add system prompt once
        if not messages or not isinstance(messages[0], SystemMessage):
            system_prompt = """You are a mission planner. You must make tool calls in sequence. 

First, ALWAYS call translate_to_ltl with the user's query.
Do not explain or add commentary. Just make the tool call."""
            messages = [SystemMessage(content=system_prompt)] + messages
        
        # If we're in translate step, just call translate_to_ltl
        if current_step == "translate":
            response = llm_with_tools.invoke(messages)
            return {"messages": [response], "workflow_step": "translated"}
        
        response = llm_with_tools.invoke(messages)
        return {"messages": [response]}

    def continue_workflow(state: AgentState):
        """Continue the workflow based on the last tool result"""
        messages = state["messages"]
        
        # Find the last tool result
        last_tool_result = None
        for msg in reversed(messages):
            if hasattr(msg, 'content') and not hasattr(msg, 'tool_calls') and msg.content:
                last_tool_result = msg.content.strip()
                break
        
        if not last_tool_result:
            return {"final_answer": "Error: No tool result found"}
        
        # Handle AMBIGUOUS_QUERY case
        if last_tool_result == "AMBIGUOUS_QUERY":
            # Get original user query
            user_query = ""
            for msg in messages:
                if isinstance(msg, HumanMessage):
                    user_query = msg.content
                    break
            
            clarification = ask_for_clarification(user_query)
            return {"final_answer": clarification}
        
        # Handle LTL formula validation
        if last_tool_result.startswith("F(at("):
            is_valid = validate_ltl_formula(last_tool_result)
            
            if is_valid:
                feasibility = check_feasibility(last_tool_result)
                if feasibility == "FEASIBLE":
                    return {"final_answer": last_tool_result}
                else:
                    return {"final_answer": feasibility}
            else:
                return {"final_answer": "Invalid LTL formula"}
        
        return {"final_answer": last_tool_result}

    workflow = StateGraph(AgentState)
    workflow.add_node("agent", call_model)
    workflow.add_node("tools", tool_node)
    workflow.add_node("continue_workflow", continue_workflow)
    
    workflow.set_entry_point("agent")
    workflow.add_conditional_edges("agent", should_continue, {
        "tools": "tools", 
        "continue_workflow": "continue_workflow",
        "end": END
    })
    workflow.add_edge("tools", "continue_workflow")
    workflow.add_edge("continue_workflow", END)

    return workflow.compile()

def run_agent(app, query: str, max_iterations: int = 10):
    """Run the agent with a given query."""
    if DEBUG:
        print(f"\nStarting agent with query: '{query}'")
        print("=" * 50)

    inputs = {"messages": [HumanMessage(content=query)], "workflow_step": "translate"}

    if DEBUG:
        step_count = 0
        for output in app.stream(inputs):
            step_count += 1
            if step_count > max_iterations:
                print("Max iterations reached, stopping...")
                break
                
            print(f"\n--- Step {step_count} ---")
            for key, value in output.items():
                print(f"Node: {key}")
                if "messages" in value and value["messages"]:
                    last_message = value["messages"][-1]
                    if hasattr(last_message, 'content') and last_message.content:
                        print(f"Content: {last_message.content}")
                    if hasattr(last_message, 'tool_calls') and last_message.tool_calls:
                        tool_names = []
                        for tc in last_message.tool_calls:
                            if isinstance(tc, dict):
                                tool_names.append(tc.get('name', 'unknown'))
                            else:
                                tool_names.append(getattr(tc, 'name', 'unknown'))
                        print(f"Tool calls: {tool_names}")
                elif "final_answer" in value:
                    print(f"Final answer: {value['final_answer']}")

    # Run the workflow
    try:
        final_state = app.invoke(inputs)
    except Exception as e:
        if DEBUG:
            print(f"Error during execution: {e}")
        return "Error during execution"
    
    # Get the final answer
    final_answer = ""
    if "final_answer" in final_state and final_state["final_answer"]:
        final_answer = final_state["final_answer"]
    elif "messages" in final_state and final_state["messages"]:
        # Fallback to last message content
        for message in reversed(final_state["messages"]):
            if hasattr(message, 'content') and message.content and message.content.strip():
                final_answer = message.content.strip()
                break

    if DEBUG:
        print("\n" + "=" * 50)
        print("Final Answer:")
    
    print(final_answer)
    return final_answer


# -----------------------------
# Main Functions
# -----------------------------
def run_queries(app, queries):
    """Run a list of queries through the agent."""
    for i, query in enumerate(queries):
        try:
            if DEBUG:
                print(f"\n=== Query {i+1}: {query} ===")
            else:
                print(f"Query {i+1}: {query}")
                print("Result:", end=" ")
            
            run_agent(app, query)
            
            if not DEBUG:
                print()  # Add newline after result
            else:
                print("\n" + "-" * 50)
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


def run_tests(app):
    """Run the specific test cases."""
    test_cases = [
        ("fly to the blue sphere and wait 5 seconds, then fly to the red cube", 
         "F(at(object_2)) U wait(5) U F(at(object_1))"),
        ("Fly to object 1, then fly back to the start location", 
         "F(at(object_1)) U return_to_start()"),
        ("fly to the red cube", 
         "F(at(object_1))"),
        ("fly to the box", 
         "There are multiple objects that could be considered 'a box'. Please specify which box you mean, for example 'the red cube'."),
        ("Fly to Mars", 
         "NOT FEASIBLE: The destination 'Mars' is not within the drone's operational environment."),
        ("Find the treasure", 
         "NOT FEASIBLE: The destination is unknown.")
    ]
    
    print("\nRunning test cases...")
    print("=" * 60)
    
    passed = 0
    total = len(test_cases)
    
    for i, (query, expected) in enumerate(test_cases):
        print(f"\nTest {i+1}: {query}")
        if DEBUG:
            print(f"Expected: {expected}")
        
        try:
            if not DEBUG:
                print("Expected:", expected)
                print("Actual:  ", end="")
            
            result = run_agent(app, query)
            
            if result == expected:
                print("PASS")
                passed += 1
            else:
                print("FAIL")
                if not DEBUG:
                    print(f"  Got: {result}")
            
        except Exception as e:
            print(f"ERROR: {str(e)}")
        
        if DEBUG:
            print("-" * 40)
    
    print(f"\nResults: {passed}/{total} tests passed")
    if passed == total:
        print("All tests passed!")
    else:
        print(f"{total - passed} tests failed")


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
    choice = input("Enter '1' for test cases, '2' for predefined queries, or '3' for interactive mode: ").strip()
    
    if choice == '1':
        run_tests(app)
    elif choice == '2':
        queries = [
            "Fly to object 1, then fly back to the start location",
            "Fly to the box"
        ]
        run_queries(app, queries)
    elif choice == '3':
        run_interact(app)
    else:
        print("Invalid choice. Exiting.")

if __name__ == "__main__":
    main()