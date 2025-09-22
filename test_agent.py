"""
Improved LangGraph ReAct Agent with Ollama
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


# -----------------------------
# Tools
# -----------------------------
@tool
def search_the_web(query: str) -> str:
    """Searches the internet for information on a given query."""
    print(f"Executing Search Tool for query: '{query}'")
    return f"Search results for '{query}': Found relevant information about the topic."


@tool
def calculate(expression: str) -> str:
    """Performs basic mathematical calculations."""
    print(f"Executing Calculator Tool for expression: '{expression}'")
    try:
        result = eval(expression.replace("^", "**"))
        return f"The result of {expression} is {result}"
    except Exception:
        return f"Could not calculate {expression}. Please check the expression format."


tools = [search_the_web, calculate]


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
        response = llm_with_tools.invoke(messages)
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
        "Tell me what 69 - 2 is equal to and then tell me what the sum of each digit in that answer is"
    ]

    run_queries(app, queries)
    run_interact(app)
