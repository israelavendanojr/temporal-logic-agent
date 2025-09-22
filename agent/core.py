import operator
from typing import TypedDict, Annotated, Sequence
from langchain_core.messages import BaseMessage, HumanMessage
from langgraph.prebuilt import ToolNode
from langgraph.graph import StateGraph, END
from .tools import (
    translate_to_ltl,
    sanitize_ltl_formula,
    validate_ltl_formula,
    check_feasibility,
    ask_for_clarification
)

# -----------------------------
# Agent State Definition
# -----------------------------
class AgentState(TypedDict):
    """Represents the state of our agent."""
    messages: Annotated[Sequence[BaseMessage], operator.add]
    ltl_formula: str

# -----------------------------
# LangGraph Node Functions
# -----------------------------
def translate_node(state: AgentState):
    """Translates the user's query into an intermediate format."""
    query = state['messages'][-1].content
    ltl = translate_to_ltl.invoke(query)
    return {"messages": [HumanMessage(content=ltl)], "ltl_formula": ltl}

def sanitize_node(state: AgentState):
    """Sanitizes the LTL formula string."""
    ltl = state['ltl_formula']
    sanitized_ltl = sanitize_ltl_formula.invoke(ltl)
    return {"messages": [HumanMessage(content=sanitized_ltl)], "ltl_formula": sanitized_ltl}

def validate_node(state: AgentState):
    """Verifies the LTL formula is syntactically valid."""
    ltl = state['ltl_formula']
    is_valid = validate_ltl_formula.invoke(ltl)
    return {"messages": [HumanMessage(content=str(is_valid))]}
    
def check_feasibility_node(state: AgentState):
    """Checks the feasibility of the LTL formula."""
    ltl = state['ltl_formula']
    feasibility = check_feasibility.invoke(ltl)
    return {"messages": [HumanMessage(content=feasibility)]}
    
def clarify_node(state: AgentState):
    """Asks the user for clarification."""
    query = state['messages'][0].content
    clarification_msg = ask_for_clarification.invoke(query)
    return {"messages": [HumanMessage(content=clarification_msg)]}

def format_final_answer(state: AgentState):
    """Formats the final answer based on the feasibility and validation checks."""
    last_message = state['messages'][-1].content
    
    if last_message == "FEASIBLE":
        return {"messages": [HumanMessage(content=state['ltl_formula'])]}
    elif last_message == "NOT FEASIBLE":
        return {"messages": [HumanMessage(content="I'm sorry, that mission is not feasible.")]}
    elif last_message == "False":
        return {"messages": [HumanMessage(content="I'm sorry, I couldn't translate that request into a valid mission.")]}
    
    return {"messages": [HumanMessage(content=last_message)]}

# -----------------------------
# Graph Router
# -----------------------------
def router(state: AgentState) -> str:
    """Determines the next step based on the translation result."""
    last_message_content = state['messages'][-1].content
    
    if last_message_content == "AMBIGUOUS_QUERY":
        return "clarify"
    
    # Check for known logical fails before attempting validation
    if "F(at(unknown))" in last_message_content:
        return "check_feasibility"
        
    # All other cases go to the new sanitize node
    return "sanitize"

# -----------------------------
# Graph Compilation
# -----------------------------
def get_compiled_graph():
    """Compiles and returns the LangGraph application."""
    workflow = StateGraph(AgentState)
    
    workflow.add_node("translate", translate_node)
    workflow.add_node("sanitize", sanitize_node)
    workflow.add_node("validate", validate_node)
    workflow.add_node("check_feasibility", check_feasibility_node)
    workflow.add_node("clarify", clarify_node)
    workflow.add_node("final_answer", format_final_answer)

    workflow.set_entry_point("translate")
    
    # After translation, route based on the LLM's raw output
    workflow.add_conditional_edges("translate", router, {
        "clarify": "clarify",
        "sanitize": "sanitize",
        "check_feasibility": "check_feasibility"
    })
    
    # After sanitization, proceed to validation
    workflow.add_edge("sanitize", "validate")
    
    # After validation, route based on the result
    workflow.add_conditional_edges("validate", lambda x: "check_feasibility" if x['messages'][-1].content == "True" else "final_answer", {
        "check_feasibility": "check_feasibility",
        "final_answer": "final_answer"
    })
    
    # All final outputs from validation, feasibility, and clarification should go to a final answer or END
    workflow.add_edge("check_feasibility", "final_answer")
    workflow.add_edge("clarify", END)
    workflow.add_edge("final_answer", END)

    return workflow.compile()