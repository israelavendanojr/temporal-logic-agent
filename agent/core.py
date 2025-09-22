import operator
from typing import TypedDict, Annotated, Sequence
from langchain_core.messages import BaseMessage, HumanMessage
from langgraph.prebuilt import ToolNode
from langgraph.graph import StateGraph, END
from .tools import (
    translate_to_ltl,
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
    """Translates the user's query into an LTL formula."""
    query = state['messages'][-1].content
    ltl = translate_to_ltl(query)
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
        
# -----------------------------
# Graph Router
# -----------------------------
def router(state: AgentState) -> str:
    """Determines the next step based on the LTL formula and validation results."""
    last_message_content = state['messages'][-1].content
    
    if state.get('ltl_formula') and "AMBIGUOUS_QUERY" in state['ltl_formula']:
        return "clarify"
    elif state.get('ltl_formula') and "F(at(unknown))" in state['ltl_formula']:
        return "check_feasibility"
    elif state.get('ltl_formula') and last_message_content == state['ltl_formula']:
        return "validate"
        
    if last_message_content == "True":
        return "check_feasibility"
    if last_message_content == "False":
        return "end"

    if "FEASIBLE" in last_message_content or "NOT FEASIBLE" in last_message_content:
        return "final_answer"
    
    return "end"

# -----------------------------
# Graph Compilation
# -----------------------------
def get_compiled_graph():
    """Compiles and returns the LangGraph application."""
    workflow = StateGraph(AgentState)
    
    workflow.add_node("translate", translate_node)
    workflow.add_node("validate", validate_node)
    workflow.add_node("check_feasibility", check_feasibility_node)
    workflow.add_node("clarify", clarify_node)
    workflow.add_node("final_answer", format_final_answer)

    workflow.set_entry_point("translate")
    
    workflow.add_conditional_edges("translate", router, {
        "validate": "validate",
        "check_feasibility": "check_feasibility",
        "clarify": "clarify",
        "end": END
    })
    
    workflow.add_conditional_edges("validate", router, {
        "check_feasibility": "check_feasibility",
        "end": "final_answer"
    })
    
    workflow.add_edge("check_feasibility", "final_answer")
    workflow.add_edge("clarify", END)
    workflow.add_edge("final_answer", END)

    return workflow.compile()