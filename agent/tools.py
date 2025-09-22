from typing import TypedDict, Annotated, Sequence
from langchain_core.tools import tool
from langchain_core.messages import BaseMessage, HumanMessage, SystemMessage
from langchain_ollama import ChatOllama

# MOCK DATA for a stateful environment
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
        "F(at(unknown))",
        "F(at(object_2)) U F(at(object_1))",
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