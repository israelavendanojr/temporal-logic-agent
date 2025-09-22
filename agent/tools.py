import re
from typing import TypedDict, Annotated, Sequence
from langchain_core.tools import tool
from langchain_core.messages import BaseMessage, HumanMessage, SystemMessage
from langchain_ollama import ChatOllama

# MOCK DATA for a stateful environment
MOCK_OBJECTS = {
    "X": (1, 2, 3),
    "Y": (4, 5, 6),
    "Z": (0, 0, 0)
}
MOCK_CRAZYFLIE_LOCATION = (-0.1, 0.2, 0.1)

def translate_with_llm(natural_language_query: str) -> str:
    """Helper function to perform the actual LLM-based translation."""
    llm = ChatOllama(model="llama3.2", temperature=0, base_url="http://localhost:11434")

    known_objects = ", ".join(MOCK_OBJECTS.keys())
    current_location = str(MOCK_CRAZYFLIE_LOCATION) # Dynamically add drone location

    system_prompt = (
        "You are a master translator of Crazyflie drone missions into Linear Temporal Logic (LTL).\n"
        "Your mission is to output ONLY the LTL formula for a given command. No other text.\n"
        "\n"
        "--- Language Rules ---\n"
        "- Use these operators: F(at(object)), U (until), wait(n), move(direction, distance), return_to_start().\n"
        "- Valid directions: forward, backward, up, down.\n"
        "\n"
        "--- Known Information ---\n"
        f"- Known objects and their IDs: {known_objects}.\n"
        f"- The drone's current location is: {current_location}.\n"
        "\n"
        "--- Edge Cases ---\n"
        "- If a request is ambiguous, a greeting, or an unknown action, output: AMBIGUOUS_QUERY.\n"
        "- If a request is impossible or refers to an unknown object, output: F(at(unknown)).\n"
        "\n"
        "--- Translation Examples ---\n"
        "User: fly to X\n"
        "Output: F(at(X))\n"
        "\n"
        "User: wait 5 seconds\n"
        "Output: wait(5)\n"
        "\n"
        "User: fly forward 10 meters\n"
        "Output: move(forward, 10)\n"
        "\n"
        "User: fly back to the start\n"
        "Output: return_to_start()\n"
        "\n"
        "User: fly to Y then wait 5 seconds then fly to Z\n"
        "Output: F(at(Y)) U wait(5) U F(at(Z))\n"
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
    """
    Verifies that an LTL formula is syntactically valid and grounded in the environment.
    This tool now uses a dynamic regex pattern instead of a static list.
    """
    # Regex to match common LTL patterns
    # F(at(obj)) U F(at(obj))
    # F(at(obj))
    # F(at(obj)) U wait(n) U F(at(obj))
    # wait(n)
    # P(at(obj))
    # return_to_start()
    
    # Define valid objects and directions
    valid_objects = "|".join(re.escape(obj) for obj in MOCK_OBJECTS.keys())
    valid_directions = "forward|backward|up|down"

    # Define the core patterns
    at_pattern = f"F\\(at\\((?:{valid_objects}|unknown|mars)\\)\\)"
    move_pattern = f"move\\((?:{valid_directions}), \\d+\\)"
    wait_pattern = "wait\\(\\d+\\)"
    return_pattern = "return_to_start\\(\\)"
    general_pattern = f"^(?:{at_pattern}|{move_pattern}|{wait_pattern}|{return_pattern})(?: U (?:{at_pattern}|{move_pattern}|{wait_pattern}|{return_pattern}))*$"

    # The issue was that the `move_pattern` and `return_pattern`
    # were not included in the final `valid_pattern` regex.
    # The corrected pattern combines all valid command types.
    return bool(re.match(general_pattern, ltl_formula))

@tool
def ask_for_clarification(ambiguous_query: str) -> str:
    """Asks the user for clarification on an ambiguous query."""
    return "I need more information to process your request. Can you please clarify?"

@tool
def check_feasibility(ltl_formula: str) -> str:
    """Checks if the LTL formula is physically possible to execute."""
    # Check for impossible objects based on the LTL formula
    if "F(at(unknown))" in ltl_formula or "F(at(mars))" in ltl_formula:
        return "NOT FEASIBLE"
    
    # Check if all objects in the formula are known
    known_objects = MOCK_OBJECTS.keys()
    
    # Use regex to find all "at(object)" instances
    objects_in_formula = re.findall(r"at\((.*?)\)", ltl_formula)
    
    for obj in objects_in_formula:
        if obj not in known_objects:
            return "NOT FEASIBLE: Contains unknown objects."
            
    return "FEASIBLE"