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

@tool
def translate_with_llm(natural_language_query: str) -> str:
    """Helper function to perform the actual LLM-based translation."""
    llm = ChatOllama(model="llama3.2", temperature=0, base_url="http://localhost:11434")

    known_objects = ", ".join(MOCK_OBJECTS.keys())
    
    system_prompt = (
        "You are a highly reliable translator of Crazyflie drone missions into Linear Temporal Logic (LTL). "
        "Your mission is to output ONLY the LTL formula for a given command. No other text.\n"
        "\n"
        "--- Translation Process ---\n"
        "1. **Analyze Intent:** First, determine the user's intent. Identify the sequence of actions, if any.\n"
        "2. **Translate to LTL:** Convert each action into its corresponding LTL formula fragment.\n"
        "3. **Combine Fragments:** For multi-step commands, connect the LTL fragments using the 'U' operator.\n"
        "4. **Final Output:** Provide only the final, complete LTL formula.\n"
        "\n"
        "--- Language Rules & Examples ---\n"
        "- **Absolute Movement:** A single action to fly to a location. Uses `F(at(object))`. \n"
        "  User: Fly to X \n"
        "  Output: F(at(X))\n"
        "- **Relative Movement:** A single action to move in a cardinal direction. Uses `move(direction, distance)`. \n"
        "  Valid directions: `forward`, `backward`, `up`, `down`. \n"
        "  User: Go forward 10 meters \n"
        "  Output: move(forward, 10)\n"
        "- **Temporal Wait:** A single action to pause. Uses `wait(n)`. \n"
        "  User: Wait 5 seconds \n"
        "  Output: wait(5)\n"
        "- **Return to Start:** A single action to return home. Uses `return_to_start()`. \n"
        "  User: Return to the start \n"
        "  Output: return_to_start()\n"
        "- **Complex Sequence:** A combination of the above actions. \n"
        "  User: Fly to Y, then wait 5 seconds, then go back to the start. \n"
        "  Output: F(at(Y)) U wait(5) U return_to_start()\n"
        "\n"
        "--- Edge Case Handling ---\n"
        "- **Impossible Location:** If the user asks to go to an unknown or impossible place, output: `F(at(unknown))`. \n"
        "  User: Fly to the moon \n"
        "  Output: F(at(unknown))\n"
        "- **Impossible Action:** If the command is an unknown or physically impossible action (e.g., greetings, abstract concepts), output: `AMBIGUOUS_QUERY`. \n"
        "  User: Do a backflip \n"
        "  Output: AMBIGUOUS_QUERY\n"
    )

    response = llm.invoke([
        SystemMessage(content=system_prompt),
        HumanMessage(content=natural_language_query)
    ])
    return response.content.strip()

@tool
def translate_to_ltl(natural_language_query: str) -> str:
    """Translates a natural language command into a complete LTL formula."""
    return translate_with_llm.invoke(natural_language_query)

@tool
def sanitize_ltl_formula(ltl_formula: str) -> str:
    """Cleans up and normalizes the LTL formula string for consistent validation."""
    cleaned = ltl_formula.replace(" ", "")
    cleaned = cleaned.replace("U", " U ")
    return " ".join(cleaned.split())

@tool
def validate_ltl_formula(ltl_formula: str) -> bool:
    """
    Verifies that an LTL formula is syntactically valid and grounded in the environment.
    """
    # Use a more robust approach with multiple regex patterns
    
    # Pattern for single LTL fragments
    fragment_pattern = re.compile(
        r"^(F\(at\((?:X|Y|Z|unknown)\)\)|"
        r"move\((?:forward|backward|up|down),\d+\)|"
        r"wait\(\d+\)|"
        r"return_to_start\(\))$"
    )
    
    # Pattern for sequences with 'U'
    sequence_pattern = re.compile(
        r"^(F\(at\((?:X|Y|Z|unknown)\)\s*U\s*)*"
        r"(move\((?:forward|backward|up|down),\d+\)\s*U\s*)*"
        r"(wait\(\d+\)\s*U\s*)*"
        r"(return_to_start\(\))?$"
    )
    
    # Check if the formula matches either a single fragment or a valid sequence
    if fragment_pattern.match(ltl_formula):
        return True
        
    # Split the formula by the 'U' operator and check each part
    parts = ltl_formula.split(' U ')
    
    if len(parts) > 1:
        for part in parts:
            if not fragment_pattern.match(part):
                return False
        return True
        
    return False

@tool
def check_feasibility(ltl_formula: str) -> str:
    """Checks if the LTL formula is physically possible to execute."""
    if "F(at(unknown))" in ltl_formula:
        return "NOT FEASIBLE"
    
    known_objects = MOCK_OBJECTS.keys()
    
    objects_in_formula = re.findall(r"at\((.*?)\)", ltl_formula)
    
    for obj in objects_in_formula:
        if obj not in known_objects and obj != "unknown":
            return "NOT FEASIBLE: Contains unknown objects."
            
    return "FEASIBLE"

@tool
def ask_for_clarification(ambiguous_query: str) -> str:
    """Asks the user for clarification on an ambiguous query."""
    return "I need more information to process your request. Can you please clarify?"