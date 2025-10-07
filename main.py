"""
main.py: UAV Task Planner - LTL Translation Agent
Integrates the comprehensive test suite as the default execution mode.
"""
import argparse
import logging
import time
from typing import List, Tuple

from langchain_core.messages import HumanMessage
from agent.core import get_compiled_graph
from agent.config_loader import get_config

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


# ==============================================================================
# CORE AGENT FUNCTIONS
# ==============================================================================

def run_agent(app, query: str, session_state: dict = None):
    """Run agent with session state tracking"""
    if app is None:
        return "ERROR: Agent graph not compiled.", session_state

    # Initialize session state on first run
    if session_state is None:
        config = get_config()
        drone_state = config.get_drone_state()
        session_state = {
            'conversation_log': [],
            'spatial_memory': {
                'current_position': tuple(drone_state['start_position']),
                'start_position': tuple(drone_state['start_position']),
                'objects': config.get_waypoints(),
                'flight_zone': config.get_flight_zone(),
                'obstacles': config.get_obstacles()
            }
        }
    
    # Prepare inputs for LangGraph
    inputs = {
        "messages": [HumanMessage(content=query)],
        "conversation_log": session_state['conversation_log'],
        "spatial_memory": session_state['spatial_memory']
    }
    
    # Execute LangGraph workflow
    final_answer = ""
    final_state = None
    for output in app.stream(inputs):
        for key, value in output.items():
            messages = value.get("messages", [])
            if messages and hasattr(messages[-1], 'content'):
                final_answer = messages[-1].content
            final_state = value
    
    # Update session state from results
    if final_state and 'conversation_log' in final_state:
        session_state['conversation_log'] = final_state['conversation_log']
        session_state['spatial_memory'] = final_state['spatial_memory']
    
    return final_answer, session_state

# ==============================================================================
# COMPREHENSIVE TEST SUITE (INTEGRATED)
# ==============================================================================

# Test queries as provided by the user
BASIC_QUERIES = [
    "fly to waypoint_a", 
    "go to waypoint_b", 
    "navigate to waypoint_c",
    "go to waypoint_a then waypoint_b", 
    "fly to waypoint_a then waypoint_b then waypoint_c",
    "stay above 2 meters", 
    "maintain altitude above 1.5 meters",
    "maintain altitude above 2m until reaching waypoint_a", 
    "keep moving until you reach waypoint_c",
    "go to waypoint_a while avoiding obstacle_1", 
    "fly to waypoint_b but never enter restricted_zone",
    "hover for 10 seconds", 
    "scan area_1",
]

NOVEL_QUERIES = [
    "make sure you don't go near obstacle_1 while heading to waypoint_c",
    "I need you to get to waypoint_b as quickly as possible",
    "can you check out area_1 without getting too close to obstacle_2?",
    "scan area_1 unless battery is below 30 percent",
    "return home if wind speed gets dangerous",
    "position yourself 2 meters above waypoint_c",
    "spend no more than 5 seconds at each waypoint",
    "hover at waypoint_a until I tell you to move",
    "visit all waypoints except waypoint_c",
    "never go below 1 meter or above 4 meters",
    "go there", 
    "scan that area over there",
    "just hover in place", 
    "go to waypoint_a then immediately come back",
    "emergency landing right now", 
    "abort mission and return to base immediately",
]

def run_test_category(app, queries: List[str], category_name: str) -> Tuple[int, int, List[str]]:
    """Run a category of test queries and report results"""
    print(f"\n{'='*80}")
    print(f"Testing: {category_name}")
    print(f"{'='*80}")
    
    passed = 0
    failed = 0
    failures = []
    session_state = None
    total_time = 0.0
    
    for i, query in enumerate(queries, 1):
        print(f"\n[{i}/{len(queries)}] Query: {query}")
        try:
            start_time = time.time()
            result, session_state = run_agent(app, query, session_state)
            elapsed = time.time() - start_time
            total_time += elapsed
            
            # Basic validation
            if result.startswith("INVALID_") or result.startswith("ERROR:"):
                print(f"FAILED: {result} ({elapsed:.2f}s)")
                failed += 1
                failures.append(f"{query} -> {result}")
            elif "I'm sorry" in result and "not feasible" in result:
                print(f"NOT FEASIBLE: {result} ({elapsed:.2f}s)")
                passed += 1  # Still counts as handling correctly
            elif not result or result.strip() == "":
                print(f"FAILED: Empty result ({elapsed:.2f}s)")
                failed += 1
                failures.append(f"{query} -> Empty result")
            else:
                print(f"PASSED: {result} ({elapsed:.2f}s)")
                passed += 1

        except Exception as e:
            print(f"EXCEPTION: {str(e)}")
            failed += 1
            failures.append(f"{query} -> EXCEPTION: {str(e)}")
    
    avg_time = total_time / len(queries) if queries else 0
    print(f"\n{category_name} Results: {passed}/{len(queries)} passed (avg {avg_time:.2f}s/query)")
    return passed, failed, failures

def run_full_test_suite():
    """Run complete test suite with reporting"""
    print("\n" + "="*80)
    print("UAV TASK PLANNER - COMPREHENSIVE TEST SUITE")
    print("="*80)
    
    try:
        print("\nCompiling agent graph...")
        app = get_compiled_graph()
        print("Agent graph compiled successfully\n")
    except Exception as e:
        print(f"Failed to compile agent graph: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    # Test categories
    all_results = []
    all_failures = []
    
    # 1. Basic queries (should all pass)
    passed, failed, failures = run_test_category(app, BASIC_QUERIES, "BASIC QUERIES")
    all_results.append(("Basic", passed, failed))
    all_failures.extend(failures)
    
    # 2. Novel queries (generalization test)
    passed, failed, failures = run_test_category(app, NOVEL_QUERIES, "NOVEL QUERIES")
    all_results.append(("Novel", passed, failed))
    all_failures.extend(failures)
    
    # Final report
    print("\n" + "="*80)
    print("FINAL REPORT")
    print("="*80)
    
    total_passed = sum(r[1] for r in all_results)
    total_failed = sum(r[2] for r in all_results)
    total_tests = total_passed + total_failed
    
    for category, passed, failed in all_results:
        success_rate = (passed / (passed + failed) * 100) if (passed + failed) > 0 else 0
        print(f"{category:12} {passed:3}/{passed+failed:3} ({success_rate:.1f}%)")
    
    final_success_rate = (total_passed / total_tests * 100) if total_tests > 0 else 0
    print(f"\n{'Total':12} {total_passed:3}/{total_tests:3} ({final_success_rate:.1f}%)")
    
    if all_failures:
        print("\n" + "="*80)
        print("FAILURES")
        print("="*80)
        for failure in all_failures:
            print(f"  - {failure}")
    
    # Readiness assessment
    print("\n" + "="*80)
    print("READINESS ASSESSMENT")
    print("="*80)
    
    if final_success_rate >= 90:
        print("READY FOR ROS2 INTEGRATION")
        print("   - High success rate on both basic and novel queries")
        print("   - Model generalizes well beyond training data")
    elif final_success_rate >= 75:
        print("NEEDS REFINEMENT")
        print("   - Acceptable performance but room for improvement")
        print("   - Consider additional fine-tuning on failure cases")
    else:
        print("NOT READY")
        print("   - Significant failures detected")
        print("   - Requires model retraining or architecture changes")
    
    return final_success_rate >= 75

# ==============================================================================
# MAIN ENTRY POINT
# ==============================================================================

def run_interactive(app):
    """Interactive mode with persistent session"""
    print("\nInteractive mode - type 'quit' to exit")
    session_state = None
    
    while True:
        try:
            query = input("\nQuery: ").strip()
            if query.lower() in ['quit', 'exit', 'q']:
                print("Goodbye!")
                break
            if query:
                # Print the final result in interactive mode
                result, session_state = run_agent(app, query, session_state)
                print(f"\nResult: {result}")
        except KeyboardInterrupt:
            print("\nGoodbye!")
            break

def main():
    parser = argparse.ArgumentParser(description='UAV Task Planner - LTL Translation Agent')
    parser.add_argument('--interactive', action='store_true', help='Run in interactive mode')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Run mode selection
    if args.interactive:
        print("\nCompiling agent graph...")
        app = get_compiled_graph()
        print("Agent graph compiled successfully")
        run_interactive(app)
    else:
        # Default mode is now the comprehensive test suite
        is_ready = run_full_test_suite()
        exit(0 if is_ready else 1)

if __name__ == "__main__":
    main()