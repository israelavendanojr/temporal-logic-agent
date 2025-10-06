import json
import re
from collections import defaultdict

def validate_dataset(file_path='models/ltl_training_dataset.jsonl'):
    errors = []
    warnings = []
    stats = defaultdict(int)
    
    with open(file_path, 'r') as f:
        for i, line in enumerate(f, 1):
            data = json.loads(line)
            instruction = data['instruction'].lower()
            output = data['output']
            
            # Check 1: No inverted obstacle/perimeter avoidance
            if re.search(r'avoid|staying clear|staying away|keep.*distance', instruction):
                if '!clear_of(obstacle' in output or '!clear_of(perimeter' in output:
                    errors.append(f"Line {i}: Inverted negation for avoidance pattern")
            
            # Check 2: Balanced parentheses
            if output.count('(') != output.count(')'):
                errors.append(f"Line {i}: Unbalanced parentheses")
            
            # Check 3: Case consistency (lowercase waypoints)
            if re.search(r'waypoint_[A-Z]', output):
                errors.append(f"Line {i}: Uppercase waypoint in output")
            
            # Statistics
            if ' & ' in output: stats['conjunction'] += 1
            if 'F(' in output: stats['eventually'] += 1
            if 'G(' in output: stats['always'] += 1
            if 'U ' in output: stats['until'] += 1
            if 'X(' in output: stats['next'] += 1
            
    print(f"\n=== Dataset Validation Report ===")
    print(f"Total examples: {i}")
    print(f"Errors found: {len(errors)}")
    print(f"Warnings: {len(warnings)}")
    print(f"\nOperator Distribution:")
    for op, count in sorted(stats.items()):
        print(f"  {op}: {count}")
    
    if errors:
        print(f"\n=== ERRORS ===")
        for error in errors[:10]:  # Show first 10
            print(f"  {error}")
    
    return len(errors) == 0

if __name__ == "__main__":
    validate_dataset()
