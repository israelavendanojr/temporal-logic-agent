# UAV Task Planner Agent

An intelligent agent that translates natural language commands into Linear Temporal Logic (LTL) formulas for autonomous drone mission planning. The system uses a fine-tuned language model to understand human instructions and convert them into formal temporal logic expressions that can be safely executed by the Crazyflie 2.0 UAV.

## Project Overview

**Key Technologies:**
- **Python 3.12** - Core programming language
- **LangGraph & LangChain** - Multi-agent workflow orchestration and LLM integration
- **GGUF/llama-cpp-python** - Local language model inference for LTL translation
- **Linear Temporal Logic (LTL)** - Formal specification language for temporal behaviors
- **PyYAML** - Configuration management for environment and safety parameters

## Directory Overview

The project follows a clean layered architecture pattern with clear separation of concerns between domain logic, business services, and application integration. The structure separates core business concepts from infrastructure concerns, making the system highly testable and maintainable.

**Structure:**
- **`/agent/`** - Core LangGraph workflow orchestration and legacy tool implementations
  - Contains the main state machine (`core.py`), model server integration (`model_server.py`), and LTL processing modules (`ltl/`)
- **`/domain/`** - Immutable business domain models representing core concepts
  - Defines `LTLFormula` and `MissionContext` as value objects with validation states
- **`/services/`** - Business logic services for translation, validation, and feasibility checking
  - Implements the core workflow: NLP→LTL translation, syntax validation, safety injection, and mission feasibility
- **`/application/`** - Integration layer connecting services with LangChain tools and dependency injection
  - Provides factory pattern for service creation and context builders for session state management
- **`/config/`** - Environment configuration including flight zones, waypoints, obstacles, and safety parameters
- **`/models/`** - Fine-tuned GGUF model for LTL translation and training dataset
- **`/main.py`** - Application entry point with CLI interface for interactive and batch processing modes

## Validation Pipeline

The system uses a multi-stage validation pipeline to ensure LTL correctness:

1. **Syntax Validation** (`LTLParser`) - Checks grammar, balanced parentheses, valid operators
2. **Semantic Validation** (`SemanticValidator`) - Checks logical consistency, detects impossible constraints
3. **Feasibility Checking** - Verifies waypoints exist, altitude bounds respected

### Semantic Rules

The semantic validator enforces:
- `clear_of(X)` = "maintain distance from X" (positive assertion)
- `!clear_of(X)` = "do NOT maintain distance" = collision (always invalid for obstacles)
- No conflicting spatial constraints (e.g., `G(at(A)) & G(at(B))`)
- No stationary + moving conflicts