# Crazyflie Mission Planner Agent

An AI agent that translates NLP to LTL for a Crazyflie drone.
-----

## Core Concepts

  * **Natural Language Processing (NLP)**: The AI's ability to understand your English commands, such as "fly to X" or "wait 5 seconds."
  * **Linear Temporal Logic (LTL)**: A formal language used to define drone missions. For example, the command `F(at(Y))` means "Eventually, be at location Y."

-----

## Usage

This project uses **Ollama** to run a local large language model.

### Ollama Setup

1.  **Install Ollama**: Download and install Ollama from `ollama.ai`.
2.  **Pull the Model**: Open your terminal and download the required model with the command: `ollama pull llama3.2`.

### Running the Agent

1.  Ensure your Ollama server is running and the `llama3.2` model is available.
2.  Open your terminal, navigate to the project directory, and run the main script.
    ```bash
    python main.py
    ```
