#!/bin/bash
# Wrapper script to run LLM task planner with the correct Python environment

# Activate the virtual environment where llama-cpp-python is installed
source /home/farouk/.venv/llm/bin/activate

# Run the actual node
exec python3 /home/farouk/ros2_ws/install/visiona_bridge/lib/visiona_bridge/llm_task_planner "$@"
