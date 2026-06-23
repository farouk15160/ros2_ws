"""
LLM Layer Package – Visiona Jarvis Pipeline.

Modules:
    llm_planner_node      – Local LLM (Ollama) receives command + world state,
                            outputs structured JSON action plan.
    action_executor_node  – Translates action plan into ROS2 commands.
"""
