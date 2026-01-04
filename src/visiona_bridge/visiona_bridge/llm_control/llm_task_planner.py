#!/usr/bin/env python3
"""
LLM Task Planner Node

Decomposes natural language commands into executable task sequences using a
local LLM (Llama 3.2).

Author: Antigravity AI
Date: 2026-01-04
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from pathlib import Path

try:
    from llama_cpp import Llama
    LLAMA_CPP_AVAILABLE = True
except ImportError:
    LLAMA_CPP_AVAILABLE = False


class LLMTaskPlanner(Node):
    """ROS2 node for LLM-based task planning"""
    
    def __init__(self):
        super().__init__('llm_task_planner')
        
        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('language', 'en')
        self.declare_parameter('n_ctx', 2048)
        self.declare_parameter('n_gpu_layers', 35)
        self.declare_parameter('temperature', 0.3)
        self.declare_parameter('max_tokens', 512)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.language = self.get_parameter('language').value
        n_ctx = self.get_parameter('n_ctx').value
        n_gpu_layers = self.get_parameter('n_gpu_layers').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value
        
        # Validate model path
        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(
                f'Model path not found: {model_path}\n'
                'Please run scripts/setup_models.sh first!'
            )
            self.llm = None
        else:
            # Load LLM model
            if not LLAMA_CPP_AVAILABLE:
                self.get_logger().error(
                    'llama-cpp-python not installed! '
                    'Run: pip install llama-cpp-python'
                )
                self.llm = None
            else:
                self.get_logger().info(f'Loading LLM model from {model_path}...')
                try:
                    self.llm = Llama(
                        model_path=model_path,
                        n_ctx=n_ctx,
                        n_gpu_layers=n_gpu_layers,
                        n_threads=4,
                        verbose=False
                    )
                    self.get_logger().info('LLM model loaded successfully!')
                except Exception as e:
                    self.get_logger().error(f'Failed to load LLM: {e}')
                    self.llm = None
        
        # Load prompt templates
        self.prompts = self._load_prompts()
        
        # Subscriber for natural language commands
        self.command_sub = self.create_subscription(
            String,
            '/llm/command',
            self.command_callback,
            10
        )
        
        # Publisher for task sequences
        self.task_pub = self.create_publisher(
            String,
            '/llm/tasks',
            10
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(
            String,
            '/llm/status',
            10
        )
        
        self.get_logger().info(
            f'LLM Task Planner ready (language: {self.language})'
        )
        
    def _load_prompts(self):
        """Load bilingual prompt templates from files"""
        prompts = {}
        package_dir = Path(__file__).parent
        
        for lang in ['en', 'de']:
            prompt_file = package_dir / 'prompts' / f'task_decomposition_{lang}.txt'
            try:
                with open(prompt_file, 'r', encoding='utf-8') as f:
                    prompts[lang] = f.read()
                self.get_logger().info(f'Loaded {lang} prompt template')
            except FileNotFoundError:
                self.get_logger().warn(f'Prompt file not found: {prompt_file}')
                prompts[lang] = self._get_fallback_prompt(lang)
                
        return prompts
    
    def _get_fallback_prompt(self, lang='en'):
        """Fallback prompt if file not found"""
        if lang == 'en':
            return """Decompose the command into tasks. Output JSON only:
{{"tasks": [{{"action": "...", "target": "...", "parameters": {{}}}}]}}

Command: "{command}"
Output:"""
        else:  # German
            return """Zerlege den Befehl in Aufgaben. Nur JSON ausgeben:
{{"tasks": [{{"action": "...", "target": "...", "parameters": {{}}}}]}}

Befehl: "{command}"
Ausgabe:"""
    
    def command_callback(self, msg):
        """Process incoming natural language command"""
        command = msg.data
        self.get_logger().info(f'='*60)
        self.get_logger().info(f'📥 RECEIVED COMMAND: "{command}"')
        self.get_logger().info(f'='*60)
        
        # Publish status
        status_msg = String()
        status_msg.data = f'Planning: {command}'
        self.status_pub.publish(status_msg)
        
        # Generate task sequence
        if self.llm is None:
            self.get_logger().error('❌ LLM not loaded, cannot plan tasks')
            self.get_logger().error('   Did you run scripts/setup_models.sh?')
            return
            
        try:
            self.get_logger().info('🤖 Calling LLM to decompose command...')
            tasks = self.plan_tasks(command)
            
            self.get_logger().info(f'✅ LLM generated {len(tasks)} tasks:')
            for i, task in enumerate(tasks):
                self.get_logger().info(f'   [{i+1}] {task["action"]} -> {task.get("target", "N/A")}')
            
            # Publish task sequence
            task_msg = String()
            task_msg.data = json.dumps(tasks)
            self.task_pub.publish(task_msg)
            self.get_logger().info(f'📤 Published tasks to /llm/tasks')
            
            # Update status
            status_msg.data = f'Ready ({len(tasks)} tasks planned)'
            self.status_pub.publish(status_msg)
            self.get_logger().info(f'='*60)
            
        except Exception as e:
            self.get_logger().error(f'❌ Task planning failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            status_msg.data = f'Error: {str(e)}'
            self.status_pub.publish(status_msg)
    
    def plan_tasks(self, command: str):
        """
        Use LLM to decompose command into task sequence
        
        Args:
            command: Natural language command string
            
        Returns:
            List of task dictionaries
        """
        # Build prompt        prompt = self.prompts[self.language].format(command=command)
        
        self.get_logger().info(f'🔄 Starting LLM inference (this may take 10-15s on first run)...')
        
        # Generate response
        result = self.llm(
            prompt,
            max_tokens=self.max_tokens,
            temperature=self.temperature,
            stop=["</s>", "\n\n\n"],
            echo=False
        )
        
        output_text = result['choices'][0]['text'].strip()
        self.get_logger().info(f'📝 LLM raw output: {output_text[:200]}...')
        
        # Parse JSON
        try:
            # Extract JSON from output (in case there's extra text)
            import re
            json_match = re.search(r'\{.*\}', output_text, re.DOTALL)
            if json_match:
                output_text = json_match.group()
                
            task_data = json.loads(output_text)
            
            if 'tasks' in task_data:
                self.get_logger().info(f'✅ Successfully parsed {len(task_data["tasks"])} tasks from LLM')
                return task_data['tasks']
            else:
                self.get_logger().warn('⚠️  No tasks field in LLM output')
                return []
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ Failed to parse LLM output as JSON: {e}')
            self.get_logger().error(f'   Output was: {output_text}')
            return []


def main(args=None):
    rclpy.init(args=args)
    
    node = LLMTaskPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
