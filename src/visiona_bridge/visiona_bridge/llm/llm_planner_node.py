#!/usr/bin/env python3
import json, os, rclpy
from rclpy.node import Node
from std_msgs.msg import String
try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    HAS_REQUESTS = False

PROMPT_PATH = os.path.join(os.path.dirname(__file__), "prompt_templates", "system_prompt.txt")

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__("jarvis_llm_planner")
        self.declare_parameter("ollama_url", "http://localhost:11434")
        self.declare_parameter("planner_model", "mistral")
        self.declare_parameter("request_timeout", 120)
        self.declare_parameter("temperature", 0.1)
        self.ollama_url = self.get_parameter("ollama_url").value
        self.planner_model = self.get_parameter("planner_model").value
        self.req_timeout = self.get_parameter("request_timeout").value
        self.temperature = self.get_parameter("temperature").value
        self.world_state = "{}"
        self.is_thinking = False
        try:
            with open(PROMPT_PATH) as f: self.prompt_template = f.read()
            self.get_logger().info(f"Loaded prompt: {PROMPT_PATH}")
        except FileNotFoundError:
            self.prompt_template = 'Reply ONLY with JSON: {"reasoning":"...","actions":[...]}\nWorld: {WORLD_MODEL}\nCommand: {COMMAND}'
            self.get_logger().warn("Using fallback prompt.")
        self.plan_pub = self.create_publisher(String, "/jarvis/action_plan", 10)
        self.status_pub = self.create_publisher(String, "/jarvis/llm_status", 10)
        self.feedback_pub = self.create_publisher(String, "/jarvis/feedback", 10)
        self.create_subscription(String, "/jarvis/command", self._on_command, 10)
        self.create_subscription(String, "/jarvis/world_state", self._on_world_state, 10)
        if not HAS_REQUESTS:
            self.get_logger().error("Install: pip3 install requests")
        else:
            self._pt = self.create_timer(0.5, self._preload_once)
        self.get_logger().info(f"\n{'='*50}\n  JARVIS LLM Planner\n  Model  : {self.planner_model}\n  Server : {self.ollama_url}\n{'='*50}")

    def _preload_once(self):
        self._pt.cancel()
        self.get_logger().info(f"Preloading {self.planner_model} (may take 30-60s)...")
        self._pub_status("loading")
        try:
            r = requests.post(f"{self.ollama_url}/api/generate",
                json={"model": self.planner_model, "prompt": "say ready", "stream": False, "options": {"num_predict": 3}},
                timeout=120)
            r.raise_for_status()
            self.get_logger().info(f"{self.planner_model} ready.")
            self.get_logger().info("JARVIS waiting for commands...")
            self._pub_status("ready")
            self._pub_feedback("JARVIS online. How can I help?")
        except requests.exceptions.ConnectionError:
            self.get_logger().error(f"Cannot connect to Ollama. Start: ollama serve && ollama pull {self.planner_model}")
            self._pub_status("error: ollama offline")
        except Exception as e:
            self.get_logger().error(f"Preload: {e}")
            self._pub_status(f"error: {e}")

    def _on_world_state(self, msg: String):
        self.world_state = msg.data

    def _on_command(self, msg: String):
        cmd = msg.data.strip()
        if not cmd: return
        if self.is_thinking:
            self._pub_feedback("Still thinking..."); return
        self.get_logger().info(f"Command: {cmd}")
        self.is_thinking = True; self._pub_status("thinking")
        self._pub_feedback(f'Command: "{cmd}"')
        try:
            w = json.loads(self.world_state)
            if w:
                self.get_logger().info(f"World ({len(w)} objects):")
                for n, o in w.items():
                    p = o.get("position", {})
                    self.get_logger().info(f"  {n}: ({p.get('x',0):.3f},{p.get('y',0):.3f},{p.get('z',0):.3f})")
            else: self.get_logger().info("World empty.")
        except: pass
        self._pub_feedback("Generating plan...")
        prompt = self.prompt_template.replace("{WORLD_MODEL}", self.world_state).replace("{COMMAND}", cmd)
        self.get_logger().info(f"Querying Ollama ({self.planner_model})...")
        try:
            r = requests.post(f"{self.ollama_url}/api/generate",
                json={"model": self.planner_model, "prompt": prompt, "stream": False, "options": {"temperature": self.temperature}},
                timeout=self.req_timeout)
            r.raise_for_status(); raw = r.json().get("response", "").strip()
            self.get_logger().info(f"Response:\n{raw}")
            plan = self._parse_plan(raw)
            if plan is None:
                self.get_logger().error("Parse failed."); self._pub_feedback("Could not parse plan."); return
            acts = plan.get("actions", []); reasoning = plan.get("reasoning", "")
            self.get_logger().info(f"Reasoning: {reasoning}")
            self.get_logger().info(f"{len(acts)} action(s):")
            for i, a in enumerate(acts): self.get_logger().info(f"  {i+1}. {a}")
            self._pub_feedback(reasoning)
            self._pub_feedback(f"{len(acts)} step(s) queued.")
            m = String(); m.data = json.dumps(plan); self.plan_pub.publish(m)
            self._pub_status("ready")
        except requests.exceptions.ConnectionError:
            self.get_logger().error("Ollama offline.")
            self._pub_feedback("Ollama offline."); self._pub_status("error: ollama offline")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            self._pub_feedback(f"Error: {e}"); self._pub_status("error")
        finally: self.is_thinking = False

    def _parse_plan(self, text: str):
        t = text.strip()
        if "```" in t: t = "\n".join(l for l in t.split("\n") if not l.strip().startswith("```"))
        s = t.find("{"); e = t.rfind("}") + 1
        if s == -1 or e == 0: return None
        try: return json.loads(t[s:e])
        except: return None

    def _pub_status(self, s: str):
        m = String(); m.data = s; self.status_pub.publish(m)
    def _pub_feedback(self, s: str):
        m = String(); m.data = s; self.feedback_pub.publish(m)


def main(args=None):
    rclpy.init(args=args); node = LLMPlannerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
