"""Bridge JARVIS ROS topics to Flask SocketIO events for the web GUI."""

import json

from std_msgs.msg import String

from .socketio_handlers import (
    emit_jarvis_action_plan,
    emit_jarvis_feedback,
    emit_jarvis_world_state,
    emit_llm_status,
)


class JarvisGuiBridge:
    """Subscribe to JARVIS topics and push updates to connected browsers."""

    def __init__(self, node, socketio):
        self._node = node
        self._socketio = socketio

        node.create_subscription(String, '/jarvis/feedback', self._on_feedback, 10)
        node.create_subscription(String, '/jarvis/world_state', self._on_world_state, 10)
        node.create_subscription(String, '/jarvis/action_plan', self._on_action_plan, 10)
        node.create_subscription(String, '/jarvis/llm_status', self._on_llm_status, 10)

        node.get_logger().info('JARVIS GUI bridge active (SocketIO feedback wired)')

    def _on_feedback(self, msg: String):
        emit_jarvis_feedback(self._socketio, msg.data)

    def _on_world_state(self, msg: String):
        try:
            self._node._jarvis_world_state = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self._node._jarvis_world_state = {}
        emit_jarvis_world_state(self._socketio, msg.data)

    def _on_action_plan(self, msg: String):
        emit_jarvis_action_plan(self._socketio, msg.data)

    def _on_llm_status(self, msg: String):
        self._node._jarvis_llm_status = msg.data
        emit_llm_status(self._socketio, msg.data)
