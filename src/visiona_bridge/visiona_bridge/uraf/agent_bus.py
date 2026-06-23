"""URAF Agent Orchestration Bus — ROS 2 pub/sub transport (Phase 0)."""

from __future__ import annotations

from std_msgs.msg import String

from .agent_message import AgentMessage, MessageType


class AgentBus:
    """Thin wrapper publishing JSON agent messages on /uraf/agent_bus."""

    TOPIC = "/uraf/agent_bus"

    def __init__(self, node):
        self._node = node
        self._pub = node.create_publisher(String, self.TOPIC, 10)
        self._handlers: dict[str, list] = {}
        node.create_subscription(String, self.TOPIC, self._on_message, 10)

    def publish(self, message: AgentMessage) -> None:
        msg = String()
        msg.data = message.to_json()
        self._pub.publish(msg)

    def emit_status(self, agent_id: str, payload_type: str, payload: dict, confidence: float = 1.0):
        self.publish(AgentMessage(
            agent_id=agent_id,
            message_type=MessageType.STATUS,
            payload_type=payload_type,
            payload=payload,
            confidence=confidence,
        ))

    def emit_result(self, agent_id: str, payload_type: str, payload: dict, confidence: float = 1.0):
        self.publish(AgentMessage(
            agent_id=agent_id,
            message_type=MessageType.RESULT,
            payload_type=payload_type,
            payload=payload,
            confidence=confidence,
        ))

    def on(self, payload_type: str, callback):
        self._handlers.setdefault(payload_type, []).append(callback)

    def _on_message(self, msg: String):
        try:
            message = AgentMessage.from_json(msg.data)
        except (ValueError, KeyError):
            return
        for cb in self._handlers.get(message.payload_type, []):
            try:
                cb(message)
            except Exception as exc:
                self._node.get_logger().warn(f"AgentBus handler error: {exc}")
