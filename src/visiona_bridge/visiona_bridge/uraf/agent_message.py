"""URAF inter-agent message schema (PLAN.md §6.2.2)."""

from __future__ import annotations

import json
import time
import uuid
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Optional


class MessageType(str, Enum):
    RESULT = "RESULT"
    REQUEST = "REQUEST"
    STATUS = "STATUS"
    ERROR = "ERROR"
    CONFIRMATION_REQUIRED = "CONFIRMATION_REQUIRED"


@dataclass
class AgentMessage:
    agent_id: str
    message_type: MessageType
    payload_type: str
    payload: dict[str, Any]
    pipeline_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    confidence: float = 1.0
    requires_human_confirmation: bool = False
    confirmation_question: Optional[str] = None
    rollback_id: Optional[str] = None
    parent_message_id: Optional[str] = None

    def to_json(self) -> str:
        body = asdict(self)
        body["message_type"] = self.message_type.value
        body["$schema"] = "uraf/agent-message/v1"
        return json.dumps(body, indent=2)

    @classmethod
    def from_json(cls, raw: str) -> "AgentMessage":
        data = json.loads(raw)
        return cls(
            agent_id=data["agent_id"],
            message_type=MessageType(data["message_type"]),
            payload_type=data["payload_type"],
            payload=data.get("payload", {}),
            pipeline_id=data.get("pipeline_id", str(uuid.uuid4())),
            timestamp_ns=data.get("timestamp_ns", time.time_ns()),
            confidence=float(data.get("confidence", 1.0)),
            requires_human_confirmation=bool(data.get("requires_human_confirmation", False)),
            confirmation_question=data.get("confirmation_question"),
            rollback_id=data.get("rollback_id"),
            parent_message_id=data.get("parent_message_id"),
        )
