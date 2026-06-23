"""Phase 6 — Self-healing detection and recovery logic (PLAN.md §12.1)."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class FailureType(str, Enum):
    JOINT_STATES_STALE = "joint_states_stale"
    SERIAL_DISCONNECTED = "serial_disconnected"
    MOTION_TIMEOUT = "motion_timeout"
    PERCEPTION_DEGRADED = "perception_degraded"
    LLM_UNAVAILABLE = "llm_unavailable"
    ESTOP_ACTIVE = "estop_active"


class RecoveryAction(str, Enum):
    REPUBLISH_HOME = "republish_home"
    TRIGGER_DISCOVERY = "trigger_discovery"
    TRIGGER_RESCAN = "trigger_rescan"
    ALERT_USER = "alert_user"
    CLEAR_ESTOP = "clear_estop"


@dataclass
class FailureEvent:
    failure_type: FailureType
    message: str
    severity: str = "warn"
    metadata: dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


@dataclass
class RecoveryResult:
    action: RecoveryAction
    success: bool
    message: str
    timestamp: float = field(default_factory=time.time)


class SelfHealingEngine:
    """Detect failures and choose recovery actions with cooldowns."""

    RECOVERY_MAP: dict[FailureType, list[RecoveryAction]] = {
        FailureType.JOINT_STATES_STALE: [
            RecoveryAction.TRIGGER_DISCOVERY,
            RecoveryAction.REPUBLISH_HOME,
            RecoveryAction.ALERT_USER,
        ],
        FailureType.SERIAL_DISCONNECTED: [
            RecoveryAction.TRIGGER_DISCOVERY,
            RecoveryAction.ALERT_USER,
        ],
        FailureType.MOTION_TIMEOUT: [
            RecoveryAction.REPUBLISH_HOME,
            RecoveryAction.ALERT_USER,
        ],
        FailureType.PERCEPTION_DEGRADED: [
            RecoveryAction.TRIGGER_RESCAN,
            RecoveryAction.ALERT_USER,
        ],
        FailureType.LLM_UNAVAILABLE: [
            RecoveryAction.ALERT_USER,
        ],
        FailureType.ESTOP_ACTIVE: [
            RecoveryAction.ALERT_USER,
        ],
    }

    def __init__(self, cooldown_sec: float = 30.0, max_auto_attempts: int = 3):
        self.cooldown_sec = cooldown_sec
        self.max_auto_attempts = max_auto_attempts
        self._last_recovery: dict[str, float] = {}
        self._attempt_counts: dict[str, int] = {}
        self.history: list[RecoveryResult] = []

    def plan_recovery(self, event: FailureEvent) -> list[RecoveryAction]:
        key = event.failure_type.value
        attempts = self._attempt_counts.get(key, 0)
        if attempts >= self.max_auto_attempts:
            return [RecoveryAction.ALERT_USER]

        last = self._last_recovery.get(key, 0.0)
        if time.time() - last < self.cooldown_sec:
            return []

        actions = list(self.RECOVERY_MAP.get(event.failure_type, [RecoveryAction.ALERT_USER]))
        if attempts >= 1 and RecoveryAction.ALERT_USER not in actions:
            actions.append(RecoveryAction.ALERT_USER)
        return actions

    def record_recovery(self, event: FailureEvent, result: RecoveryResult):
        key = event.failure_type.value
        self._last_recovery[key] = time.time()
        if result.success:
            self._attempt_counts[key] = self._attempt_counts.get(key, 0) + 1
        self.history.append(result)
        if len(self.history) > 100:
            self.history = self.history[-100:]

    def build_health_report(
        self,
        *,
        joint_ok: bool,
        serial_ok: bool,
        llm_ok: bool,
        perception_ok: bool,
        estop: bool,
        active_failures: list[dict],
        recovery_stats: dict,
    ) -> dict[str, Any]:
        status = "healthy"
        if estop or not serial_ok:
            status = "critical"
        elif not joint_ok or active_failures:
            status = "degraded"

        return {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "status": status,
            "checks": {
                "joint_states_ok": joint_ok,
                "serial_connected": serial_ok,
                "llm_available": llm_ok,
                "perception_ok": perception_ok,
                "estop_active": estop,
            },
            "active_failures": active_failures,
            "recovery": recovery_stats,
            "agent_id": "self_healing_agent",
        }
