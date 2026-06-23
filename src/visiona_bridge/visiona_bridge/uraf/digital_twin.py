"""Digital Twin subsystem — motion validation and state mirror (PLAN.md §18)."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class ValidationResult:
    approved: bool
    reason: str
    checks: dict[str, bool] = field(default_factory=dict)
    confidence: float = 1.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class TwinState:
    joints_rad: list[float] = field(default_factory=lambda: [0.0] * 6)
    ee_x: float = 0.0
    ee_y: float = 0.25
    ee_z: float = 0.35
    backend: str = "bridge_sim"
    synced: bool = False
    last_update: float = field(default_factory=time.time)


class MotionValidator:
    """Validate Cartesian targets against workspace and reach limits."""

    def __init__(self, workspace: dict[str, float] | None = None):
        ws = workspace or {}
        self.x_min = float(ws.get("x_min", -0.63))
        self.x_max = float(ws.get("x_max", 0.63))
        self.y_min = float(ws.get("y_min", -0.63))
        self.y_max = float(ws.get("y_max", 0.63))
        self.z_min = float(ws.get("z_min", 0.0))
        self.z_max = float(ws.get("z_max", 0.63))
        self.min_radius = float(ws.get("min_radius", 0.12))
        self.max_reach = float(ws.get("max_reach", 0.69))

    def validate(self, x: float, y: float, z: float) -> ValidationResult:
        checks = {
            "workspace_x": self.x_min <= x <= self.x_max,
            "workspace_y": self.y_min <= y <= self.y_max,
            "workspace_z": self.z_min <= z <= self.z_max,
        }
        radius_xy = math.sqrt(x * x + y * y)
        checks["min_radius"] = radius_xy >= self.min_radius
        reach = math.sqrt(x * x + y * y + z * z)
        checks["max_reach"] = reach <= self.max_reach

        if not all(checks.values()):
            failed = [k for k, ok in checks.items() if not ok]
            return ValidationResult(
                approved=False,
                reason=f"Motion rejected: failed checks {failed}",
                checks=checks,
                confidence=0.95,
            )
        return ValidationResult(
            approved=True,
            reason="Twin validation passed",
            checks=checks,
            confidence=0.98,
        )


class DigitalTwinEngine:
    """Maintain twin state and validate motions before physical execution."""

    AGENT_ID = "digital_twin_agent"

    def __init__(self, backend: str = "bridge_sim", workspace: dict | None = None):
        self.backend = backend
        self.state = TwinState(backend=backend)
        self.validator = MotionValidator(workspace)
        self.validation_history: list[dict[str, Any]] = []

    def update_joints(self, positions: list[float]):
        self.state.joints_rad = positions[:6] if positions else self.state.joints_rad
        self.state.synced = True
        self.state.last_update = time.time()

    def update_ee_pose(self, x: float, y: float, z: float):
        self.state.ee_x = x
        self.state.ee_y = y
        self.state.ee_z = z
        self.state.last_update = time.time()

    def validate_motion(self, x: float, y: float, z: float) -> ValidationResult:
        result = self.validator.validate(x, y, z)
        self.validation_history.append({
            "target": {"x": x, "y": y, "z": z},
            "approved": result.approved,
            "reason": result.reason,
            "timestamp": result.timestamp,
        })
        if len(self.validation_history) > 100:
            self.validation_history = self.validation_history[-100:]
        return result

    def status(self) -> dict[str, Any]:
        return {
            "agent_id": self.AGENT_ID,
            "backend": self.backend,
            "synced": self.state.synced,
            "last_update": self.state.last_update,
            "joints_rad": self.state.joints_rad,
            "ee_pose": {"x": self.state.ee_x, "y": self.state.ee_y, "z": self.state.ee_z},
            "validations_total": len(self.validation_history),
            "validations_passed": sum(1 for v in self.validation_history if v["approved"]),
        }
