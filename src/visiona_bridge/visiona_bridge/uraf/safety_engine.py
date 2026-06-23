"""URAF Safety Engine — PLAN.md §19 (SIL 2-oriented software safety layer)."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class SafetyFunction(str, Enum):
    SF01_JOINT_LIMIT = "SF-01"
    SF02_JOINT_VELOCITY = "SF-02"
    SF05_CURRENT_OVERLOAD = "SF-05"
    SF07_WATCHDOG = "SF-07"
    SF08_ESTOP = "SF-08"
    SF10_WORKSPACE = "SF-10"
    SF11_SENSOR_LOSS = "SF-11"


class StartupPhase(str, Enum):
    POWER_ON = "power_on"
    FAL_CHECK = "fal_check"
    POSITION_VALIDATE = "position_validate"
    CONTROLLER_ENABLE = "controller_enable"
    MOVE_HOME = "move_home"
    OPERATIONAL = "operational"


@dataclass
class SafetyViolation:
    function: SafetyFunction
    message: str
    severity: str  # warn | critical
    timestamp: float = field(default_factory=time.time)


@dataclass
class SafetyState:
    armed: bool = False
    operational: bool = False
    startup_phase: StartupPhase = StartupPhase.POWER_ON
    motion_scale: float = 0.0
    violations: list[SafetyViolation] = field(default_factory=list)
    estop_active: bool = False


class SafetyEngine:
    """Central safety function evaluator."""

    AGENT_ID = "safety_monitor_agent"

    def __init__(self, params: dict[str, Any] | None = None):
        p = params or {}
        self.joint_limits = p.get("joint_limits_deg", {})
        self.max_velocity = float(p.get("max_joint_velocity_rad_s", 3.0))
        self.max_current = float(p.get("max_main_current_a", 5.0))
        self.workspace = p.get("workspace", {})
        self.boundary_zone = float(self.workspace.get("boundary_slow_zone_m", 0.05))
        self.watchdog_sec = float(p.get("watchdog_timeout_sec", 0.6))
        self.state = SafetyState()
        self._prev_joints: list[float] | None = None
        self._prev_time: float | None = None

    def evaluate(
        self,
        *,
        joints_rad: list[float] | None,
        ee_x: float | None,
        ee_y: float | None,
        ee_z: float | None,
        main_current: float,
        estop: bool,
        serial_ok: bool,
        joint_age_sec: float | None,
    ) -> SafetyState:
        violations: list[SafetyViolation] = []
        motion_scale = 1.0

        if estop:
            violations.append(SafetyViolation(SafetyFunction.SF08_ESTOP, "Emergency stop active", "critical"))

        if not serial_ok and self.state.armed:
            violations.append(SafetyViolation(
                SafetyFunction.SF07_WATCHDOG, "Serial watchdog — MCU disconnected", "critical"))

        if joint_age_sec is not None and joint_age_sec > self.watchdog_sec:
            violations.append(SafetyViolation(
                SafetyFunction.SF11_SENSOR_LOSS, f"Joint states stale ({joint_age_sec:.1f}s)", "critical"))

        if main_current > self.max_current:
            violations.append(SafetyViolation(
                SafetyFunction.SF05_CURRENT_OVERLOAD,
                f"Main current {main_current:.2f}A exceeds {self.max_current}A",
                "critical",
            ))

        if joints_rad:
            for i, val in enumerate(joints_rad[:6]):
                key = f"joint_{i}"
                lim = self.joint_limits.get(key, {})
                lo = math.radians(float(lim.get("lower", -180)))
                hi = math.radians(float(lim.get("upper", 180)))
                if val < lo - 0.01 or val > hi + 0.01:
                    violations.append(SafetyViolation(
                        SafetyFunction.SF01_JOINT_LIMIT,
                        f"Joint {i} limit violation ({math.degrees(val):.1f}°)",
                        "critical",
                    ))

            if self._prev_joints and self._prev_time:
                dt = time.time() - self._prev_time
                if dt > 0.001:
                    for i, (a, b) in enumerate(zip(self._prev_joints[:6], joints_rad[:6])):
                        vel = abs(b - a) / dt
                        if vel > self.max_velocity:
                            violations.append(SafetyViolation(
                                SafetyFunction.SF02_JOINT_VELOCITY,
                                f"Joint {i} velocity {vel:.2f} rad/s exceeds limit",
                                "warn",
                            ))
            self._prev_joints = list(joints_rad)
            self._prev_time = time.time()

        if ee_x is not None and ee_y is not None and ee_z is not None:
            scale, ws_violations = self._workspace_scale(ee_x, ee_y, ee_z)
            motion_scale = min(motion_scale, scale)
            violations.extend(ws_violations)

        critical = [v for v in violations if v.severity == "critical"]
        self.state.violations = violations
        self.state.estop_active = estop or bool(critical)
        self.state.motion_scale = 0.0 if self.state.estop_active else motion_scale
        self.state.operational = (
            self.state.startup_phase == StartupPhase.OPERATIONAL
            and self.state.armed
            and not self.state.estop_active
        )
        return self.state

    def _workspace_scale(self, x: float, y: float, z: float) -> tuple[float, list[SafetyViolation]]:
        violations = []
        xmin = float(self.workspace.get("x_min", -0.63))
        xmax = float(self.workspace.get("x_max", 0.63))
        ymin = float(self.workspace.get("y_min", -0.63))
        ymax = float(self.workspace.get("y_max", 0.63))
        zmin = float(self.workspace.get("z_min", 0.0))
        zmax = float(self.workspace.get("z_max", 0.63))

        if not (xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax):
            violations.append(SafetyViolation(
                SafetyFunction.SF10_WORKSPACE,
                f"EE outside workspace ({x:.2f},{y:.2f},{z:.2f})",
                "critical",
            ))
            return 0.0, violations

        margins = [x - xmin, xmax - x, y - ymin, ymax - y, z - zmin, zmax - z]
        min_margin = min(margins)
        if min_margin < self.boundary_zone:
            scale = max(0.1, min_margin / self.boundary_zone)
            violations.append(SafetyViolation(
                SafetyFunction.SF10_WORKSPACE,
                f"Approaching workspace boundary (margin {min_margin:.3f}m)",
                "warn",
            ))
            return scale, violations
        return 1.0, violations

    def arm(self):
        self.state.armed = True

    def disarm(self):
        self.state.armed = False
        self.state.operational = False
        self.state.motion_scale = 0.0

    def advance_startup(self, phase: StartupPhase):
        self.state.startup_phase = phase
        if phase == StartupPhase.OPERATIONAL:
            self.state.armed = True
            self.state.operational = not self.state.estop_active

    def status_dict(self) -> dict[str, Any]:
        return {
            "agent_id": self.AGENT_ID,
            "armed": self.state.armed,
            "operational": self.state.operational,
            "startup_phase": self.state.startup_phase.value,
            "motion_scale": round(self.state.motion_scale, 3),
            "estop_active": self.state.estop_active,
            "violations": [
                {"function": v.function.value, "message": v.message, "severity": v.severity}
                for v in self.state.violations
            ],
        }
