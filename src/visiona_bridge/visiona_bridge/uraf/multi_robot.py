"""Phase 6  Multi-robot coordination helpers (PLAN.md 12.3)."""

from __future__ import annotations

from typing import Any


DEFAULT_SINGLE_ROBOT = {
    "robots": [
        {
            "name": "visiona",
            "namespace": "",
            "config": "config/robots/visiona_v1.yaml",
            "base_frame_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }
    ],
    "multi_robot": {
        "enabled": False,
        "shared_planning_scene": False,
        "coordination_strategy": "decoupled",
        "priority": ["visiona"],
    },
}


class MultiRobotCoordinator:
    """Namespace and priority management for single or dual-arm setups."""

    def __init__(self, config: dict[str, Any] | None = None):
        self.config = config or DEFAULT_SINGLE_ROBOT
        multi = self.config.get("multi_robot") or {}
        robots = self.config.get("robots") or multi.get("robots") or DEFAULT_SINGLE_ROBOT["robots"]
        self.robots = robots
        self.settings = {**DEFAULT_SINGLE_ROBOT["multi_robot"], **multi}
        self.enabled = bool(self.settings.get("enabled", False)) and len(robots) > 1

    def topic(self, robot_name: str, relative: str) -> str:
        robot = self._robot(robot_name)
        ns = (robot or {}).get("namespace", "").strip("/")
        rel = relative if relative.startswith("/") else f"/{relative}"
        return f"/{ns}{rel}" if ns else rel

    def active_robot(self) -> dict[str, Any]:
        priority = self.settings.get("priority") or [r["name"] for r in self.robots]
        for name in priority:
            robot = self._robot(name)
            if robot:
                return robot
        return self.robots[0]

    def status(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "robot_count": len(self.robots),
            "robots": [
                {
                    "name": r.get("name"),
                    "namespace": r.get("namespace", ""),
                    "config": r.get("config"),
                    "base_frame_pose": r.get("base_frame_pose", {}),
                }
                for r in self.robots
            ],
            "coordination_strategy": self.settings.get("coordination_strategy", "decoupled"),
            "priority": self.settings.get("priority", []),
            "active_robot": self.active_robot().get("name"),
        }

    def _robot(self, name: str) -> dict[str, Any] | None:
        for r in self.robots:
            if r.get("name") == name:
                return r
        return None
