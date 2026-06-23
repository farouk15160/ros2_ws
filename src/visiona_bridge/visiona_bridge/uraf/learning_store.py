"""Phase 6 — Learning data persistence (PLAN.md §12.2)."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from ..bridge_constants import get_data_dir


class LearningStore:
    """Record task executions, grasp outcomes, and planner performance."""

    def __init__(self, logger=None):
        self._logger = logger
        self.root = Path(get_data_dir()) / "learning"
        self.root.mkdir(parents=True, exist_ok=True)
        self.executions_path = self.root / "executions.jsonl"
        self.grasp_model_path = self.root / "grasp_success_model.json"
        self.planner_model_path = self.root / "planner_performance.json"
        self.task_library_path = self.root / "task_plan_library.json"

    def record_execution(self, event: dict[str, Any]):
        event.setdefault("timestamp", time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
        with open(self.executions_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(event) + "\n")
        self._update_models(event)

    def _update_models(self, event: dict[str, Any]):
        grasp = self._load_json(self.grasp_model_path, {})
        planner = self._load_json(self.planner_model_path, {})
        tasks = self._load_json(self.task_library_path, {"plans": []})

        action = event.get("action_type", "unknown")
        success = bool(event.get("success", False))
        object_name = event.get("object", "")
        backend = event.get("planning_backend", "unknown")
        duration = float(event.get("duration_sec", 0.0))

        if action == "pick" and object_name:
            entry = grasp.setdefault(object_name, {"attempts": 0, "successes": 0})
            entry["attempts"] += 1
            if success:
                entry["successes"] += 1
            entry["success_rate"] = round(entry["successes"] / max(entry["attempts"], 1), 3)

        if backend:
            p = planner.setdefault(backend, {"runs": 0, "successes": 0, "total_duration_sec": 0.0})
            p["runs"] += 1
            if success:
                p["successes"] += 1
            p["total_duration_sec"] = round(p["total_duration_sec"] + duration, 3)
            p["avg_duration_sec"] = round(p["total_duration_sec"] / max(p["runs"], 1), 3)
            p["success_rate"] = round(p["successes"] / max(p["runs"], 1), 3)

        plan_id = event.get("plan_id")
        if plan_id and event.get("event") == "plan_complete":
            tasks["plans"].append({
                "plan_id": plan_id,
                "actions": event.get("actions", []),
                "success": success,
                "timestamp": event.get("timestamp"),
            })
            tasks["plans"] = tasks["plans"][-200:]

        self._save_json(self.grasp_model_path, grasp)
        self._save_json(self.planner_model_path, planner)
        self._save_json(self.task_library_path, tasks)

    def get_stats(self) -> dict[str, Any]:
        executions = 0
        if self.executions_path.is_file():
            with open(self.executions_path, encoding="utf-8") as f:
                executions = sum(1 for _ in f)
        return {
            "executions_recorded": executions,
            "grasp_models": self._load_json(self.grasp_model_path, {}),
            "planner_performance": self._load_json(self.planner_model_path, {}),
            "task_plans": len(self._load_json(self.task_library_path, {}).get("plans", [])),
            "store_path": str(self.root),
        }

    def _load_json(self, path: Path, default: Any) -> Any:
        if not path.is_file():
            return default
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            return default

    def _save_json(self, path: Path, data: Any):
        path.write_text(json.dumps(data, indent=2), encoding="utf-8")
