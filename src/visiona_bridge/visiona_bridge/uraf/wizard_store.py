"""GUI Setup Wizard state persistence (PLAN.md §17)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from ..bridge_constants import get_data_dir


WIZARD_STEPS = [
    "welcome",
    "hardware",
    "discovery",
    "confirm",
    "visualize",
    "calibrate",
    "complete",
]


class WizardStore:
    """Persist multi-step wizard progress."""

    def __init__(self):
        self.path = Path(get_data_dir()) / "wizard_state.json"
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def load(self) -> dict[str, Any]:
        if not self.path.is_file():
            return self.default_state()
        try:
            return json.loads(self.path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            return self.default_state()

    def save(self, state: dict[str, Any]) -> dict[str, Any]:
        self.path.write_text(json.dumps(state, indent=2), encoding="utf-8")
        return state

    def default_state(self) -> dict[str, Any]:
        return {
            "step": 0,
            "step_name": WIZARD_STEPS[0],
            "mode": "guided",
            "discovery": {},
            "profile_match": {},
            "confirmed": False,
            "calibration_progress": 0,
            "complete": False,
        }

    def advance(self, step: int | None = None) -> dict[str, Any]:
        state = self.load()
        idx = step if step is not None else state.get("step", 0) + 1
        idx = max(0, min(idx, len(WIZARD_STEPS) - 1))
        state["step"] = idx
        state["step_name"] = WIZARD_STEPS[idx]
        if idx == len(WIZARD_STEPS) - 1:
            state["complete"] = True
        return self.save(state)

    def update(self, patch: dict[str, Any]) -> dict[str, Any]:
        state = self.load()
        state.update(patch)
        if "step" in patch:
            idx = int(patch["step"])
            state["step_name"] = WIZARD_STEPS[max(0, min(idx, len(WIZARD_STEPS) - 1))]
        return self.save(state)
