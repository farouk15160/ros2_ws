"""Versioned URAF configuration store (PLAN.md §5.2, local-first)."""

from __future__ import annotations

import json
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import yaml

from ..bridge_constants import get_config_dir


class ConfigStore:
    """Load, save, and snapshot URAF YAML configuration."""

    CONFIG_NAME = "uraf_config.yaml"
    HISTORY_DIR = "config_history"

    def __init__(self, logger=None):
        self._logger = logger
        self.config_dir = Path(get_config_dir())
        self.config_dir.mkdir(parents=True, exist_ok=True)
        self.history_dir = self.config_dir / self.HISTORY_DIR
        self.history_dir.mkdir(parents=True, exist_ok=True)
        self.active_path = self.config_dir / self.CONFIG_NAME

    def load(self, fallback_path: Optional[Path] = None) -> dict[str, Any]:
        if self.active_path.is_file():
            with open(self.active_path, encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        if fallback_path and fallback_path.is_file():
            data = yaml.safe_load(fallback_path.read_text(encoding="utf-8")) or {}
            self.save(data, snapshot=False)
            return data
        return {}

    def save(self, data: dict[str, Any], snapshot: bool = True) -> Path:
        if snapshot and self.active_path.is_file():
            stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
            backup = self.history_dir / f"uraf_config_{stamp}.yaml"
            shutil.copy2(self.active_path, backup)
        with open(self.active_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        if self._logger:
            self._logger.info(f"URAF config saved ? {self.active_path}")
        return self.active_path

    def list_snapshots(self) -> list[dict[str, str]]:
        items = sorted(self.history_dir.glob("uraf_config_*.yaml"), reverse=True)
        return [{"name": p.name, "path": str(p)} for p in items[:20]]

    def export_json(self) -> str:
        return json.dumps(self.load(), indent=2)
