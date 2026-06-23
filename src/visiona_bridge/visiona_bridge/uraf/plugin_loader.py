"""Dynamic URAF plugin loader (PLAN.md §20.2)."""

from __future__ import annotations

import importlib
from typing import Any

from .plugin_registry import PluginRegistry


class PluginLoader:
    """Load enabled plugins by entry_point string."""

    def __init__(self, registry: PluginRegistry | None = None, logger=None):
        self.registry = registry or PluginRegistry(logger)
        self._logger = logger
        self._instances: dict[str, Any] = {}

    def load_all(self) -> dict[str, Any]:
        loaded = {}
        for meta in self.registry.list_plugins():
            if not meta.get("enabled", True):
                continue
            name = meta.get("name", "")
            try:
                inst = self._load_entry(meta.get("entry_point", ""))
                if inst is not None:
                    loaded[name] = inst
            except Exception as exc:
                if self._logger:
                    self._logger.warn(f"Plugin {name} failed to load: {exc}")
        self._instances = loaded
        return loaded

    def _load_entry(self, entry_point: str):
        if not entry_point or ":" not in entry_point:
            return None
        module_path, class_name = entry_point.rsplit(":", 1)
        module = importlib.import_module(module_path)
        cls = getattr(module, class_name)
        return cls()

    def status(self) -> dict[str, Any]:
        plugins = self.registry.list_plugins()
        return {
            "installed": len(plugins),
            "enabled": sum(1 for p in plugins if p.get("enabled", True)),
            "loaded": len(self._instances),
            "plugins": plugins,
        }
