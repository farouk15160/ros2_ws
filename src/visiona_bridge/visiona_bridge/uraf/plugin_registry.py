"""URAF plugin registry (PLAN.md §20.3)."""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any

import yaml

from ..bridge_constants import get_data_dir


class PluginRegistry:
    """Install, list, and manage URAF plugins in ~/.visiona_bridge/plugins/."""

    REGISTRY_NAME = "registry.json"

    def __init__(self, logger=None):
        self._logger = logger
        self.root = Path(get_data_dir()) / "plugins"
        self.root.mkdir(parents=True, exist_ok=True)
        self.registry_path = self.root / self.REGISTRY_NAME
        self._ensure_registry()

    def _ensure_registry(self):
        if not self.registry_path.is_file():
            self._save_registry({"plugins": [], "version": "1.0"})

    def _load_registry(self) -> dict[str, Any]:
        try:
            return json.loads(self.registry_path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            return {"plugins": [], "version": "1.0"}

    def _save_registry(self, data: dict[str, Any]):
        self.registry_path.write_text(json.dumps(data, indent=2), encoding="utf-8")

    def list_plugins(self) -> list[dict[str, Any]]:
        return self._load_registry().get("plugins", [])

    def install_from_manifest(self, manifest_path: Path) -> dict[str, Any]:
        manifest_path = Path(manifest_path)
        if not manifest_path.is_file():
            raise FileNotFoundError(f"Plugin manifest not found: {manifest_path}")

        raw = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
        info = raw.get("plugin_info", raw)
        name = info.get("name")
        if not name:
            raise ValueError("plugin_info.name is required")

        dest = self.root / name
        src_dir = manifest_path.parent
        if dest.exists():
            shutil.rmtree(dest)
        shutil.copytree(src_dir, dest)

        entry = {
            "name": name,
            "type": info.get("type", "generic"),
            "version": info.get("version", "1.0.0"),
            "author": info.get("author", "unknown"),
            "description": info.get("description", ""),
            "entry_point": info.get("entry_point", ""),
            "path": str(dest),
            "enabled": True,
        }
        reg = self._load_registry()
        plugins = [p for p in reg.get("plugins", []) if p.get("name") != name]
        plugins.append(entry)
        reg["plugins"] = plugins
        self._save_registry(reg)
        if self._logger:
            self._logger.info(f"Plugin installed: {name}")
        return entry

    def install_bundled(self, bundled_share_path: Path, plugin_name: str) -> dict[str, Any]:
        manifest = bundled_share_path / plugin_name / "plugin.yaml"
        return self.install_from_manifest(manifest)

    def set_enabled(self, name: str, enabled: bool) -> bool:
        reg = self._load_registry()
        found = False
        for p in reg.get("plugins", []):
            if p.get("name") == name:
                p["enabled"] = enabled
                found = True
        if found:
            self._save_registry(reg)
        return found

    def uninstall(self, name: str) -> bool:
        reg = self._load_registry()
        plugins = reg.get("plugins", [])
        new_list = [p for p in plugins if p.get("name") != name]
        if len(new_list) == len(plugins):
            return False
        dest = self.root / name
        if dest.is_dir():
            shutil.rmtree(dest)
        reg["plugins"] = new_list
        self._save_registry(reg)
        return True
