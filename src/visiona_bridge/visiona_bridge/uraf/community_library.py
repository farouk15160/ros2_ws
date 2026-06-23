"""Community Robot Library  fingerprinting and profile lookup (PLAN.md 21)."""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path
from typing import Any

import yaml

from ..bridge_constants import get_data_dir


def compute_robot_fingerprint(profile: dict[str, Any]) -> str:
    """Stable hash of robot physical identity (excludes calibration)."""
    robot = profile.get("robot", profile.get("crl", profile))
    hw = profile.get("hardware", robot.get("hardware", {}))
    kin = profile.get("kinematics", robot.get("kinematics", profile.get("geometry", {})))
    dh = kin.get("dh_params", [])
    identity = {
        "dof": robot.get("dof", hw.get("dof", 6)),
        "joint_types": hw.get("joint_types", ["revolute"] * 6),
        "firmware_type": hw.get("type", hw.get("firmware", "unknown")),
        "dh_params_approx": [[round(float(v), 2) for v in row] for row in dh[:6]],
    }
    raw = json.dumps(identity, sort_keys=True)
    return "sha256:" + hashlib.sha256(raw.encode()).hexdigest()[:16]


class CommunityLibrary:
    """Curated robot profiles  bundled + user-installed."""

    AGENT_ID = "community_library_agent"

    def __init__(self, bundled_dir: Path | None = None, logger=None):
        self._logger = logger
        self.user_dir = Path(get_data_dir()) / "community"
        self.user_dir.mkdir(parents=True, exist_ok=True)
        self.bundled_dir = bundled_dir or Path()
        self._profiles: dict[str, dict] = {}
        self.reload()

    def reload(self):
        self._profiles = {}
        for base in (self.bundled_dir, self.user_dir):
            if not base.is_dir():
                continue
            for path in sorted(base.glob("*.yaml")):
                try:
                    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
                    crl = data.get("crl", data)
                    pid = crl.get("profile_id", path.stem)
                    if not crl.get("fingerprint"):
                        crl["fingerprint"] = compute_robot_fingerprint(data)
                    self._profiles[pid] = {"crl": crl, "path": str(path), "raw": data}
                except (yaml.YAMLError, OSError):
                    continue

    def list_profiles(self) -> list[dict[str, Any]]:
        out = []
        for pid, entry in self._profiles.items():
            crl = entry["crl"]
            out.append({
                "profile_id": pid,
                "robot_name": crl.get("robot_name", pid),
                "manufacturer": crl.get("manufacturer", "unknown"),
                "fingerprint": crl.get("fingerprint"),
                "test_status": crl.get("test_status", "community"),
                "version": crl.get("version", "1.0"),
            })
        return sorted(out, key=lambda x: x["profile_id"])

    def lookup_by_fingerprint(self, fingerprint: str) -> dict | None:
        for entry in self._profiles.values():
            if entry["crl"].get("fingerprint") == fingerprint:
                return entry
        return None

    def lookup_by_id(self, profile_id: str) -> dict | None:
        return self._profiles.get(profile_id)

    def lookup_from_discovery(self, discovery: dict) -> dict | None:
        family = discovery.get("known_robot_family")
        if family:
            for entry in self._profiles.values():
                if entry["crl"].get("profile_id", "").startswith(family.split("_")[0]):
                    return entry
        match = discovery.get("community_profile_match", {})
        if match.get("found"):
            return self.lookup_by_id(match.get("profile_id", "visiona_v1"))
        return None

    def install_profile(self, src_path: Path) -> dict[str, Any]:
        src_path = Path(src_path)
        data = yaml.safe_load(src_path.read_text(encoding="utf-8")) or {}
        crl = data.get("crl", data)
        pid = crl.get("profile_id", src_path.stem)
        dest = self.user_dir / f"{pid}.yaml"
        shutil.copy2(src_path, dest)
        self.reload()
        return {"profile_id": pid, "path": str(dest)}

    def export_profile(self, profile_id: str) -> dict | None:
        entry = self.lookup_by_id(profile_id)
        return entry["raw"] if entry else None
