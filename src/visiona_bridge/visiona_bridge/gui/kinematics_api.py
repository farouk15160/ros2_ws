"""Kinematics helpers for the web GUI 3D viewport."""

from __future__ import annotations

import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def _load_kinematics() -> dict:
    share = Path(get_package_share_directory("visiona_bridge"))
    cfg_path = share / "config" / "kinematic_params.yaml"
    with open(cfg_path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    params = raw.get("kinematic_params", {}).get("ros__parameters", raw)
    dh = params.get("dh_params", {})
    rows = []
    for key in ("joint_0", "joint_1", "joint_2", "joint_3"):
        j = dh.get(key, {})
        rows.append([j.get("a", 0.0), j.get("alpha", 0.0), j.get("d", 0.0), j.get("theta_offset", 0.0)])
    return {
        "dh_params": rows,
        "joint_names": ["J0 Base", "J1 Shoulder", "J2 Elbow", "J3 Wrist"],
        "joint_count": 6,
        "workspace": params.get("workspace", {}),
    }
