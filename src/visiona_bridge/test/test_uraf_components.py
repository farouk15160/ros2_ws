"""Unit tests for URAF wizard store and community library (PLAN.md 17, 21)."""

import tempfile
from pathlib import Path

from visiona_bridge.uraf.community_library import CommunityLibrary, compute_robot_fingerprint
from visiona_bridge.uraf.wizard_store import WizardStore, WIZARD_STEPS


def test_wizard_advance_steps():
    with tempfile.TemporaryDirectory() as tmp:
        store = WizardStore()
        store.path = Path(tmp) / "wizard_state.json"
        state = store.advance(0)
        assert state["step_name"] == WIZARD_STEPS[0]
        state = store.advance(3)
        assert state["step"] == 3


def test_robot_fingerprint_stable():
    profile = {
        "robot": {"dof": 6},
        "hardware": {"type": "serial", "joint_types": ["revolute"] * 6},
        "kinematics": {"dh_params": [[0.0, 1.57, 0.14, 0.0]]},
    }
    fp1 = compute_robot_fingerprint(profile)
    fp2 = compute_robot_fingerprint(profile)
    assert fp1 == fp2
    assert fp1.startswith("sha256:")


def test_community_library_loads_bundled():
    root = Path(__file__).resolve().parents[1] / "community"
    lib = CommunityLibrary(bundled_dir=root)
    profiles = lib.list_profiles()
    ids = [p["profile_id"] for p in profiles]
    assert "visiona_v1" in ids
