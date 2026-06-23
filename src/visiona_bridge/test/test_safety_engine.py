"""Unit tests for URAF safety engine (PLAN.md §19, §23)."""

import math

from visiona_bridge.uraf.safety_engine import SafetyEngine, StartupPhase


def _engine():
    return SafetyEngine({
        "joint_limits_deg": {
            "joint_0": {"lower": -180, "upper": 180},
            "joint_1": {"lower": 0, "upper": 180},
        },
        "max_joint_velocity_rad_s": 2.0,
        "max_main_current_a": 5.0,
        "workspace": {
            "x_min": -0.5, "x_max": 0.5,
            "y_min": -0.5, "y_max": 0.5,
            "z_min": 0.0, "z_max": 0.5,
            "boundary_slow_zone_m": 0.05,
        },
        "watchdog_timeout_sec": 0.5,
    })


def test_operational_when_armed_and_clean():
    eng = _engine()
    eng.arm()
    eng.advance_startup(StartupPhase.OPERATIONAL)
    state = eng.evaluate(
        joints_rad=[0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
        ee_x=0.2, ee_y=0.2, ee_z=0.3,
        main_current=1.0,
        estop=False,
        serial_ok=True,
        joint_age_sec=0.1,
    )
    assert state.operational is True
    assert state.motion_scale == 1.0


def test_joint_limit_violation_triggers_estop():
    eng = _engine()
    eng.arm()
    state = eng.evaluate(
        joints_rad=[math.radians(200), 1.0, 1.0, 0.0, 0.0, 0.0],
        ee_x=0.2, ee_y=0.2, ee_z=0.3,
        main_current=1.0,
        estop=False,
        serial_ok=True,
        joint_age_sec=0.1,
    )
    assert state.estop_active is True
    assert state.motion_scale == 0.0


def test_workspace_boundary_reduces_motion_scale():
    eng = _engine()
    eng.arm()
    state = eng.evaluate(
        joints_rad=[0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
        ee_x=0.48, ee_y=0.2, ee_z=0.3,
        main_current=1.0,
        estop=False,
        serial_ok=True,
        joint_age_sec=0.1,
    )
    assert 0.0 < state.motion_scale < 1.0


def test_current_overload_critical():
    eng = _engine()
    state = eng.evaluate(
        joints_rad=[0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
        ee_x=0.2, ee_y=0.2, ee_z=0.3,
        main_current=6.0,
        estop=False,
        serial_ok=True,
        joint_age_sec=0.1,
    )
    assert any(v.function.value == "SF-05" for v in state.violations)
