"""Hybrid motion planner  Simple IK + MoveIt 2 (PLAN Phase 3)."""

from __future__ import annotations

import math
import threading
import time
import json
import uuid

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String


class HybridMotionPlanner:
    """Execute Cartesian goals via Simple IK or MoveIt MoveGroup."""

    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self._current_pose: PoseStamped | None = None
        self._motion_status = "idle"

        self._world_frame = node.get_parameter("world_frame").value
        self._planning_mode = node.get_parameter("planning_mode").value
        self._position_tolerance = node.get_parameter("position_tolerance").value
        self._move_timeout = node.get_parameter("move_timeout").value
        self._group_name = node.get_parameter("moveit_group").value
        self._ee_link = node.get_parameter("moveit_ee_link").value
        self._use_twin_validation = node.get_parameter("use_twin_validation").value
        self._use_safety_gate = node.get_parameter("use_safety_gate").value

        self._cartesian_pub = node.create_publisher(PoseStamped, "/visiona/cartesian_command", 10)
        self._twin_validate_pub = node.create_publisher(String, "/visiona/twin/validate_motion", 10)
        self._move_group = ActionClient(node, MoveGroup, "move_action")
        self._twin_lock = threading.Lock()
        self._twin_results: dict[str, bool] = {}
        self._safety_lock = threading.Lock()
        self._safety_status: dict = {"operational": True, "motion_scale": 1.0}

        node.create_subscription(PoseStamped, "/visiona/current_pose", self._on_pose, 10)
        node.create_subscription(String, "/visiona/motion_status", self._on_motion_status, 10)
        node.create_subscription(String, "/uraf/twin/validation", self._on_twin_validation, 10)
        node.create_subscription(String, "/uraf/safety/status", self._on_safety_status, 10)

    def _on_pose(self, msg: PoseStamped):
        with self._lock:
            self._current_pose = msg

    def _on_motion_status(self, msg: String):
        with self._lock:
            self._motion_status = msg.data.strip().lower()

    def _on_twin_validation(self, msg: String):
        try:
            data = json.loads(msg.data)
            request_id = data.get("request_id")
            if request_id:
                with self._twin_lock:
                    self._twin_results[request_id] = bool(data.get("approved", False))
        except (json.JSONDecodeError, TypeError):
            pass

    def _on_safety_status(self, msg: String):
        try:
            with self._safety_lock:
                self._safety_status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            pass

    def move_to(self, x: float, y: float, z: float, timeout: float | None = None) -> bool:
        timeout = timeout or self._move_timeout
        if self._use_safety_gate and not self._safety_allows_motion():
            self._node.get_logger().warn("Safety gate blocked motion")
            return False
        if self._use_twin_validation and not self._validate_twin(x, y, z):
            self._node.get_logger().warn(f"Digital twin rejected motion ({x:.3f},{y:.3f},{z:.3f})")
            return False
        if self._use_moveit():
            ok = self._moveit_move(x, y, z, timeout)
            if ok:
                return True
            self._node.get_logger().warn("MoveIt failed  falling back to Simple IK")
        return self._simple_ik_move(x, y, z, timeout)

    def move_home(self, timeout: float | None = None) -> bool:
        timeout = timeout or self._move_timeout
        if self._use_moveit():
            ok = self._moveit_named_state("home", timeout)
            if ok:
                return True
        return self.move_to(0.0, 0.25, 0.35, timeout)

    def _use_moveit(self) -> bool:
        mode = self._planning_mode.lower()
        if mode == "simple_ik":
            return False
        if mode == "moveit":
            return self._move_group.wait_for_server(timeout_sec=1.0)
        if mode == "auto":
            return self._move_group.wait_for_server(timeout_sec=0.5)
        return False

    def _simple_ik_move(self, x: float, y: float, z: float, timeout: float) -> bool:
        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self._cartesian_pub.publish(msg)
        self._node.get_logger().info(f"IK move ? ({x:.3f},{y:.3f},{z:.3f})")
        return self._wait_for_target(x, y, z, timeout)

    def _moveit_move(self, x: float, y: float, z: float, timeout: float) -> bool:
        if not self._move_group.wait_for_server(timeout_sec=2.0):
            return False

        goal = MoveGroup.Goal()
        goal.request.group_name = self._group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.4
        goal.request.max_acceleration_scaling_factor = 0.4

        constraints = Constraints()
        constraints.name = "cartesian_goal"

        pc = PositionConstraint()
        pc.header.frame_id = self._world_frame
        pc.link_name = self._ee_link
        pc.target_point_offset.x = pc.target_point_offset.y = pc.target_point_offset.z = 0.0
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.015])
        )
        target = PoseStamped()
        target.header.frame_id = self._world_frame
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        target.pose.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(target.pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = self._world_frame
        oc.link_name = self._ee_link
        oc.orientation.w = 1.0
        oc.absolute_x_axis_tolerance = 0.2
        oc.absolute_y_axis_tolerance = 0.2
        oc.absolute_z_axis_tolerance = 0.2
        oc.weight = 0.5
        constraints.orientation_constraints.append(oc)

        goal.request.goal_constraints.append(constraints)

        done = threading.Event()
        result_ok = {"ok": False}

        def on_result(future):
            try:
                result = future.result().result
                result_ok["ok"] = result.error_code.val == MoveItErrorCodes.SUCCESS
            except Exception:
                result_ok["ok"] = False
            done.set()

        def on_goal(future):
            handle = future.result()
            if not handle.accepted:
                done.set()
                return
            handle.get_result_async().add_done_callback(on_result)

        self._move_group.send_goal_async(goal).add_done_callback(on_goal)
        done.wait(timeout=timeout + 5.0)
        if result_ok["ok"]:
            self._wait_for_target(x, y, z, min(timeout, 3.0))
        return result_ok["ok"]

    def _moveit_named_state(self, state_name: str, timeout: float) -> bool:
        if not self._move_group.wait_for_server(timeout_sec=2.0):
            return False

        home_joints = {
            "base_link_joint": 1.57,
            "link_1_shoulder_joint": 1.57,
            "link_2_elbow_joint": 1.57,
            "link_3_wrist_joint": 1.57,
            "link_3_wrist_to_gripper_base_joint": 1.57,
        }

        goal = MoveGroup.Goal()
        goal.request.group_name = self._group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.35

        target = Constraints()
        for jname, val in home_joints.items():
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = val
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            target.joint_constraints.append(jc)
        goal.request.goal_constraints.append(target)

        done = threading.Event()
        result_ok = {"ok": False}

        def on_result(future):
            try:
                result_ok["ok"] = future.result().result.error_code.val == MoveItErrorCodes.SUCCESS
            except Exception:
                pass
            done.set()

        def on_goal(future):
            handle = future.result()
            if handle.accepted:
                handle.get_result_async().add_done_callback(on_result)
            else:
                done.set()

        self._move_group.send_goal_async(goal).add_done_callback(on_goal)
        done.wait(timeout=timeout + 5.0)
        return result_ok["ok"]

    def _safety_allows_motion(self) -> bool:
        with self._safety_lock:
            status = self._safety_status
        if not status:
            return True
        if status.get("estop_active"):
            return False
        scale = float(status.get("motion_scale", 1.0))
        if scale <= 0.0:
            return False
        if status.get("armed") and not status.get("operational"):
            return False
        return True

    def _validate_twin(self, x: float, y: float, z: float, timeout: float = 1.0) -> bool:
        request_id = str(uuid.uuid4())
        payload = json.dumps({"request_id": request_id, "x": x, "y": y, "z": z})
        msg = String()
        msg.data = payload
        self._twin_validate_pub.publish(msg)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._twin_lock:
                if request_id in self._twin_results:
                    return self._twin_results.pop(request_id)
            time.sleep(0.02)
        return True

    def _wait_for_target(self, x: float, y: float, z: float, timeout: float) -> bool:
        deadline = time.time() + timeout
        tol = self._position_tolerance
        while time.time() < deadline:
            with self._lock:
                pose = self._current_pose
                status = self._motion_status
            if pose is not None:
                dx = pose.pose.position.x - x
                dy = pose.pose.position.y - y
                dz = pose.pose.position.z - z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist <= tol and status == "idle":
                    return True
            time.sleep(0.05)
        return False
