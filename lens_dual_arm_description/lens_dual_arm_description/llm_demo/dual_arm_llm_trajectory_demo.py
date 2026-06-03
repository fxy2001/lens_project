from __future__ import annotations

import hashlib
import json
import math
import os
import sys
from pathlib import Path
import threading
import time
import tkinter as tk
from dataclasses import dataclass

import mujoco
import mujoco.viewer
import numpy as np

# Ensure we can import sibling modules when executed from any cwd.
_THIS_DIR = Path(__file__).resolve().parent
_PKG_ROOT = _THIS_DIR.parent  # lens_dual_arm_description/
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from joint_limits import (
    JointLimitError,
    limits_for_joint_names,
    validate_joint_positions,
    validate_joint_trajectory,
)

from left_arm_ik_slider import (
    DEFAULT_MODEL_PATH,
    IkConfig,
    LEFT_EE_BODY,
    LEFT_JOINT_NAMES,
    Ros2BridgeConfig,
    Ros2JointCommandPublisher,
    _joint_ids,
    _joint_indices,
    _left_actuator_ids,
    rpy_to_quat_wxyz,
    solve_left_ik_multi,
    solve_left_ik_once,
)
from right_arm_ik_slider import (
    RIGHT_EE_BODY,
    RIGHT_JOINT_NAMES,
    _right_actuator_ids,
    solve_right_ik_multi,
    solve_right_ik_once,
)


# Fixed drawing plane/range inherited from dual_arm_ik_draw_circle_square.py
LEFT_DRAW_ORIGIN = np.array([0.48, 0.24, 0.42], dtype=float)
RIGHT_DRAW_ORIGIN = np.array([0.48, -0.24, 0.42], dtype=float)
DRAW_PLANE_RPY_DEG = np.array([0.0, 0.0, 0.0], dtype=float)
DRAW_FRAME_SCALE = 1.0
# Use the square demo range as strict drawing range (0.12 half-side).
DRAW_RANGE_WIDTH_M = 0.24
DRAW_RANGE_HEIGHT_M = 0.24

# Flange / EE body Z axis perpendicular to robot base XY plane (normal // world ±Z).
# Verified in MuJoCo: [0,-90,0] is horizontal; [0,180,0] is tool-down; [0,0,0] is tool-up.
FLANGE_RPY_PERP_XY_DOWN_DEG = np.array([0.0, 180.0, 0.0], dtype=float)
FLANGE_RPY_PERP_XY_UP_DEG = np.array([0.0, 0.0, 0.0], dtype=float)
# Flange Z // world ±Y  →  flange ⊥ XZ plane (normal // Y).
FLANGE_RPY_PERP_XZ_POS_Y_DEG = np.array([-90.0, 0.0, 0.0], dtype=float)
FLANGE_RPY_PERP_XZ_NEG_Y_DEG = np.array([90.0, 0.0, 0.0], dtype=float)
# Flange Z // world ±X  →  flange ⊥ YZ plane (normal // X).
FLANGE_RPY_PERP_YZ_POS_X_DEG = np.array([0.0, 90.0, 0.0], dtype=float)
FLANGE_RPY_PERP_YZ_NEG_X_DEG = np.array([0.0, -90.0, 0.0], dtype=float)


@dataclass
class CommandRuntime:
    command_id: str
    arm: str
    mode: str
    sample_hz: float
    left_world_points: np.ndarray
    right_world_points: np.ndarray
    pen_down: np.ndarray
    duration_s: float
    repeat: int
    ee_quat: np.ndarray
    tool_offset: np.ndarray
    lock_flange_perp_xy: bool = False
    flange_lock_mode: str | None = None  # "xy" | "xz" | "yz" | None


@dataclass
class JointTrajectory:
    joint_names: list[str]
    q_refs: list[np.ndarray]
    hz: float


@dataclass
class ExecuteResult:
    total_s: float
    motion_s: float
    hold_s: float
    ros_init_s: float
    resample_s: float


@dataclass
class PipelineResult:
    traj: JointTrajectory
    runtime: CommandRuntime
    command: dict
    execute: ExecuteResult | None
    timing: "RunTiming"


class RunTiming:
    """Wall-clock profiler: command accept → robot motion complete."""

    def __init__(self, *, threshold_s: float | None = None) -> None:
        if threshold_s is None:
            threshold_s = float(os.environ.get("LENS_TIMING_THRESHOLD_S", "1.0"))
        self.threshold_s = max(0.0, float(threshold_s))
        self.t_command_start = time.perf_counter()
        self.phases: list[tuple[str, float]] = []
        self.meta: dict[str, object] = {}

    def record(self, name: str, duration_s: float) -> None:
        self.phases.append((name, max(0.0, float(duration_s))))

    class _Span:
        def __init__(self, timing: "RunTiming", name: str) -> None:
            self._timing = timing
            self._name = name
            self._t0 = 0.0

        def __enter__(self) -> "RunTiming._Span":
            self._t0 = time.perf_counter()
            return self

        def __exit__(self, *_exc: object) -> None:
            self._timing.record(self._name, time.perf_counter() - self._t0)

    def span(self, name: str) -> _Span:
        return RunTiming._Span(self, name)

    def elapsed_since_command(self) -> float:
        return time.perf_counter() - self.t_command_start

    def motion_done_total_s(self) -> float:
        """Command start → last joint command published (incl. hold)."""
        v = self.meta.get("motion_done_total_s")
        return float(v) if v is not None else self.elapsed_since_command()

    def report_dict(self) -> dict[str, object]:
        total = self.elapsed_since_command()
        motion_s = float(self.meta.get("execute.motion_s", 0.0))
        hold_s = float(self.meta.get("execute.hold_s", 0.0))
        plan_s = sum(d for n, d in self.phases if n.startswith("plan."))
        exec_overhead = sum(
            d
            for n, d in self.phases
            if n.startswith("execute.")
            and n not in ("execute.motion", "execute.hold", "execute.total")
        )
        slow = [(n, d) for n, d in self.phases if d >= self.threshold_s]
        return {
            "command_to_motion_done_s": round(self.motion_done_total_s(), 4),
            "wall_clock_total_s": round(total, 4),
            "motion_s": round(motion_s, 4),
            "hold_s": round(hold_s, 4),
            "plan_s": round(plan_s, 4),
            "execute_overhead_s": round(exec_overhead, 4),
            "threshold_s": self.threshold_s,
            "phases": [(n, round(d, 4)) for n, d in self.phases],
            "slow_phases": [(n, round(d, 4)) for n, d in slow],
        }

    def print_report(self) -> None:
        r = self.report_dict()
        cmd_id = self.meta.get("command_id", "")
        id_suffix = f" command_id={cmd_id}" if cmd_id else ""
        print(f"[Timing] ========== 运行耗时报告{id_suffix} ==========")
        print(
            f"[Timing] 命令→动作完成: {r['command_to_motion_done_s']:.3f}s "
            f"(motion={r['motion_s']:.3f}s + hold={r['hold_s']:.3f}s + "
            f"plan={r['plan_s']:.3f}s + exec_overhead={r['execute_overhead_s']:.3f}s)"
        )
        print(f"[Timing] 进程总耗时(wall): {r['wall_clock_total_s']:.3f}s")
        print("[Timing] --- 各阶段 ---")
        for name, dur in self.phases:
            mark = "  *** >1s ***" if dur >= self.threshold_s else ""
            print(f"[Timing]   {name:<28} {dur:.3f}s{mark}")
        slow = r["slow_phases"]
        if slow:
            print(f"[Timing] --- 超过 {self.threshold_s:.1f}s 的阶段 ---")
            for name, dur in slow:
                print(f"[Timing]   {name:<28} {dur:.3f}s")
        else:
            print(f"[Timing] --- 无超过 {self.threshold_s:.1f}s 的阶段 ---")
        print("[Timing] =====================================")


def _rotmat_from_rpy_deg(shape_rpy_deg: np.ndarray) -> np.ndarray:
    quat = rpy_to_quat_wxyz(np.deg2rad(shape_rpy_deg))
    rot = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rot, quat.astype(np.float64))
    return rot.reshape(3, 3)


def _rotmat_world_from_quat_wxyz(q: np.ndarray) -> np.ndarray:
    rot = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rot, q.astype(np.float64))
    return rot.reshape(3, 3)


def _plane_rotmat() -> np.ndarray:
    return _rotmat_from_rpy_deg(DRAW_PLANE_RPY_DEG)


def _plane_normal_world() -> np.ndarray:
    """Unit normal of the local path plane in world frame (local +Z axis)."""
    n = _plane_rotmat()[2, :].astype(float)
    n_norm = float(np.linalg.norm(n))
    return n / max(n_norm, 1e-12)


def _flange_pos_from_tip(
    tip_pos: np.ndarray, ee_quat: np.ndarray, tool_offset: np.ndarray
) -> np.ndarray:
    """Path IR world points are tool-tip targets; IK solves for the EE body (flange)."""
    off = np.asarray(tool_offset, dtype=float).reshape(3)
    if float(np.linalg.norm(off)) < 1e-12:
        return np.asarray(tip_pos, dtype=float).copy()
    R = _rotmat_world_from_quat_wxyz(ee_quat)
    return np.asarray(tip_pos, dtype=float) - R @ off


def _fallback_parse_path_ir() -> dict:
    return {
        "version": "2.0",
        "command_id": "cmd_fallback_triangle",
        "command_type": "draw_path",
        "target": {"arm": "left", "mode": "ee_pose"},
        "frame": {
            "type": "local_2d_on_3d_plane",
            "unit": "meter",
            "origin": LEFT_DRAW_ORIGIN.tolist(),
            "plane_rpy_deg": DRAW_PLANE_RPY_DEG.tolist(),
            "scale": 1.0,
        },
        "path": {
            "coordinate_mode": "normalized",
            "normalize_box_m": {"width": DRAW_RANGE_WIDTH_M, "height": DRAW_RANGE_HEIGHT_M},
            "commands": [
                {"cmd": "M", "p": [-0.5, -0.5]},
                {"cmd": "L", "p": [0.5, -0.5]},
                {"cmd": "L", "p": [0.0, 0.5]},
                {"cmd": "Z"},
            ],
        },
        "motion": {
            "duration_s": 5.0,
            "repeat": 1,
            "speed_mode": "constant_path_speed",
            "sample_hz": 120,
            "lift_between_subpaths": False,
            "lift_height_m": 0.03,
        },
        "end_effector": {
            "rpy_deg": [0.0, 180.0, 0.0],
            "tool_offset": [0.0, 0.0, -0.08],
            "lock_perpendicular_to_xy": True,
        },
        "safety": {
            "workspace_min": [0.20, -0.50, 0.15],
            "workspace_max": [0.80, 0.50, 0.80],
            "max_linear_speed": 0.25,
            "max_acc": 0.8,
            "allow_partial": False,
        },
    }


def _to_vec2(v: object, name: str) -> np.ndarray:
    if not isinstance(v, list) or len(v) != 2:
        raise ValueError(f"{name} must be [x, y]")
    return np.asarray(v, dtype=float)


def _to_vec3(v: object, name: str) -> np.ndarray:
    if not isinstance(v, list) or len(v) != 3:
        raise ValueError(f"{name} must be [x, y, z]")
    return np.asarray(v, dtype=float)


def _sample_line(p0: np.ndarray, p1: np.ndarray, n: int) -> list[np.ndarray]:
    return [p0 * (1.0 - t) + p1 * t for t in np.linspace(0.0, 1.0, n)]


def _sample_quad(p0: np.ndarray, c: np.ndarray, p1: np.ndarray, n: int) -> list[np.ndarray]:
    out = []
    for t in np.linspace(0.0, 1.0, n):
        out.append(((1 - t) ** 2) * p0 + 2 * (1 - t) * t * c + (t**2) * p1)
    return out


def _sample_cubic(p0: np.ndarray, c1: np.ndarray, c2: np.ndarray, p1: np.ndarray, n: int) -> list[np.ndarray]:
    out = []
    for t in np.linspace(0.0, 1.0, n):
        out.append(((1 - t) ** 3) * p0 + 3 * ((1 - t) ** 2) * t * c1 + 3 * (1 - t) * (t**2) * c2 + (t**3) * p1)
    return out


def _sample_arc(center: np.ndarray, radius: float, start_deg: float, end_deg: float, clockwise: bool, n: int) -> list[np.ndarray]:
    start = math.radians(start_deg)
    end = math.radians(end_deg)
    delta = end - start
    if clockwise and delta > 0:
        delta -= 2 * math.pi
    if (not clockwise) and delta < 0:
        delta += 2 * math.pi
    out: list[np.ndarray] = []
    for t in np.linspace(0.0, 1.0, n):
        a = start + delta * t
        out.append(center + np.array([radius * math.cos(a), radius * math.sin(a)], dtype=float))
    return out


def _parse_and_validate_command(command: dict) -> None:
    if command.get("version") != "2.0":
        raise ValueError("UNSUPPORTED_VERSION")
    if command.get("command_type") != "draw_path":
        raise ValueError("UNSUPPORTED_COMMAND")
    for k in ("target", "frame", "path", "motion", "end_effector", "safety"):
        if k not in command:
            raise ValueError(f"INVALID_JSON missing={k}")
    if command["target"].get("arm") not in ("left", "right", "both"):
        raise ValueError("INVALID_JSON target.arm")
    if command["target"].get("mode") not in ("ee_position", "ee_pose"):
        raise ValueError("INVALID_JSON target.mode")
    if command["frame"].get("type") != "local_2d_on_3d_plane":
        raise ValueError("INVALID_JSON frame.type")
    if command["frame"].get("unit") != "meter":
        raise ValueError("INVALID_JSON frame.unit")
    if command["path"].get("coordinate_mode") not in ("normalized", "meter"):
        raise ValueError("INVALID_JSON path.coordinate_mode")
    if command["motion"].get("speed_mode") != "constant_path_speed":
        raise ValueError("INVALID_JSON motion.speed_mode")


def _sample_path_2d(path_cfg: dict, motion_cfg: dict) -> tuple[np.ndarray, np.ndarray]:
    cmds = path_cfg.get("commands", [])
    if not isinstance(cmds, list) or not cmds:
        raise ValueError("INVALID_JSON path.commands")
    coord_mode = path_cfg.get("coordinate_mode", "normalized")
    box = path_cfg.get("normalize_box_m", {"width": 0.24, "height": 0.18})
    width = float(box.get("width", 0.24))
    height = float(box.get("height", 0.18))
    sample_hz = float(motion_cfg.get("sample_hz", 120))
    duration_s = float(motion_cfg.get("duration_s", 5.0))
    nominal_n = max(60, int(sample_hz * duration_s))
    seg_n = max(8, nominal_n // max(1, len(cmds)))

    points: list[np.ndarray] = []
    pen_down: list[bool] = []
    cur: np.ndarray | None = None
    subpath_start: np.ndarray | None = None
    after_move = False

    def push_segment(seg: list[np.ndarray], down: bool) -> None:
        for i, p in enumerate(seg):
            if points and i == 0 and np.linalg.norm(points[-1] - p) < 1e-12:
                continue
            points.append(p)
            pen_down.append(down)

    for item in cmds:
        if not isinstance(item, dict):
            raise ValueError("INVALID_JSON commands item")
        c = str(item.get("cmd", "")).upper()
        if c not in ("M", "L", "Q", "C", "A", "Z"):
            raise ValueError("UNSUPPORTED_PATH_CMD")
        if c == "M":
            cur = _to_vec2(item.get("p"), "M.p")
            subpath_start = cur.copy()
            push_segment([cur], False)
            after_move = True
            continue
        if cur is None:
            raise ValueError("INVALID_JSON path must start with M")
        if c == "L":
            p1 = _to_vec2(item.get("p"), "L.p")
            push_segment(_sample_line(cur, p1, seg_n), not after_move)
            cur = p1
        elif c == "Q":
            c1 = _to_vec2(item.get("c"), "Q.c")
            p1 = _to_vec2(item.get("p"), "Q.p")
            push_segment(_sample_quad(cur, c1, p1, seg_n), not after_move)
            cur = p1
        elif c == "C":
            c1 = _to_vec2(item.get("c1"), "C.c1")
            c2 = _to_vec2(item.get("c2"), "C.c2")
            p1 = _to_vec2(item.get("p"), "C.p")
            push_segment(_sample_cubic(cur, c1, c2, p1, seg_n), not after_move)
            cur = p1
        elif c == "A":
            center = _to_vec2(item.get("center"), "A.center")
            radius = float(item.get("radius", 0.0))
            if coord_mode == "normalized":
                radius *= min(width, height)
            start_deg = float(item.get("start_deg", 0.0))
            end_deg = float(item.get("end_deg", 360.0))
            clockwise = bool(item.get("clockwise", False))
            push_segment(_sample_arc(center, radius, start_deg, end_deg, clockwise, seg_n), not after_move)
            cur = points[-1]
        elif c == "Z":
            if subpath_start is None:
                raise ValueError("INVALID_JSON Z without subpath")
            push_segment(_sample_line(cur, subpath_start, seg_n), not after_move)
            cur = subpath_start.copy()
        after_move = False

    pts = np.asarray(points, dtype=float)
    if coord_mode == "normalized":
        pts[:, 0] *= width
        pts[:, 1] *= height
    return pts, np.asarray(pen_down, dtype=bool)


def _resample_constant_speed(points: np.ndarray, pen_down: np.ndarray, duration_s: float, sample_hz: float) -> tuple[np.ndarray, np.ndarray]:
    if len(points) < 2:
        raise ValueError("INVALID_JSON path too short")
    d = np.linalg.norm(np.diff(points, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(d)])
    total = float(s[-1])
    if total < 1e-9:
        raise ValueError("INVALID_JSON zero path length")
    n = max(2, int(round(duration_s * sample_hz)))
    s_target = np.linspace(0.0, total, n)
    x = np.interp(s_target, s, points[:, 0])
    y = np.interp(s_target, s, points[:, 1])
    idx = np.searchsorted(s, s_target, side="right") - 1
    idx = np.clip(idx, 0, len(pen_down) - 1)
    return np.stack([x, y], axis=1), pen_down[idx]


def _fit_points_to_draw_range(points_2d: np.ndarray) -> np.ndarray:
    if points_2d.shape[0] < 2:
        raise ValueError("INVALID_JSON path too short")
    pmin = np.min(points_2d, axis=0)
    pmax = np.max(points_2d, axis=0)
    center = 0.5 * (pmin + pmax)
    span = pmax - pmin
    span_x = max(float(span[0]), 1e-9)
    span_y = max(float(span[1]), 1e-9)
    scale = min(DRAW_RANGE_WIDTH_M / span_x, DRAW_RANGE_HEIGHT_M / span_y)
    return (points_2d - center[None, :]) * scale


def _map_2d_to_world(points_2d: np.ndarray, origin: np.ndarray) -> np.ndarray:
    rot = _plane_rotmat()
    pts3 = np.zeros((points_2d.shape[0], 3), dtype=float)
    pts3[:, :2] = points_2d * float(DRAW_FRAME_SCALE)
    return origin[None, :] + (pts3 @ rot.T)


def _check_safety(points_3d: np.ndarray, sample_hz: float, safety: dict) -> None:
    if os.environ.get("LENS_SKIP_SAFETY_CHECK", "1").strip().lower() in ("1", "true", "yes", "on"):
        print("[Safety] skip checks in demo mode.")
        return
    allow_partial = bool(safety.get("allow_partial", False))
    ws_min = _to_vec3(safety.get("workspace_min"), "safety.workspace_min")
    ws_max = _to_vec3(safety.get("workspace_max"), "safety.workspace_max")
    if np.any(points_3d < ws_min[None, :]) or np.any(points_3d > ws_max[None, :]):
        if allow_partial:
            print("[Safety] warning: OUT_OF_WORKSPACE but allow_partial=true, continue planning.")
            return
        raise ValueError("OUT_OF_WORKSPACE")
    dt = 1.0 / sample_hz
    vel = np.linalg.norm(np.diff(points_3d, axis=0), axis=1) / max(dt, 1e-6)
    acc = np.linalg.norm(np.diff(points_3d, n=2, axis=0), axis=1) / max(dt * dt, 1e-6)
    if np.max(vel) > float(safety.get("max_linear_speed", 0.25)):
        raise ValueError("SPEED_TOO_HIGH")
    if acc.size and np.max(acc) > float(safety.get("max_acc", 0.8)):
        raise ValueError("ACC_TOO_HIGH")


def _load_demo_draw_scope_from_env() -> None:
    global LEFT_DRAW_ORIGIN, RIGHT_DRAW_ORIGIN, DRAW_PLANE_RPY_DEG, DRAW_FRAME_SCALE
    global DRAW_RANGE_WIDTH_M, DRAW_RANGE_HEIGHT_M
    LEFT_DRAW_ORIGIN = np.array(
        [
            float(os.environ.get("LENS_DEMO_LEFT_X", str(LEFT_DRAW_ORIGIN[0]))),
            float(os.environ.get("LENS_DEMO_LEFT_Y", str(LEFT_DRAW_ORIGIN[1]))),
            float(os.environ.get("LENS_DEMO_LEFT_Z", str(LEFT_DRAW_ORIGIN[2]))),
        ],
        dtype=float,
    )
    RIGHT_DRAW_ORIGIN = np.array(
        [
            float(os.environ.get("LENS_DEMO_RIGHT_X", str(RIGHT_DRAW_ORIGIN[0]))),
            float(os.environ.get("LENS_DEMO_RIGHT_Y", str(RIGHT_DRAW_ORIGIN[1]))),
            float(os.environ.get("LENS_DEMO_RIGHT_Z", str(RIGHT_DRAW_ORIGIN[2]))),
        ],
        dtype=float,
    )
    DRAW_PLANE_RPY_DEG = np.array(
        [
            float(os.environ.get("LENS_DEMO_SHAPE_ROLL_DEG", str(DRAW_PLANE_RPY_DEG[0]))),
            float(os.environ.get("LENS_DEMO_SHAPE_PITCH_DEG", str(DRAW_PLANE_RPY_DEG[1]))),
            float(os.environ.get("LENS_DEMO_SHAPE_YAW_DEG", str(DRAW_PLANE_RPY_DEG[2]))),
        ],
        dtype=float,
    )
    half = float(os.environ.get("LENS_DEMO_SQUARE_HALF", "0.12"))
    half = max(0.01, half)
    DRAW_RANGE_WIDTH_M = 2.0 * half
    DRAW_RANGE_HEIGHT_M = 2.0 * half
    if os.environ.get("LENS_DEMO_FRAME_SCALE"):
        DRAW_FRAME_SCALE = float(os.environ.get("LENS_DEMO_FRAME_SCALE", "1.0"))


def _apply_frame_from_command(command: dict) -> None:
    """Apply Path IR frame fields (plane_rpy_deg / origin / scale) when env vars do not override."""
    global DRAW_PLANE_RPY_DEG, DRAW_FRAME_SCALE, LEFT_DRAW_ORIGIN, RIGHT_DRAW_ORIGIN
    frame = command.get("frame", {})
    if "scale" in frame and not os.environ.get("LENS_DEMO_FRAME_SCALE"):
        DRAW_FRAME_SCALE = max(1e-9, float(frame["scale"]))
    if "plane_rpy_deg" in frame:
        frame_rpy = _to_vec3(frame["plane_rpy_deg"], "frame.plane_rpy_deg").astype(float)
        rpy = DRAW_PLANE_RPY_DEG.astype(float).copy()
        for env_k, idx in (
            ("LENS_DEMO_SHAPE_ROLL_DEG", 0),
            ("LENS_DEMO_SHAPE_PITCH_DEG", 1),
            ("LENS_DEMO_SHAPE_YAW_DEG", 2),
        ):
            if os.environ.get(env_k):
                continue
            rpy[idx] = float(frame_rpy[idx])
        DRAW_PLANE_RPY_DEG = rpy

    if "origin" not in frame:
        return
    origin = _to_vec3(frame["origin"], "frame.origin").astype(float)
    arm = str(command.get("target", {}).get("arm", "both"))
    if arm in ("left", "both") and not any(
        os.environ.get(k) for k in ("LENS_DEMO_LEFT_X", "LENS_DEMO_LEFT_Y", "LENS_DEMO_LEFT_Z")
    ):
        LEFT_DRAW_ORIGIN = origin.copy()
    if arm in ("right", "both") and not any(
        os.environ.get(k) for k in ("LENS_DEMO_RIGHT_X", "LENS_DEMO_RIGHT_Y", "LENS_DEMO_RIGHT_Z")
    ):
        right_origin = origin.copy()
        if arm == "both":
            right_origin[1] = -float(origin[1])
        RIGHT_DRAW_ORIGIN = right_origin


def _env_truthy(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


def _env_falsy(name: str) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return False
    return raw.strip().lower() in ("0", "false", "no", "off")


def _flange_lock_mode(command: dict) -> str | None:
    """Return orientation lock. Default policy: flange always ⊥ XY (Z // world Z)."""
    if not _env_truthy("LENS_FLANGE_ALLOW_ALT_LOCK"):
        if _env_falsy("LENS_FLANGE_PERP_XY"):
            return None
        return "xy"
    ee = command.get("end_effector", {})
    if isinstance(ee, dict):
        if bool(ee.get("lock_perpendicular_to_yz", False)):
            return "yz"
        if bool(ee.get("lock_perpendicular_to_xz", False)):
            return "xz"
        if bool(ee.get("lock_perpendicular_to_xy", False)):
            return "xy"
    if _env_truthy("LENS_FLANGE_PERP_YZ"):
        return "yz"
    if _env_truthy("LENS_FLANGE_PERP_XZ"):
        return "xz"
    if _env_truthy("LENS_FLANGE_PERP_XY"):
        return "xy"
    return None


def _flange_xy_tool_down() -> bool:
    """True when global ⊥XY policy targets tool vertical down (Z // world -Z)."""
    return os.environ.get("LENS_FLANGE_XY_NORMAL", "down").strip().lower() not in (
        "up",
        "+z",
        "pos",
        "positive",
    )


def _enforce_default_flange_xy_lock(command: dict) -> None:
    """Normalize Path IR: flange ⊥ XY, tool vertical down unless LENS_FLANGE_XY_NORMAL=up."""
    if _env_truthy("LENS_FLANGE_ALLOW_ALT_LOCK") or _env_falsy("LENS_FLANGE_PERP_XY"):
        return
    ee = command.get("end_effector")
    if not isinstance(ee, dict):
        ee = {}
        command["end_effector"] = ee
    had_alt = bool(ee.get("lock_perpendicular_to_yz")) or bool(ee.get("lock_perpendicular_to_xz"))
    ee["lock_perpendicular_to_xy"] = True
    ee["lock_perpendicular_to_xz"] = False
    ee["lock_perpendicular_to_yz"] = False
    req_rpy = ee.get("rpy_deg")
    ee["rpy_deg"] = _flange_rpy_perpendicular_to_xy_deg().tolist()
    if had_alt:
        print(
            "[EE] global tool-down policy: cleared lock_perpendicular_to_xz/yz "
            "(set LENS_FLANGE_ALLOW_ALT_LOCK=1 to use other planes)"
        )
    if req_rpy is not None:
        req = np.asarray(_to_vec3(req_rpy, "end_effector.rpy_deg"), dtype=float)
        if float(np.linalg.norm(req - ee["rpy_deg"])) > 0.5:
            print(f"[EE] tool-down policy: rpy_deg {req.tolist()} → {ee['rpy_deg']}")


def _flange_lock_plane_label(lock_mode: str | None) -> str:
    return {"xy": "XY", "xz": "XZ", "yz": "YZ"}.get(lock_mode or "", "?")


def _flange_lock_world_axis(lock_mode: str | None) -> str:
    """World axis that flange body Z should align with when locked."""
    return {"xy": "z", "xz": "y", "yz": "x"}.get(lock_mode or "", "z")


def _flange_orientation_locked(command: dict) -> bool:
    return _flange_lock_mode(command) is not None


def _lock_flange_perp_xy_enabled(command: dict) -> bool:
    """Backward-compatible alias."""
    return _flange_lock_mode(command) == "xy"


def _flange_rpy_perpendicular_to_yz_deg() -> np.ndarray:
    """RPY (deg) so EE body Z is parallel to world ±X (flange ⊥ YZ plane)."""
    override = os.environ.get("LENS_FLANGE_RPY_DEG", "").strip()
    if override:
        parts = [float(x) for x in override.replace(",", " ").split()]
        if len(parts) == 3:
            print(f"[EE] LENS_FLANGE_RPY_DEG calibration: {parts}")
            return np.asarray(parts, dtype=float)
    sign = os.environ.get("LENS_FLANGE_YZ_NORMAL", "+x").strip().lower()
    if sign in ("-x", "neg", "negative"):
        return FLANGE_RPY_PERP_YZ_NEG_X_DEG.copy()
    return FLANGE_RPY_PERP_YZ_POS_X_DEG.copy()


def _flange_rpy_perpendicular_to_xz_deg() -> np.ndarray:
    """RPY (deg) so EE body Z is parallel to world ±Y (flange ⊥ XZ plane)."""
    override = os.environ.get("LENS_FLANGE_RPY_DEG", "").strip()
    if override:
        parts = [float(x) for x in override.replace(",", " ").split()]
        if len(parts) == 3:
            print(f"[EE] LENS_FLANGE_RPY_DEG calibration: {parts}")
            return np.asarray(parts, dtype=float)
    sign = os.environ.get("LENS_FLANGE_XZ_NORMAL", "+y").strip().lower()
    if sign in ("-y", "neg", "negative", "down"):
        return FLANGE_RPY_PERP_XZ_NEG_Y_DEG.copy()
    return FLANGE_RPY_PERP_XZ_POS_Y_DEG.copy()


def _flange_rpy_perpendicular_to_xy_deg() -> np.ndarray:
    """RPY (deg) so EE body Z is parallel to world Z (flange ⊥ horizontal XY plane)."""
    override = os.environ.get("LENS_FLANGE_RPY_DEG", "").strip()
    if override:
        parts = [float(x) for x in override.replace(",", " ").split()]
        if len(parts) == 3:
            print(f"[EE] LENS_FLANGE_RPY_DEG calibration: {parts}")
            return np.asarray(parts, dtype=float)
    sign = os.environ.get("LENS_FLANGE_XY_NORMAL", "down").strip().lower()
    if sign in ("up", "+z", "pos", "positive"):
        return FLANGE_RPY_PERP_XY_UP_DEG.copy()
    return FLANGE_RPY_PERP_XY_DOWN_DEG.copy()


def _locked_flange_rpy_deg(command: dict) -> np.ndarray:
    mode = _flange_lock_mode(command)
    if mode == "yz":
        return _flange_rpy_perpendicular_to_yz_deg()
    if mode == "xz":
        return _flange_rpy_perpendicular_to_xz_deg()
    if mode == "xy":
        return _flange_rpy_perpendicular_to_xy_deg()
    raise ValueError("internal: locked rpy requested without lock mode")


def _resolve_end_effector_from_command(command: dict) -> tuple[np.ndarray, np.ndarray]:
    """Return (ee_quat_wxyz, tool_offset). Optionally lock flange orientation."""
    ee = command.get("end_effector", {})
    if not isinstance(ee, dict):
        ee = {}
    tool_offset = _to_vec3(ee.get("tool_offset", [0.0, 0.0, -0.08]), "end_effector.tool_offset")
    lock_mode = _flange_lock_mode(command)
    if lock_mode:
        rpy_deg = _locked_flange_rpy_deg(command)
        if "rpy_deg" in ee:
            req = np.asarray(_to_vec3(ee["rpy_deg"], "end_effector.rpy_deg"), dtype=float)
            if float(np.linalg.norm(req - rpy_deg)) > 0.5:
                plane = _flange_lock_plane_label(lock_mode)
                print(
                    f"[EE] flange ⊥ {plane}: override rpy_deg {req.tolist()} → {rpy_deg.tolist()}"
                )
    else:
        rpy_deg = _to_vec3(ee.get("rpy_deg", [0.0, 180.0, 0.0]), "end_effector.rpy_deg")
    ee_quat = rpy_to_quat_wxyz(np.deg2rad(rpy_deg.astype(float)))
    if lock_mode:
        plane = _flange_lock_plane_label(lock_mode)
        if lock_mode == "xy" and _flange_xy_tool_down():
            print(
                f"[EE] tool vertical DOWN locked: rpy_deg={rpy_deg.tolist()} "
                f"(flange Z // world -Z, ⊥ XY; calibrate real robot with LENS_FLANGE_RPY_DEG)"
            )
        else:
            axis = _flange_lock_world_axis(lock_mode)
            print(
                f"[EE] flange ⊥ {plane} locked: rpy_deg={rpy_deg.tolist()} "
                f"(body Z // world {axis.upper()}; calibrate with LENS_FLANGE_RPY_DEG if needed)"
            )
    return ee_quat, tool_offset


def _joint_limit_margin_rad() -> float:
    return max(0.0, float(os.environ.get("LENS_JOINT_LIMIT_MARGIN_RAD", "0.0")))


def _enforce_joint_limits_enabled() -> bool:
    return _env_truthy("LENS_ENFORCE_JOINT_LIMITS", True)


def _validate_traj_joint_limits(traj: JointTrajectory, *, context: str) -> None:
    if not _enforce_joint_limits_enabled():
        return
    try:
        validate_joint_trajectory(
            traj.joint_names,
            traj.q_refs,
            margin_rad=_joint_limit_margin_rad(),
        )
    except JointLimitError as exc:
        print(f"[JointLimit] {context} rejected: {exc}")
        raise


def _validate_exec_joint_positions(
    joint_names: list[str],
    q: np.ndarray,
    *,
    tag: str,
) -> None:
    if not _enforce_joint_limits_enabled():
        return
    try:
        validate_joint_positions(
            joint_names,
            np.asarray(q, dtype=float),
            margin_rad=_joint_limit_margin_rad(),
        )
    except JointLimitError as exc:
        print(f"[JointLimit] execute {tag} rejected: {exc}")
        raise


def _has_display() -> bool:
    return bool(os.environ.get("DISPLAY", "").strip())


def _apply_runtime_defaults() -> None:
    """Default to fast plan + cache; auto-skip GUI when headless."""
    os.environ.setdefault("LENS_FLANGE_PERP_XY", "1")
    os.environ.setdefault("LENS_FLANGE_XY_NORMAL", "down")
    os.environ.setdefault("LENS_ENFORCE_TOOL_DOWN", "1")
    os.environ.setdefault("LENS_MAX_TOTAL_S", "10")
    os.environ.setdefault("LENS_PLAN_CACHE", "1")
    os.environ.setdefault(
        "LENS_PATH_IR_PLAN_CACHE_DIR",
        str(Path.home() / ".lens_path_ir_plan_cache"),
    )
    if not _env_falsy("LENS_SLOW_PLAN"):
        os.environ.setdefault("LENS_FAST_EXECUTE", "1")
        os.environ.setdefault("LENS_IK_WARM_START", "1")
        os.environ.setdefault("LENS_IK_START_AT_HOME", "1")
        os.environ.setdefault("LENS_SKIP_IK_GUARD", "1")
    if not _has_display():
        os.environ.setdefault("LENS_AUTO_EXECUTE", "1")
    default_ir = _THIS_DIR / "ir.json"
    if not os.environ.get("LENS_PATH_IR_FILE", "").strip() and not os.environ.get(
        "LENS_PATH_IR_JSON", ""
    ).strip():
        if default_ir.is_file():
            os.environ.setdefault("LENS_PATH_IR_FILE", str(default_ir))


def _should_auto_execute() -> bool:
    """Skip MuJoCo preview / confirm UI and publish trajectory to real robot."""
    return _env_truthy("LENS_AUTO_EXECUTE") or _env_truthy("LENS_SKIP_PREVIEW")


def _fast_mode_enabled() -> bool:
    """Fast IK planning for low-latency real-robot execution (CPU; not BPU)."""
    if _env_falsy("LENS_SLOW_PLAN"):
        return False
    return _env_truthy("LENS_FAST_EXECUTE", default=True)


def _stable_command_key(command: dict) -> str:
    s = json.dumps(command, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(s.encode("utf-8")).hexdigest()


def _ultra_time_budget() -> bool:
    """End-to-end wall budget ≤ ~1s (command accept → motion done on robot)."""
    return float(os.environ.get("LENS_MAX_TOTAL_S", "10")) <= 1.05


def _prepare_fast_plan_command(command: dict) -> dict:
    """Cap trajectory resolution and motion duration to fit LENS_MAX_TOTAL_S budget."""
    cmd = json.loads(json.dumps(command))
    _enforce_default_flange_xy_lock(cmd)
    if not _fast_mode_enabled():
        return cmd
    motion = cmd.get("motion")
    if not isinstance(motion, dict):
        return cmd

    max_total_s = float(os.environ.get("LENS_MAX_TOTAL_S", "10"))
    requested_duration = float(motion.get("duration_s", 5.0))

    # Reserve wall time for non-motion phases so plan+motion+hold fits max_total_s.
    ultra = _ultra_time_budget()
    tight_budget = max_total_s <= 1.5
    if ultra:
        ros_reserve = float(os.environ.get("LENS_EXEC_ROS_RESERVE_S", "0.08"))
        hold_reserve = float(os.environ.get("LENS_EXEC_HOLD_S", "0.02"))
        plan_reserve = float(os.environ.get("LENS_PLAN_RESERVE_S", "0.42"))
        misc_reserve = float(os.environ.get("LENS_MISC_RESERVE_S", "0.02"))
        min_motion_s = float(os.environ.get("LENS_MIN_MOTION_S", "0.35"))
    elif tight_budget:
        ros_reserve = float(os.environ.get("LENS_EXEC_ROS_RESERVE_S", "0.10"))
        hold_reserve = float(os.environ.get("LENS_EXEC_HOLD_S", "0.05"))
        plan_reserve = float(os.environ.get("LENS_PLAN_RESERVE_S", "0.25"))
        misc_reserve = float(os.environ.get("LENS_MISC_RESERVE_S", "0.03"))
        min_motion_s = float(os.environ.get("LENS_MIN_MOTION_S", "0.30"))
    else:
        ros_reserve = float(os.environ.get("LENS_EXEC_ROS_RESERVE_S", "0.05"))
        hold_reserve = float(os.environ.get("LENS_EXEC_HOLD_S", "0.6"))
        plan_reserve = float(os.environ.get("LENS_PLAN_RESERVE_S", "0.4"))
        misc_reserve = float(os.environ.get("LENS_MISC_RESERVE_S", "0.03"))
        min_motion_s = float(os.environ.get("LENS_MIN_MOTION_S", "0.5"))

    motion_budget = max_total_s - ros_reserve - hold_reserve - plan_reserve - misc_reserve
    duration_s = min(requested_duration, motion_budget)
    if ultra:
        duration_s = min(duration_s, max(min_motion_s, motion_budget))
    else:
        duration_s = max(min_motion_s, min(requested_duration, motion_budget))
    motion["duration_s"] = duration_s

    exec_reserve_s = float(os.environ.get("LENS_EXEC_RESERVE_S", str(duration_s)))
    plan_budget_s = max(0.1, max_total_s - min(exec_reserve_s, duration_s))

    max_points = int(os.environ.get("LENS_PLAN_MAX_POINTS", "0"))
    explicit_max_points = max_points > 0
    if max_points <= 0:
        if ultra:
            max_points = 3
        else:
            max_points = max(12, min(35, int(round(duration_s * 5.0))))
            if plan_budget_s < 2.0:
                max_points = min(max_points, max(2, int(plan_budget_s * 10)))
    if ultra:
        os.environ.setdefault("LENS_CARTESIAN_LIFT_POINTS", "3")
        os.environ.setdefault("LENS_SETTLE_IK_ITERS", "12")
        os.environ.setdefault("LENS_IK_MAX_ITERS", "10")

    plan_hz_cap = float(os.environ.get("LENS_PLAN_SAMPLE_HZ", "0"))
    if plan_hz_cap <= 0:
        if explicit_max_points:
            plan_hz_cap = max_points / max(duration_s, 1e-3)
        else:
            plan_hz_cap = max(4.0, max_points / max(duration_s, 1e-3))

    src_hz = float(motion.get("sample_hz", 120.0))
    motion["sample_hz"] = min(src_hz, plan_hz_cap)
    motion["repeat"] = 1
    os.environ.setdefault("LENS_PLAN_MAX_POINTS", str(max_points))
    if tight_budget:
        os.environ.setdefault("LENS_EXEC_HOLD_S", str(hold_reserve))
    est_points = int(round(duration_s * float(motion["sample_hz"])))
    if duration_s < requested_duration - 1e-6:
        print(
            f"[FastPlan] duration capped {requested_duration:.2f}s → {duration_s:.2f}s "
            f"(budget={max_total_s:.1f}s)"
        )
    print(
        f"[FastPlan] max_total={max_total_s:.1f}s plan_budget~{plan_budget_s:.1f}s "
        f"motion={duration_s:.2f}s hold~{hold_reserve:.2f}s "
        f"sample_hz={float(motion['sample_hz']):.1f} ~{est_points} IK points"
    )
    return cmd


def _ik_config_for_command(command: dict | None = None) -> IkConfig:
    fast = _fast_mode_enabled()
    lock = command is not None and _flange_orientation_locked(command)
    ultra = _ultra_time_budget()
    if lock and ultra:
        default_iters = 28
        default_pos_tol = "8e-3"
        default_rot_tol = "5e-3"
        default_rot_weight = "10.0"
        default_refine = "18"
        default_wrist_ns = "0.0"
        default_null_gain = "0.015"
    elif lock:
        default_iters = 60
        default_pos_tol = "5e-3"
        default_rot_tol = "2e-3"
        default_rot_weight = "10.0"
        default_refine = "25"
        default_wrist_ns = "0.0"
        default_null_gain = "0.02"
    elif fast:
        default_iters = 12
        default_pos_tol = "8e-3"
        default_rot_tol = "4e-2"
        default_rot_weight = "1.0"
        default_refine = "0"
        default_wrist_ns = "1.0"
        default_null_gain = "0.06"
    else:
        default_iters = 80
        default_pos_tol = "1e-4"
        default_rot_tol = "2e-3"
        default_rot_weight = "1.0"
        default_refine = "0"
        default_wrist_ns = "1.0"
        default_null_gain = "0.06"
    return IkConfig(
        max_iters=int(os.environ.get("LENS_IK_MAX_ITERS", str(default_iters))),
        pos_tol=float(os.environ.get("LENS_IK_POS_TOL", default_pos_tol)),
        rot_tol=float(os.environ.get("LENS_IK_ROT_TOL", default_rot_tol)),
        step_clip=float(os.environ.get("LENS_IK_STEP_CLIP", "0.12" if fast else "0.10")),
        pos_weight=float(os.environ.get("LENS_IK_POS_WEIGHT", "1.0")),
        rot_weight=float(os.environ.get("LENS_IK_ROT_WEIGHT", default_rot_weight)),
        nullspace_gain=float(os.environ.get("LENS_IK_NULLSPACE_GAIN", default_null_gain)),
        wrist_nullspace_scale=float(
            os.environ.get("LENS_IK_WRIST_NULLSPACE_SCALE", default_wrist_ns)
        ),
        orientation_refine_iters=int(
            os.environ.get("LENS_IK_ORIENTATION_REFINE_ITERS", default_refine)
        ),
        refine_pos_hold_weight=float(
            os.environ.get("LENS_IK_REFINE_POS_HOLD_WEIGHT", "5.0")
        ),
    )


def _ik_use_multi_when_locked() -> bool:
    """7-DOF arms: use multi-seed IK when flange orientation is locked (all motors cooperate)."""
    return _env_truthy("LENS_IK_MULTI_WHEN_LOCKED", True)


def _ik_config_from_env() -> IkConfig:
    """Backward-compatible alias when no command dict is available."""
    return _ik_config_for_command(None)


def _flange_body_z_dot_world_axis(model: mujoco.MjModel, data: mujoco.MjData, ee_body_id: int, axis: str) -> float:
    """|dot(flange body Z, world axis)| for axis in x/y/z."""
    R = np.asarray(data.xmat[ee_body_id], dtype=float).reshape(3, 3)
    z_axis = R[:, 2]
    idx = {"x": 0, "y": 1, "z": 2}[axis]
    return float(abs(z_axis[idx]))


def _flange_orientation_alignment(
    model: mujoco.MjModel, data: mujoco.MjData, ee_body_id: int, lock_mode: str | None
) -> float:
    R = np.asarray(data.xmat[ee_body_id], dtype=float).reshape(3, 3)
    z_axis = R[:, 2]
    if lock_mode == "xy" and _flange_xy_tool_down():
        # Tool down: body Z // world -Z  →  -z_axis[2] should be +1.0
        return float(-z_axis[2])
    axis = _flange_lock_world_axis(lock_mode)
    idx = {"x": 0, "y": 1, "z": 2}[axis]
    return float(abs(z_axis[idx]))


def _flange_z_alignment(model: mujoco.MjModel, data: mujoco.MjData, ee_body_id: int) -> float:
    """|dot(flange body Z, world Z)|; 1.0 means flange ⊥ horizontal XY plane."""
    return _flange_body_z_dot_world_axis(model, data, ee_body_id, "z")


def _log_left_arm_traj_summary(traj: JointTrajectory, *, tag: str) -> None:
    """Log all 7 left-arm motor angles; flange ⊥XY needs cooperative 6D IK, not motor 7 alone."""
    idx = {n: i for i, n in enumerate(traj.joint_names)}
    left_cols = [(j + 1, name, idx[name]) for j, name in enumerate(LEFT_JOINT_NAMES) if name in idx]
    if not left_cols:
        return
    lines: list[str] = []
    for frame_i, q in enumerate(traj.q_refs):
        parts = [f"m{motor}={float(q[i]):+.3f}" for motor, _n, i in left_cols]
        lines.append(f"  f{frame_i}: " + " ".join(parts))
    print(
        f"[Joint check:{tag}] left 7-DOF cooperative plan ({len(traj.q_refs)} frames)\n"
        + "\n".join(lines)
    )
    spans: list[str] = []
    min_span = float("inf")
    for motor, name, i in left_cols:
        vals = [float(q[i]) for q in traj.q_refs]
        span = max(vals) - min(vals)
        min_span = min(min_span, span)
        spans.append(f"m{motor}({name.split('_')[1].lower()})={span:.4f}")
    print(f"[Joint check:{tag}] joint spans: " + ", ".join(spans))
    elbow_i = idx.get("Left_Elbow_Pitch_Joint")
    if elbow_i is not None:
        min_elbow = min(float(q[elbow_i]) for q in traj.q_refs)
        if min_elbow < -1.5:
            print(
                f"[Joint check:{tag}] WARN elbow near limit {min_elbow:+.3f} rad — "
                f"sign/calibration or stale cache; rm ~/.lens_path_ir_plan_cache/"
            )
    if len(traj.q_refs) >= 2 and min_span < 0.015:
        print(
            f"[Joint check:{tag}] WARN all motors span <0.015 rad — "
            f"increase LENS_IK_MAX_ITERS / LENS_IK_ROT_WEIGHT or duration_s; "
            f"⊥XY needs shoulder+elbow+wrist together"
        )


def _log_traj_flange_alignment(
    *,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    traj: JointTrajectory,
    arm: str,
    left_q_idx: np.ndarray,
    right_q_idx: np.ndarray,
    left_ee_id: int,
    right_ee_id: int,
    tag: str,
    lock_mode: str | None,
) -> None:
    ee_id = left_ee_id if arm == "left" else right_ee_id
    n_left = len(left_q_idx)
    align: list[float] = []
    for q in traj.q_refs:
        data.qpos[left_q_idx] = q[:n_left]
        data.qpos[right_q_idx] = q[n_left:]
        mujoco.mj_fwdPosition(model, data)
        align.append(_flange_orientation_alignment(model, data, ee_id, lock_mode))
    if not align:
        return
    a = np.asarray(align, dtype=float)
    ok = float(a.min()) >= 0.92
    level = "OK" if ok else "WARN"
    if lock_mode == "xy" and _flange_xy_tool_down():
        metric = "tool DOWN |dot(z,-world_z)| = -body_z.z"
    else:
        plane = _flange_lock_plane_label(lock_mode)
        axis = _flange_lock_world_axis(lock_mode)
        metric = f"flange Z ⊥ {plane} |dot(z,world_{axis})|"
    print(
        f"[EE check:{tag}] {metric} "
        f"min={a.min():.3f} max={a.max():.3f} mean={a.mean():.3f} "
        f"(want ≥0.92) [{level}]"
    )
    if not ok:
        print(
            "[EE check] tool-down / orientation not met in plan; try: rm plan cache, "
            "LENS_MAX_TOTAL_S=3, LENS_IK_MAX_ITERS=80, or LENS_FLANGE_RPY_DEG on real robot"
        )
        if _env_truthy("LENS_ENFORCE_TOOL_DOWN", True) and lock_mode == "xy":
            raise ValueError(
                "TOOL_DOWN_ORIENTATION_FAILED: planned trajectory does not keep flange vertical down"
            )


def _settle_active_arm_orientation(
    *,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: str,
    active_q_idx: np.ndarray,
    active_ee_id: int,
    active_q_cur: np.ndarray,
    left_q_idx: np.ndarray,
    right_q_idx: np.ndarray,
    left_q_cur: np.ndarray,
    right_q_cur: np.ndarray,
    active_q_min: np.ndarray,
    active_q_max: np.ndarray,
    ee_quat: np.ndarray,
    solve_ik,
    cfg: IkConfig,
    lock_mode: str | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Hold EE position, solve IK for target orientation (before Cartesian lift/path)."""
    data.qpos[left_q_idx] = left_q_cur
    data.qpos[right_q_idx] = right_q_cur
    mujoco.mj_fwdPosition(model, data)
    hold_pos = data.xpos[active_ee_id].copy()
    align_before = _flange_orientation_alignment(model, data, active_ee_id, lock_mode)
    min_align = float(os.environ.get("LENS_SETTLE_ORIENT_MIN", "0.88"))
    if align_before >= min_align:
        return active_q_cur, left_q_cur, right_q_cur
    settle_iters = int(
        os.environ.get("LENS_SETTLE_IK_ITERS", "12" if _ultra_time_budget() else "80")
    )
    settle_cfg = IkConfig(
        max_iters=settle_iters if _ultra_time_budget() else max(cfg.max_iters, settle_iters),
        pos_tol=float(os.environ.get("LENS_SETTLE_POS_TOL", "0.04")),
        rot_tol=min(cfg.rot_tol, float(os.environ.get("LENS_SETTLE_ROT_TOL", "0.001"))),
        step_clip=cfg.step_clip,
    )
    active_q_cur, err, _, cond = solve_ik(
        model=model,
        data=data,
        q_idx=active_q_idx,
        ee_body_id=active_ee_id,
        q_init=active_q_cur,
        target_pos=hold_pos,
        target_quat=ee_quat,
        q_min=active_q_min,
        q_max=active_q_max,
        cfg=settle_cfg,
    )
    if arm == "left":
        left_q_cur = active_q_cur
    else:
        right_q_cur = active_q_cur
    data.qpos[left_q_idx] = left_q_cur
    data.qpos[right_q_idx] = right_q_cur
    mujoco.mj_fwdPosition(model, data)
    z_after = _flange_orientation_alignment(model, data, active_ee_id, lock_mode)
    plane = _flange_lock_plane_label(lock_mode)
    if lock_mode == "xy" and _flange_xy_tool_down():
        print(
            f"[EE settle] arm={arm} tool DOWN -body_z.z "
            f"{align_before:.3f} → {z_after:.3f} ik_err={err:.4f} cond={cond:.1f}"
        )
    else:
        axis = _flange_lock_world_axis(lock_mode)
        print(
            f"[EE settle] arm={arm} ⊥ {plane} |dot(z,world_{axis})| "
            f"{align_before:.3f} → {z_after:.3f} ik_err={err:.4f} cond={cond:.1f}"
        )
    return active_q_cur, left_q_cur, right_q_cur


def _use_warm_start_ik() -> bool:
    if _env_truthy("LENS_IK_MULTI_SEED"):
        return False
    return _env_truthy("LENS_IK_WARM_START", True)


_IK_WARN_COUNT = 0
_IK_WARN_LIMIT = 2


def _maybe_log_ik_warning(arm: str, err: float, cond: float) -> None:
    global _IK_WARN_COUNT
    if not _env_truthy("LENS_IK_VERBOSE"):
        if _IK_WARN_COUNT < _IK_WARN_LIMIT:
            print(f"[IK] warning ({arm}): err={err:.4f}, cond={cond:.1f}")
            _IK_WARN_COUNT += 1
        elif _IK_WARN_COUNT == _IK_WARN_LIMIT:
            print("[IK] further warnings suppressed (set LENS_IK_VERBOSE=1 to show all)")
            _IK_WARN_COUNT += 1
        return
    print(f"[IK] warning ({arm}): err={err:.4f}, cond={cond:.1f}, continue in demo mode.")


def _cache_file_for_key(cache_dir: str, key: str) -> Path:
    return Path(cache_dir) / f"{key}.npz"


def _load_traj_from_disk_cache(cache_dir: str, key: str) -> JointTrajectory | None:
    p = _cache_file_for_key(cache_dir, key)
    if not p.exists():
        return None
    try:
        z = np.load(str(p), allow_pickle=True)
        hz_arr = np.asarray(z["hz"], dtype=float).reshape(-1)
        hz = float(hz_arr[0]) if hz_arr.size > 0 else 0.0
        joint_names = [str(v) for v in z["joint_names"].tolist()]
        q = np.asarray(z["q_refs"], dtype=float)
        q_refs = [np.asarray(row, dtype=float).copy() for row in q]
        if not q_refs:
            return None
        return JointTrajectory(joint_names=joint_names, q_refs=q_refs, hz=hz)
    except Exception as exc:  # noqa: BLE001
        print(f"[Cache] warning: failed to load {p}: {exc}")
        return None


_MJCF_MODEL_CACHE: dict[str, mujoco.MjModel] = {}


def _load_mj_model(model_path: str) -> mujoco.MjModel:
    if model_path not in _MJCF_MODEL_CACHE:
        _MJCF_MODEL_CACHE[model_path] = mujoco.MjModel.from_xml_path(model_path)
    return _MJCF_MODEL_CACHE[model_path]


def _lift_plan_cache_key(command: dict) -> str | None:
    if not _env_truthy("LENS_LIFT_PLAN_CACHE", True):
        return None
    arm = str(command.get("target", {}).get("arm", "left")).lower()
    if arm not in ("left", "right"):
        return None
    q_map = _try_read_joint_states_map()
    if not q_map:
        return None
    names = LEFT_JOINT_NAMES if arm == "left" else RIGHT_JOINT_NAMES
    step = float(os.environ.get("LENS_LIFT_CACHE_Q_STEP", "0.08"))
    rounded = [int(round(float(q_map.get(n, 0.0)) / step)) for n in names]
    payload = {"cmd": _stable_command_key(command), "arm": arm, "q": rounded}
    digest = hashlib.sha256(
        json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    return f"lift_{digest}"


def _save_traj_to_disk_cache(cache_dir: str, key: str, traj: JointTrajectory) -> None:
    try:
        d = Path(cache_dir)
        d.mkdir(parents=True, exist_ok=True)
        p = _cache_file_for_key(cache_dir, key)
        q = np.asarray([np.asarray(v, dtype=float) for v in traj.q_refs], dtype=float)
        np.savez_compressed(
            str(p),
            hz=np.asarray([float(traj.hz)], dtype=float),
            joint_names=np.asarray(list(traj.joint_names), dtype=object),
            q_refs=q,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"[Cache] warning: failed to save key={key}: {exc}")


def _runtime_for_lift_command(command: dict) -> CommandRuntime:
    """Minimal runtime metadata when motion uses lift_from_current (path is unused)."""
    _parse_and_validate_command(command)
    motion = command["motion"]
    ee_quat, tool_offset = _resolve_end_effector_from_command(command)
    dummy = np.zeros((1, 3), dtype=float)
    return CommandRuntime(
        command_id=str(command["command_id"]),
        arm=str(command["target"]["arm"]),
        mode=str(command["target"]["mode"]),
        sample_hz=float(motion.get("sample_hz", 25.0)),
        left_world_points=dummy,
        right_world_points=dummy,
        pen_down=np.array([True], dtype=bool),
        duration_s=float(motion["duration_s"]),
        repeat=int(motion.get("repeat", 1)),
        ee_quat=ee_quat,
        tool_offset=tool_offset,
        lock_flange_perp_xy=_flange_lock_mode(command) == "xy",
        flange_lock_mode=_flange_lock_mode(command),
    )


def build_runtime_for_command(command: dict) -> CommandRuntime:
    """Build runtime for path mode; skip 2D path sampling for lift_from_current."""
    if _motion_lift_from_current(command):
        return _runtime_for_lift_command(command)
    return build_runtime(command)


def build_runtime(command: dict) -> CommandRuntime:
    _parse_and_validate_command(command)
    _load_demo_draw_scope_from_env()
    _apply_frame_from_command(command)
    motion = command["motion"]
    points_2d, pen_down = _sample_path_2d(command["path"], motion)
    if not _env_truthy("LENS_SKIP_DRAW_RANGE_FIT"):
        points_2d = _fit_points_to_draw_range(points_2d)
    points_2d, pen_down = _resample_constant_speed(
        points_2d, pen_down, float(motion["duration_s"]), float(motion["sample_hz"])
    )
    left_points_3d = _map_2d_to_world(points_2d, LEFT_DRAW_ORIGIN)
    right_points_3d = _map_2d_to_world(points_2d, RIGHT_DRAW_ORIGIN)
#左右臂反向运动
 # 右臂与左臂反向：左 CCW，右 CW（俯视 +Z）
    if os.environ.get("LENS_RIGHT_PATH_REVERSE", "0").strip().lower() in ("1", "true", "yes", "on"):
        right_points_3d = right_points_3d[::-1].copy()
    if bool(motion.get("lift_between_subpaths", False)):
        lift = float(motion.get("lift_height_m", 0.03))
        lift_vec = _plane_normal_world() * lift
        left_points_3d[~pen_down] += lift_vec[None, :]
        right_points_3d[~pen_down] += lift_vec[None, :]
    _check_safety(left_points_3d, float(motion["sample_hz"]), command["safety"])
    _check_safety(right_points_3d, float(motion["sample_hz"]), command["safety"])
    ee_quat, tool_offset = _resolve_end_effector_from_command(command)
    return CommandRuntime(
        command_id=str(command["command_id"]),
        arm=str(command["target"]["arm"]),
        mode=str(command["target"]["mode"]),
        sample_hz=float(motion["sample_hz"]),
        left_world_points=left_points_3d,
        right_world_points=right_points_3d,
        pen_down=pen_down,
        duration_s=float(motion["duration_s"]),
        repeat=int(motion["repeat"]),
        ee_quat=ee_quat,
        tool_offset=tool_offset,
        lock_flange_perp_xy=_flange_lock_mode(command) == "xy",
        flange_lock_mode=_flange_lock_mode(command),
    )


def _get_target(runtime: CommandRuntime, t_rel: float) -> tuple[np.ndarray, np.ndarray, bool, bool]:
    if runtime.repeat == -1:
        cyc_t = t_rel % runtime.duration_s
        finished = False
    else:
        total = runtime.duration_s * max(runtime.repeat, 1)
        finished = t_rel >= total
        cyc_t = min(max(t_rel, 0.0), max(total - 1e-6, 0.0)) % runtime.duration_s
    idx = int(cyc_t * runtime.sample_hz)
    idx = int(np.clip(idx, 0, len(runtime.left_world_points) - 1))
    return runtime.left_world_points[idx], runtime.right_world_points[idx], bool(runtime.pen_down[idx]), finished


def _motion_lift_from_current(command: dict) -> bool:
    motion = command.get("motion", {})
    if not isinstance(motion, dict):
        return False
    return bool(motion.get("lift_from_current", False))


def _read_joint_states_map(timeout_s: float = 3.0) -> dict[str, float]:
    """Read latest /joint_states as name -> position (rad)."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError as exc:
        raise RuntimeError("ROS2 rclpy unavailable; cannot read /joint_states") from exc

    if not rclpy.ok():
        rclpy.init(args=None)

    class _OnceSub(Node):
        def __init__(self) -> None:
            super().__init__("lens_lift_joint_state_reader")
            self._msg: JointState | None = None
            self.create_subscription(JointState, "/joint_states", self._cb, 10)

        def _cb(self, msg: JointState) -> None:
            self._msg = msg

    node = _OnceSub()
    deadline = time.time() + max(0.2, float(timeout_s))
    while node._msg is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node._msg is None:
        node.destroy_node()
        raise RuntimeError("timeout waiting for /joint_states (is joint_controller running?)")

    out = {str(n): float(p) for n, p in zip(node._msg.name, node._msg.position)}
    node.destroy_node()
    return out


def _q_from_joint_map(
    joint_names: list[str],
    q_min: np.ndarray,
    q_max: np.ndarray,
    q_map: dict[str, float],
) -> np.ndarray:
    q = np.zeros(len(joint_names), dtype=float)
    for i, name in enumerate(joint_names):
        if name in q_map:
            q[i] = float(q_map[name])
        else:
            q[i] = 0.5 * (float(q_min[i]) + float(q_max[i]))
    return np.clip(q, q_min, q_max)


def _try_read_joint_states_map() -> dict[str, float]:
    try:
        timeout_s = float(os.environ.get("LENS_JOINT_STATE_TIMEOUT_S", "3.0"))
        return _read_joint_states_map(timeout_s=timeout_s)
    except Exception as exc:  # noqa: BLE001
        print(f"[Plan] warning: /joint_states unavailable ({exc}); using model defaults.")
        return {}


def _initial_arm_q_for_plan(
    arm: str,
    *,
    left_q_min: np.ndarray,
    left_q_max: np.ndarray,
    right_q_min: np.ndarray,
    right_q_max: np.ndarray,
    start_at_home: bool,
    q_map: dict[str, float] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Seed IK: active arm may start at home; inactive arm stays at live /joint_states."""
    q_map = q_map or {}
    home_left = np.clip(np.zeros_like(left_q_min), left_q_min, left_q_max)
    home_right = np.clip(np.zeros_like(right_q_min), right_q_min, right_q_max)
    mid_left = 0.5 * (left_q_min + left_q_max)
    mid_right = 0.5 * (right_q_min + right_q_max)

    def from_states(names: list[str], q_min: np.ndarray, q_max: np.ndarray, fallback: np.ndarray) -> np.ndarray:
        if q_map:
            return _q_from_joint_map(names, q_min, q_max, q_map)
        return fallback.copy()

    if arm == "left":
        left_q = home_left if start_at_home else from_states(LEFT_JOINT_NAMES, left_q_min, left_q_max, mid_left)
        right_q = from_states(RIGHT_JOINT_NAMES, right_q_min, right_q_max, mid_right)
        if q_map:
            print("[Plan] arm=left: right arm fixed at current /joint_states")
    elif arm == "right":
        left_q = from_states(LEFT_JOINT_NAMES, left_q_min, left_q_max, mid_left)
        right_q = home_right if start_at_home else from_states(RIGHT_JOINT_NAMES, right_q_min, right_q_max, mid_right)
        if q_map:
            print("[Plan] arm=right: left arm fixed at current /joint_states")
    elif start_at_home:
        left_q, right_q = home_left, home_right
    else:
        left_q = from_states(LEFT_JOINT_NAMES, left_q_min, left_q_max, mid_left)
        right_q = from_states(RIGHT_JOINT_NAMES, right_q_min, right_q_max, mid_right)
    return left_q, right_q


def _resolve_vertical_lift_m(command: dict) -> float:
    motion = command.get("motion", {})
    if isinstance(motion, dict) and "vertical_lift_m" in motion:
        return max(0.0, float(motion["vertical_lift_m"]))
    return max(0.0, float(os.environ.get("LENS_VERTICAL_LIFT_M", "0.24")))


def plan_lift_from_current_z(
    command: dict,
    *,
    duration_s: float,
    sample_hz: float,
) -> JointTrajectory:
    """
    Plan a straight vertical lift in base +Z from the current left-arm end-effector pose.

    Reads /joint_states for the start posture; holds the inactive arm fixed.
    """
    global _IK_WARN_COUNT
    _IK_WARN_COUNT = 0
    arm = str(command.get("target", {}).get("arm", "left")).lower()
    if arm not in ("left", "right"):
        raise ValueError("lift_from_current supports arm=left or arm=right only")

    lift_m = _resolve_vertical_lift_m(command)
    if lift_m < 1e-4:
        raise ValueError("vertical_lift_m must be > 0 for lift_from_current")

    timeout_s = float(os.environ.get("LENS_JOINT_STATE_TIMEOUT_S", "3.0"))
    q_map = _read_joint_states_map(timeout_s=timeout_s)

    n_points = int(os.environ.get("LENS_CARTESIAN_LIFT_POINTS", "12"))
    if _ultra_time_budget():
        n_points = min(n_points, int(os.environ.get("LENS_PLAN_MAX_POINTS", "3")))
    elif _fast_mode_enabled():
        n_points = max(n_points, int(os.environ.get("LENS_PLAN_MAX_POINTS", "8")))
    n_points = max(2, n_points)

    skip_ik_guard = _env_truthy("LENS_SKIP_IK_GUARD", True)
    cfg = _ik_config_for_command(command)
    model_path = os.environ.get("LENS_DUAL_ARM_MJCF", DEFAULT_MODEL_PATH)
    model = _load_mj_model(model_path)
    data = mujoco.MjData(model)

    left_q_idx = _joint_indices(model, LEFT_JOINT_NAMES)
    right_q_idx = _joint_indices(model, RIGHT_JOINT_NAMES)
    left_j_ids = _joint_ids(model, LEFT_JOINT_NAMES)
    right_j_ids = _joint_ids(model, RIGHT_JOINT_NAMES)
    left_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, LEFT_EE_BODY)
    right_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, RIGHT_EE_BODY)
    if left_ee_id < 0 or right_ee_id < 0:
        raise ValueError("IK_FAILED: end effector body id not found")

    left_q_min = model.jnt_range[left_j_ids, 0].copy()
    left_q_max = model.jnt_range[left_j_ids, 1].copy()
    right_q_min = model.jnt_range[right_j_ids, 0].copy()
    right_q_max = model.jnt_range[right_j_ids, 1].copy()
    if _enforce_joint_limits_enabled():
        l_lo, l_hi = limits_for_joint_names(LEFT_JOINT_NAMES)
        r_lo, r_hi = limits_for_joint_names(RIGHT_JOINT_NAMES)
        left_q_min = np.maximum(left_q_min, np.asarray(l_lo, dtype=float))
        left_q_max = np.minimum(left_q_max, np.asarray(l_hi, dtype=float))
        right_q_min = np.maximum(right_q_min, np.asarray(r_lo, dtype=float))
        right_q_max = np.minimum(right_q_max, np.asarray(r_hi, dtype=float))

    left_q_cur = _q_from_joint_map(LEFT_JOINT_NAMES, left_q_min, left_q_max, q_map)
    right_q_cur = _q_from_joint_map(RIGHT_JOINT_NAMES, right_q_min, right_q_max, q_map)
    data.qpos[left_q_idx] = left_q_cur
    data.qpos[right_q_idx] = right_q_cur
    mujoco.mj_fwdPosition(model, data)

    ee_quat, tool_offset = _resolve_end_effector_from_command(command)

    if arm == "left":
        start_pos = data.xpos[left_ee_id].copy()
        active_ee_id = left_ee_id
        active_q_idx = left_q_idx
        active_q_min, active_q_max = left_q_min, left_q_max
        active_q_cur = left_q_cur.copy()
        solve_ik = solve_left_ik_once
    else:
        start_pos = data.xpos[right_ee_id].copy()
        active_ee_id = right_ee_id
        active_q_idx = right_q_idx
        active_q_min, active_q_max = right_q_min, right_q_max
        active_q_cur = right_q_cur.copy()
        solve_ik = solve_right_ik_once

    lock_mode = _flange_lock_mode(command)
    if _flange_orientation_locked(command) and _env_truthy("LENS_SETTLE_ORIENTATION", True):
        print("[LiftFromCurrent] 6D IK: 7 joints cooperative, flange ⊥ XY locked")
        active_q_cur, left_q_cur, right_q_cur = _settle_active_arm_orientation(
            model=model,
            data=data,
            arm=arm,
            active_q_idx=active_q_idx,
            active_ee_id=active_ee_id,
            active_q_cur=active_q_cur,
            left_q_idx=left_q_idx,
            right_q_idx=right_q_idx,
            left_q_cur=left_q_cur,
            right_q_cur=right_q_cur,
            active_q_min=active_q_min,
            active_q_max=active_q_max,
            ee_quat=ee_quat,
            solve_ik=solve_ik,
            cfg=cfg,
            lock_mode=lock_mode,
        )
        start_pos = data.xpos[active_ee_id].copy()

    R_ee = _rotmat_world_from_quat_wxyz(ee_quat)
    start_tip = start_pos + R_ee @ tool_offset
    end_tip = start_tip + np.array([0.0, 0.0, lift_m], dtype=float)
    end_pos = _flange_pos_from_tip(end_tip, ee_quat, tool_offset)
    world_targets = np.linspace(start_pos, end_pos, n_points)

    q_refs: list[np.ndarray] = [np.concatenate([left_q_cur.copy(), right_q_cur.copy()])]

    t_plan0 = time.perf_counter()
    ik_multi = _flange_orientation_locked(command) and _ik_use_multi_when_locked()
    for i, target_pos in enumerate(world_targets[1:], start=1):
        if ik_multi and arm == "left":
            active_q_cur, err, _, cond = solve_left_ik_multi(
                model=model,
                data=data,
                q_idx=active_q_idx,
                ee_body_id=active_ee_id,
                q_prev=active_q_cur,
                target_pos=target_pos,
                target_quat=ee_quat,
                q_min=active_q_min,
                q_max=active_q_max,
                cfg=cfg,
            )
        elif ik_multi and arm == "right":
            active_q_cur, err, _, cond = solve_right_ik_multi(
                model=model,
                data=data,
                q_idx=active_q_idx,
                ee_body_id=active_ee_id,
                q_prev=active_q_cur,
                target_pos=target_pos,
                target_quat=ee_quat,
                q_min=active_q_min,
                q_max=active_q_max,
                cfg=cfg,
            )
        else:
            active_q_cur, err, _, cond = solve_ik(
                model=model,
                data=data,
                q_idx=active_q_idx,
                ee_body_id=active_ee_id,
                q_init=active_q_cur,
                target_pos=target_pos,
                target_quat=ee_quat,
                q_min=active_q_min,
                q_max=active_q_max,
                cfg=cfg,
            )
        if err > 0.02 or cond > 500.0:
            if not skip_ik_guard:
                raise ValueError(f"IK_FAILED {arm} at lift waypoint {i}/{n_points - 1}")
            _maybe_log_ik_warning(arm, err, cond)
        if arm == "left":
            left_q_cur = active_q_cur
            data.qpos[left_q_idx] = left_q_cur
        else:
            right_q_cur = active_q_cur
            data.qpos[right_q_idx] = right_q_cur
        data.qpos[left_q_idx] = left_q_cur
        data.qpos[right_q_idx] = right_q_cur
        mujoco.mj_fwdPosition(model, data)
        q_refs.append(np.concatenate([left_q_cur.copy(), right_q_cur.copy()]))

    plan_s = time.perf_counter() - t_plan0
    eff_hz = max(1.0, (len(q_refs) - 1) / max(duration_s, 1e-3))
    data.qpos[left_q_idx] = left_q_cur
    data.qpos[right_q_idx] = right_q_cur
    mujoco.mj_fwdPosition(model, data)
    final_pos = data.xpos[active_ee_id].copy()
    delta = final_pos - start_pos
    print(
        f"[LiftFromCurrent] arm={arm} lift={lift_m:.3f}m points={len(q_refs)} "
        f"start=[{start_pos[0]:+.3f},{start_pos[1]:+.3f},{start_pos[2]:+.3f}] "
        f"end=[{final_pos[0]:+.3f},{final_pos[1]:+.3f},{final_pos[2]:+.3f}] "
        f"delta=[{delta[0]:+.3f},{delta[1]:+.3f},{delta[2]:+.3f}] plan={plan_s:.2f}s"
    )
    if abs(delta[2] - lift_m) > 0.03:
        print(
            f"[LiftFromCurrent] warning: achieved dZ={delta[2]:+.3f}m "
            f"(target {lift_m:.3f}m); check IK / joint limits"
        )

    traj = JointTrajectory(
        joint_names=LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES,
        q_refs=q_refs,
        hz=eff_hz,
    )
    _validate_traj_joint_limits(traj, context="lift_from_current")
    _log_left_arm_traj_summary(traj, tag="lift")
    if _flange_orientation_locked(command):
        _log_traj_flange_alignment(
            model=model,
            data=data,
            traj=traj,
            arm=arm,
            left_q_idx=left_q_idx,
            right_q_idx=right_q_idx,
            left_ee_id=left_ee_id,
            right_ee_id=right_ee_id,
            tag="lift",
            lock_mode=lock_mode,
        )
    return traj


def _command_stub_for_ik(runtime: CommandRuntime) -> dict:
    ee: dict[str, object] = {}
    if runtime.flange_lock_mode == "yz":
        ee["lock_perpendicular_to_yz"] = True
    elif runtime.flange_lock_mode == "xz":
        ee["lock_perpendicular_to_xz"] = True
    elif runtime.flange_lock_mode == "xy":
        ee["lock_perpendicular_to_xy"] = True
    return {"end_effector": ee}


def plan_joint_trajectory(runtime: CommandRuntime) -> JointTrajectory:
    global _IK_WARN_COUNT
    _IK_WARN_COUNT = 0
    skip_ik_guard = _env_truthy("LENS_SKIP_IK_GUARD", True)
    start_at_home = _env_truthy("LENS_IK_START_AT_HOME", _fast_mode_enabled())
    warm_start = _use_warm_start_ik()
    use_multi_locked = runtime.flange_lock_mode is not None and _ik_use_multi_when_locked()
    if use_multi_locked:
        warm_start = False
    cfg = _ik_config_for_command(_command_stub_for_ik(runtime))
    model_path = os.environ.get("LENS_DUAL_ARM_MJCF", DEFAULT_MODEL_PATH)
    model = _load_mj_model(model_path)
    data = mujoco.MjData(model)

    left_q_idx = _joint_indices(model, LEFT_JOINT_NAMES)
    right_q_idx = _joint_indices(model, RIGHT_JOINT_NAMES)
    left_j_ids = _joint_ids(model, LEFT_JOINT_NAMES)
    right_j_ids = _joint_ids(model, RIGHT_JOINT_NAMES)
    left_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, LEFT_EE_BODY)
    right_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, RIGHT_EE_BODY)
    if left_ee_id < 0 or right_ee_id < 0:
        raise ValueError("IK_FAILED: end effector body id not found")

    left_q_min = model.jnt_range[left_j_ids, 0].copy()
    left_q_max = model.jnt_range[left_j_ids, 1].copy()
    right_q_min = model.jnt_range[right_j_ids, 0].copy()
    right_q_max = model.jnt_range[right_j_ids, 1].copy()
    if _enforce_joint_limits_enabled():
        l_lo, l_hi = limits_for_joint_names(LEFT_JOINT_NAMES)
        r_lo, r_hi = limits_for_joint_names(RIGHT_JOINT_NAMES)
        left_q_min = np.maximum(left_q_min, np.asarray(l_lo, dtype=float))
        left_q_max = np.minimum(left_q_max, np.asarray(l_hi, dtype=float))
        right_q_min = np.maximum(right_q_min, np.asarray(r_lo, dtype=float))
        right_q_max = np.minimum(right_q_max, np.asarray(r_hi, dtype=float))

    if start_at_home:
        print("[Plan] LENS_IK_START_AT_HOME=1: active arm IK seeds from zero posture")

    q_map: dict[str, float] = {}
    if runtime.arm in ("left", "right"):
        q_map = _try_read_joint_states_map()

    left_q_cur, right_q_cur = _initial_arm_q_for_plan(
        runtime.arm,
        left_q_min=left_q_min,
        left_q_max=left_q_max,
        right_q_min=right_q_min,
        right_q_max=right_q_max,
        start_at_home=start_at_home,
        q_map=q_map,
    )
    data.qpos[left_q_idx] = left_q_cur
    data.qpos[right_q_idx] = right_q_cur
    mujoco.mj_fwdPosition(model, data)

    if runtime.flange_lock_mode and runtime.arm in ("left", "right"):
        if _env_truthy("LENS_SETTLE_ORIENTATION", True):
            if runtime.arm == "left":
                active_q_cur, left_q_cur, right_q_cur = _settle_active_arm_orientation(
                    model=model,
                    data=data,
                    arm="left",
                    active_q_idx=left_q_idx,
                    active_ee_id=left_ee_id,
                    active_q_cur=left_q_cur.copy(),
                    left_q_idx=left_q_idx,
                    right_q_idx=right_q_idx,
                    left_q_cur=left_q_cur,
                    right_q_cur=right_q_cur,
                    active_q_min=left_q_min,
                    active_q_max=left_q_max,
                    ee_quat=runtime.ee_quat,
                    solve_ik=solve_left_ik_once,
                    cfg=cfg,
                    lock_mode=runtime.flange_lock_mode,
                )
            else:
                active_q_cur, left_q_cur, right_q_cur = _settle_active_arm_orientation(
                    model=model,
                    data=data,
                    arm="right",
                    active_q_idx=right_q_idx,
                    active_ee_id=right_ee_id,
                    active_q_cur=right_q_cur.copy(),
                    left_q_idx=left_q_idx,
                    right_q_idx=right_q_idx,
                    left_q_cur=left_q_cur,
                    right_q_cur=right_q_cur,
                    active_q_min=right_q_min,
                    active_q_max=right_q_max,
                    ee_quat=runtime.ee_quat,
                    solve_ik=solve_right_ik_once,
                    cfg=cfg,
                    lock_mode=runtime.flange_lock_mode,
                )

    q_refs: list[np.ndarray] = []
    t_rel = 0.0
    dt = 1.0 / runtime.sample_hz
    n_steps = 0
    t_plan0 = time.perf_counter()
    while True:
        left_target, right_target, _pen_down, finished = _get_target(runtime, t_rel)
        if finished:
            break
        if runtime.arm in ("left", "both"):
            left_ik_pos = _flange_pos_from_tip(left_target, runtime.ee_quat, runtime.tool_offset)
            if warm_start:
                left_q_cur, left_err, _, left_cond = solve_left_ik_once(
                    model=model,
                    data=data,
                    q_idx=left_q_idx,
                    ee_body_id=left_ee_id,
                    q_init=left_q_cur,
                    target_pos=left_ik_pos,
                    target_quat=runtime.ee_quat,
                    q_min=left_q_min,
                    q_max=left_q_max,
                    cfg=cfg,
                )
            else:
                left_q_cur, left_err, _, left_cond = solve_left_ik_multi(
                    model=model,
                    data=data,
                    q_idx=left_q_idx,
                    ee_body_id=left_ee_id,
                    q_prev=left_q_cur,
                    target_pos=left_ik_pos,
                    target_quat=runtime.ee_quat,
                    q_min=left_q_min,
                    q_max=left_q_max,
                    cfg=cfg,
                )
            if left_err > 0.02 or left_cond > 500.0:
                if not skip_ik_guard:
                    raise ValueError("IK_FAILED left")
                _maybe_log_ik_warning("left", left_err, left_cond)
        if runtime.arm in ("right", "both"):
            right_ik_pos = _flange_pos_from_tip(right_target, runtime.ee_quat, runtime.tool_offset)
            if warm_start:
                right_q_cur, right_err, _, right_cond = solve_right_ik_once(
                    model=model,
                    data=data,
                    q_idx=right_q_idx,
                    ee_body_id=right_ee_id,
                    q_init=right_q_cur,
                    target_pos=right_ik_pos,
                    target_quat=runtime.ee_quat,
                    q_min=right_q_min,
                    q_max=right_q_max,
                    cfg=cfg,
                )
            else:
                right_q_cur, right_err, _, right_cond = solve_right_ik_multi(
                    model=model,
                    data=data,
                    q_idx=right_q_idx,
                    ee_body_id=right_ee_id,
                    q_prev=right_q_cur,
                    target_pos=right_ik_pos,
                    target_quat=runtime.ee_quat,
                    q_min=right_q_min,
                    q_max=right_q_max,
                    cfg=cfg,
                )
            if right_err > 0.02 or right_cond > 500.0:
                if not skip_ik_guard:
                    raise ValueError("IK_FAILED right")
                _maybe_log_ik_warning("right", right_err, right_cond)
        data.qpos[left_q_idx] = left_q_cur
        data.qpos[right_q_idx] = right_q_cur
        q_refs.append(np.concatenate([left_q_cur, right_q_cur]))
        t_rel += dt
        n_steps += 1

    plan_s = time.perf_counter() - t_plan0
    mode = "warm-start" if warm_start else "multi-seed"
    if use_multi_locked:
        mode += "+locked-6D"
    print(
        f"[Timing] IK plan: {plan_s:.2f}s, {n_steps} points, iters={cfg.max_iters}, "
        f"rot_w={cfg.rot_weight}, mode={mode} (7 motors cooperative ⊥XY)"
    )
    traj = JointTrajectory(
        joint_names=LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES, q_refs=q_refs, hz=runtime.sample_hz
    )
    _validate_traj_joint_limits(traj, context="plan")
    if runtime.arm in ("left", "both"):
        _log_left_arm_traj_summary(traj, tag="path")
    if runtime.flange_lock_mode and runtime.arm in ("left", "right"):
        _log_traj_flange_alignment(
            model=model,
            data=data,
            traj=traj,
            arm=runtime.arm,
            left_q_idx=left_q_idx,
            right_q_idx=right_q_idx,
            left_ee_id=left_ee_id,
            right_ee_id=right_ee_id,
            tag="path",
            lock_mode=runtime.flange_lock_mode,
        )
    return traj


def plan_joint_trajectory_cached(
    command: dict,
    *,
    already_prepared: bool = False,
    timing: RunTiming | None = None,
) -> JointTrajectory:
    if timing is not None:
        with timing.span("plan.prepare"):
            prepared = command if already_prepared else _prepare_fast_plan_command(command)
    else:
        prepared = command if already_prepared else _prepare_fast_plan_command(command)

    if _motion_lift_from_current(prepared):
        motion = prepared.get("motion", {})
        duration_s = float(motion.get("duration_s", 0.5))
        sample_hz = float(motion.get("sample_hz", 25.0))
        use_cache = _env_truthy("LENS_PLAN_CACHE", True)
        cache_dir = os.environ.get(
            "LENS_PATH_IR_PLAN_CACHE_DIR",
            str(Path.home() / ".lens_path_ir_plan_cache"),
        )
        lift_key = _lift_plan_cache_key(prepared) if use_cache else None
        if lift_key:
            t_cache = time.perf_counter()
            cached = _load_traj_from_disk_cache(cache_dir, lift_key)
            cache_s = time.perf_counter() - t_cache
            if timing is not None:
                timing.record("plan.cache_lookup", cache_s)
            if cached is not None:
                print(
                    f"[LiftCache] hit key={lift_key[:16]}... points={len(cached.q_refs)} "
                    f"hz={cached.hz:.1f}"
                )
                if timing is not None:
                    timing.record("plan.ik_solve", 0.0)
                return cached
        print("[LiftFromCurrent] planning base +Z line from current /joint_states")
        if timing is not None:
            with timing.span("plan.ik_solve"):
                traj = plan_lift_from_current_z(
                    prepared, duration_s=duration_s, sample_hz=sample_hz
                )
        else:
            traj = plan_lift_from_current_z(
                prepared, duration_s=duration_s, sample_hz=sample_hz
            )
        if lift_key and use_cache:
            _save_traj_to_disk_cache(cache_dir, lift_key, traj)
            print(f"[LiftCache] saved key={lift_key[:16]}...")
        return traj

    use_cache = _env_truthy("LENS_PLAN_CACHE", True)
    key = _stable_command_key(prepared)
    cache_dir = os.environ.get(
        "LENS_PATH_IR_PLAN_CACHE_DIR",
        str(Path.home() / ".lens_path_ir_plan_cache"),
    )
    if use_cache:
        t_cache = time.perf_counter()
        cached = _load_traj_from_disk_cache(cache_dir, key)
        cache_s = time.perf_counter() - t_cache
        if timing is not None:
            timing.record("plan.cache_lookup", cache_s)
        if cached is not None:
            print(f"[Cache] hit key={key[:12]}... points={len(cached.q_refs)} hz={cached.hz:.1f}")
            try:
                _validate_traj_joint_limits(cached, context="cache")
            except JointLimitError as exc:
                print(f"[Cache] invalid/stale cache discarded, replanning: {exc}")
                try:
                    _cache_file_for_key(cache_dir, key).unlink(missing_ok=True)
                except OSError:
                    pass
            else:
                if timing is not None:
                    timing.record("plan.ik_solve", 0.0)
                return cached
    if timing is not None:
        with timing.span("plan.build_runtime"):
            runtime = build_runtime_for_command(prepared)
        with timing.span("plan.ik_solve"):
            traj = plan_joint_trajectory(runtime)
    else:
        runtime = build_runtime_for_command(prepared)
        traj = plan_joint_trajectory(runtime)
    if use_cache:
        t_save = time.perf_counter()
        _save_traj_to_disk_cache(cache_dir, key, traj)
        if timing is not None:
            timing.record("plan.cache_save", time.perf_counter() - t_save)
        print(f"[Cache] saved key={key[:12]}...")
    return traj


def _start_confirm_ui() -> tuple[threading.Event, threading.Event]:
    confirm_event = threading.Event()
    cancel_event = threading.Event()

    def run() -> None:
        try:
            root = tk.Tk()
            root.title("仿真确认")
            root.geometry("360x160")
            root.attributes("-topmost", True)
            tk.Label(root, text="仿真运行中，确认无误后下发真机").pack(pady=(16, 8))
            tk.Label(root, text="关闭窗口不会取消；请点击按钮选择。", fg="gray").pack(pady=(0, 8))

            def confirm() -> None:
                confirm_event.set()
                root.destroy()

            def cancel() -> None:
                cancel_event.set()
                root.destroy()

            tk.Button(root, text="确认下发真机", command=confirm, width=18, height=2).pack(pady=4)
            tk.Button(root, text="取消", command=cancel, width=10).pack()
            root.protocol("WM_DELETE_WINDOW", lambda: None)
            root.mainloop()
        except Exception as exc:  # noqa: BLE001
            print(f"[ConfirmUI] warning: failed to open confirm window: {exc}")

    threading.Thread(target=run, daemon=True).start()
    return confirm_event, cancel_event


def _draw_trace_userscn(
    viewer: mujoco.viewer.Handle,
    left_trace: list[np.ndarray],
    right_trace: list[np.ndarray],
    left_now: np.ndarray,
    right_now: np.ndarray,
) -> None:
    uscn = viewer.user_scn
    if uscn is None:
        return
    mat = np.eye(3, dtype=float).reshape(-1)
    with viewer.lock():
        uscn.ngeom = 0
        w = 0

        def add_sphere(p: np.ndarray, r: float, rgba: np.ndarray) -> None:
            nonlocal w
            if w >= uscn.maxgeom:
                return
            g = uscn.geoms[w]
            mujoco.mjv_initGeom(
                g,
                mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([r, r, r], dtype=float),
                np.asarray(p, dtype=float),
                mat,
                rgba,
            )
            w += 1

        for p in left_trace[::3]:
            add_sphere(p, 0.004, np.array([0.2, 1.0, 0.2, 0.45], dtype=float))
        for p in right_trace[::3]:
            add_sphere(p, 0.004, np.array([1.0, 0.2, 0.2, 0.45], dtype=float))
        add_sphere(left_now, 0.010, np.array([0.2, 1.0, 0.2, 0.95], dtype=float))
        add_sphere(right_now, 0.010, np.array([1.0, 0.2, 0.2, 0.95], dtype=float))
        uscn.ngeom = w


def preview_in_sim(traj: JointTrajectory) -> bool:
    model_path = os.environ.get("LENS_DUAL_ARM_MJCF", DEFAULT_MODEL_PATH)
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    left_act_ids = _left_actuator_ids(model)
    right_act_ids = _right_actuator_ids(model)
    left_q_idx = _joint_indices(model, LEFT_JOINT_NAMES)
    right_q_idx = _joint_indices(model, RIGHT_JOINT_NAMES)
    left_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, LEFT_EE_BODY)
    right_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, RIGHT_EE_BODY)
    frame_dt = 1.0 / traj.hz
    steps_per_frame = max(1, int(round(frame_dt / model.opt.timestep)))
    trace_len = int(float(os.environ.get("LENS_TRACE_LEN", "360")))
    left_trace: list[np.ndarray] = []
    right_trace: list[np.ndarray] = []
    confirm_event, cancel_event = _start_confirm_ui()

    print("[Preview] 启动仿真预览。点击弹窗“确认下发真机”后将自动下发。")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        can_add_marker = hasattr(viewer, "add_marker")
        while viewer.is_running():
            if confirm_event.is_set():
                return True
            if cancel_event.is_set():
                return False
            for q_ref in traj.q_refs:
                if not viewer.is_running():
                    break
                if confirm_event.is_set():
                    return True
                if cancel_event.is_set():
                    return False
                tic = time.time()
                left_q = q_ref[: len(LEFT_JOINT_NAMES)]
                right_q = q_ref[len(LEFT_JOINT_NAMES) :]
                data.ctrl[left_act_ids] = left_q
                data.ctrl[right_act_ids] = right_q
                data.qpos[left_q_idx] = left_q
                data.qpos[right_q_idx] = right_q
                for _ in range(steps_per_frame):
                    mujoco.mj_step(model, data)
                left_pos = data.xpos[left_ee_id].copy()
                right_pos = data.xpos[right_ee_id].copy()
                left_trace.append(left_pos)
                right_trace.append(right_pos)
                if len(left_trace) > trace_len:
                    left_trace = left_trace[-trace_len:]
                if len(right_trace) > trace_len:
                    right_trace = right_trace[-trace_len:]
                if can_add_marker:
                    try:
                        for p in left_trace[::3]:
                            viewer.add_marker(
                                pos=p,
                                size=np.array([0.0035, 0.0035, 0.0035], dtype=float),
                                rgba=np.array([0.2, 1.0, 0.2, 0.45], dtype=float),
                            )
                        for p in right_trace[::3]:
                            viewer.add_marker(
                                pos=p,
                                size=np.array([0.0035, 0.0035, 0.0035], dtype=float),
                                rgba=np.array([1.0, 0.2, 0.2, 0.45], dtype=float),
                            )
                        viewer.add_marker(
                            pos=left_pos,
                            size=np.array([0.010, 0.010, 0.010], dtype=float),
                            rgba=np.array([0.2, 1.0, 0.2, 0.95], dtype=float),
                        )
                        viewer.add_marker(
                            pos=right_pos,
                            size=np.array([0.010, 0.010, 0.010], dtype=float),
                            rgba=np.array([1.0, 0.2, 0.2, 0.95], dtype=float),
                        )
                    except Exception:
                        can_add_marker = False
                if not can_add_marker:
                    _draw_trace_userscn(viewer, left_trace, right_trace, left_pos, right_pos)
                viewer.sync()
                time.sleep(max(0.0, frame_dt - (time.time() - tic)))
    return False


def _resample_joint_trajectory_for_execute(
    q_refs: list[np.ndarray],
    plan_hz: float,
    duration_s: float,
    exec_hz: float,
) -> list[np.ndarray]:
    """Upsample/downsample planned waypoints so execution spans the full duration_s."""
    q_arr = np.asarray([np.asarray(v, dtype=float) for v in q_refs], dtype=float)
    if q_arr.ndim != 2 or q_arr.shape[0] == 0:
        return q_refs
    duration_s = max(1e-3, float(duration_s))
    plan_hz = max(1e-3, float(plan_hz))
    exec_hz = max(1e-3, float(exec_hz))
    n_plan = q_arr.shape[0]
    # Sparse plans (fast lift/path): keep every IK waypoint — downsampling drops the final target.
    max_keep = int(os.environ.get("LENS_EXEC_KEEP_PLAN_FRAMES", "16"))
    if n_plan <= max_keep and not os.environ.get("LENS_ROS_HZ", "").strip():
        return q_refs
    n_exec = max(n_plan, max(2, int(round(duration_s * exec_hz))))
    if n_exec == n_plan and abs(exec_hz - plan_hz) < 1e-6:
        return q_refs
    out: list[np.ndarray] = []
    for i in range(n_exec):
        t = min(float(i) / exec_hz, duration_s - 1e-9)
        u = np.clip(t * plan_hz, 0.0, float(n_plan - 1))
        i0 = int(np.floor(u))
        i1 = min(i0 + 1, n_plan - 1)
        alpha = float(u - i0)
        out.append(((1.0 - alpha) * q_arr[i0]) + (alpha * q_arr[i1]))
    return out


def _exec_max_joint_step_rad() -> float:
    """Limit per-command joint delta to reduce jerk on real hardware (0 = off)."""
    return max(0.0, float(os.environ.get("LENS_EXEC_MAX_DQ_RAD", "0.10")))


def _limit_joint_step(q_from: np.ndarray, q_to: np.ndarray, max_step: float) -> np.ndarray:
    if max_step <= 0:
        return q_to
    dq = np.asarray(q_to, dtype=float) - np.asarray(q_from, dtype=float)
    peak = float(np.max(np.abs(dq)))
    if peak <= max_step:
        return q_to
    return np.asarray(q_from, dtype=float) + dq * (max_step / peak)


def _publish_keepalive_segment(
    ros_bridge: Ros2JointCommandPublisher,
    joint_names: list[str],
    q_from: np.ndarray,
    q_to: np.ndarray,
    duration_s: float,
    keepalive_hz: float,
) -> None:
    """Publish interpolated joint targets so sparse plans still move and stay connected."""
    if duration_s <= 0:
        return
    dt = 1.0 / max(keepalive_hz, 1.0)
    q0 = np.asarray(q_from, dtype=float)
    q1 = np.asarray(q_to, dtype=float)
    max_step = _exec_max_joint_step_rad()
    q_prev = q0.copy()
    t_start = time.perf_counter()
    t_end = t_start + duration_s
    while True:
        now = time.perf_counter()
        if now >= t_end:
            break
        alpha = min(1.0, (now - t_start) / duration_s)
        q = (1.0 - alpha) * q0 + alpha * q1
        q = _limit_joint_step(q_prev, q, max_step)
        q_prev = q.copy()
        _validate_exec_joint_positions(joint_names, q, tag="keepalive")
        ros_bridge.maybe_publish(time.time(), joint_names, q)
        time.sleep(min(dt, max(0.0, t_end - now)))


def execute_real(
    traj: JointTrajectory,
    *,
    duration_s: float,
    timing: RunTiming | None = None,
) -> ExecuteResult:
    ros_hz = float(os.environ.get("LENS_ROS_HZ", str(traj.hz)))
    keepalive_hz = float(os.environ.get("LENS_EXEC_KEEPALIVE_HZ", "20"))
    bridge_hz = max(keepalive_hz, ros_hz)
    hold_s = max(0.0, float(os.environ.get("LENS_EXEC_HOLD_S", "0.6")))

    t_ros0 = time.perf_counter()
    ros_bridge = Ros2JointCommandPublisher(
        Ros2BridgeConfig(
            enabled=_env_truthy("LENS_ROS_BRIDGE"),
            topic=os.environ.get("LENS_ROS_TOPIC", "/joint_command"),
            hz=bridge_hz,
        )
    )
    ros_init_s = time.perf_counter() - t_ros0
    if timing is not None:
        timing.record("execute.ros_init", ros_init_s)
    if not ros_bridge.cfg.enabled:
        print("[Execute] LENS_ROS_BRIDGE 未开启，未实际下发。请设置 LENS_ROS_BRIDGE=1。")
        return ExecuteResult(0.0, 0.0, 0.0, ros_init_s, 0.0)
    if not getattr(ros_bridge, "_ok", False):
        print("[Execute] ROS 桥接初始化失败，未实际下发。请先 source ROS 环境并确认 rclpy 可导入。")
        return ExecuteResult(0.0, 0.0, 0.0, ros_init_s, 0.0)

    t_res0 = time.perf_counter()
    frames = _resample_joint_trajectory_for_execute(
        traj.q_refs, traj.hz, duration_s, ros_hz
    )
    resample_s = time.perf_counter() - t_res0
    if timing is not None:
        timing.record("execute.resample", resample_s)

    if _enforce_joint_limits_enabled():
        for i, q_frame in enumerate(frames):
            _validate_exec_joint_positions(
                list(traj.joint_names), np.asarray(q_frame, dtype=float), tag=f"frame[{i}]"
            )

    if os.environ.get("LENS_ROS_HZ") and abs(ros_hz - traj.hz) > 1e-6:
        print(
            f"[Execute] LENS_ROS_HZ={ros_hz:.1f}（规划 hz={traj.hz:.1f}），"
            f"已按 duration={duration_s:.2f}s 插值到 {len(frames)} 帧"
        )
    else:
        print(
            f"[Execute] 正在下发真机轨迹... plan={len(traj.q_refs)}@{traj.hz:.1f}Hz "
            f"→ exec={len(frames)}@{ros_hz:.1f}Hz duration={duration_s:.2f}s "
            f"keepalive={keepalive_hz:.0f}Hz hold={hold_s:.1f}s"
        )

    t_exec0 = time.perf_counter()
    motion_s = 0.0
    hold_actual_s = 0.0
    try:
        # Always interpolate between consecutive exec frames over duration_s.
        n_seg = max(1, len(frames) - 1)
        segment_s = duration_s / n_seg
        last_q = frames[-1]
        hold_dt = 1.0 / keepalive_hz
        t_motion0 = time.perf_counter()
        for i, q_ref in enumerate(frames):
            _validate_exec_joint_positions(
                list(traj.joint_names), np.asarray(q_ref, dtype=float), tag=f"frame[{i}]"
            )
            ros_bridge.maybe_publish(time.time(), traj.joint_names, q_ref)
            if i < len(frames) - 1:
                _publish_keepalive_segment(
                    ros_bridge,
                    traj.joint_names,
                    q_ref,
                    frames[i + 1],
                    segment_s,
                    keepalive_hz,
                )
        motion_s = time.perf_counter() - t_motion0
        if timing is not None:
            timing.record("execute.motion", motion_s)
            timing.meta["execute.motion_s"] = motion_s

        if hold_s > 0:
            t_hold0 = time.perf_counter()
            hold_until = time.perf_counter() + hold_s
            while time.perf_counter() < hold_until:
                tic = time.perf_counter()
                ros_bridge.maybe_publish(time.time(), traj.joint_names, last_q)
                time.sleep(max(0.0, hold_dt - (time.perf_counter() - tic)))
            hold_actual_s = time.perf_counter() - t_hold0
            if timing is not None:
                timing.record("execute.hold", hold_actual_s)
                timing.meta["execute.hold_s"] = hold_actual_s
    finally:
        ros_bridge.close()

    exec_s = time.perf_counter() - t_exec0
    if timing is not None:
        timing.record("execute.total", exec_s)
        timing.meta["motion_done_total_s"] = timing.elapsed_since_command()
    print(f"[Execute] 真机下发完成。motion={motion_s:.2f}s hold={hold_actual_s:.2f}s total={exec_s:.2f}s")
    return ExecuteResult(exec_s, motion_s, hold_actual_s, ros_init_s, resample_s)


def run_command_pipeline(
    command: dict,
    *,
    auto_execute: bool | None = None,
    timing: RunTiming | None = None,
) -> PipelineResult:
    """
    Full pipeline with timing: command JSON → plan → (optional) execute on real robot.
    Prints phases exceeding LENS_TIMING_THRESHOLD_S (default 1.0s).
    """
    timing = timing or RunTiming()
    timing.meta["command_id"] = str(command.get("command_id", ""))

    if _fast_mode_enabled():
        print("[FastPlan] 低延迟模式 ON（MuJoCo IK 走 CPU；BPU 仅用于 /root/tong 视觉检测，不参与 IK）")

    with timing.span("command.prepare"):
        prepared = _prepare_fast_plan_command(command)
    with timing.span("runtime.build"):
        runtime = build_runtime_for_command(prepared)

    print(
        f"[Command] id={runtime.command_id} arm={runtime.arm} mode={runtime.mode} "
        f"hz={runtime.sample_hz:.1f} duration={runtime.duration_s:.2f}s repeat={runtime.repeat}"
    )

    traj = plan_joint_trajectory_cached(prepared, already_prepared=True, timing=timing)

    do_auto = _should_auto_execute() if auto_execute is None else auto_execute
    execute_result: ExecuteResult | None = None
    if do_auto:
        print("[Execute] LENS_AUTO_EXECUTE=1，跳过仿真预览，直接下发真机。")
        execute_result = execute_real(traj, duration_s=runtime.duration_s, timing=timing)
    else:
        with timing.span("preview.sim"):
            confirmed = preview_in_sim(traj)
        if not confirmed:
            print("[Execute] 未确认下发，已取消。")
            timing.print_report()
            return PipelineResult(traj=traj, runtime=runtime, command=command, execute=None, timing=timing)
        execute_result = execute_real(traj, duration_s=runtime.duration_s, timing=timing)

    if execute_result is not None and "motion_done_total_s" not in timing.meta:
        timing.meta["motion_done_total_s"] = timing.elapsed_since_command()
    timing.print_report()
    return PipelineResult(
        traj=traj,
        runtime=runtime,
        command=command,
        execute=execute_result,
        timing=timing,
    )


def main() -> None:
    _apply_runtime_defaults()
    _load_demo_draw_scope_from_env()
    cmd_json = os.environ.get("LENS_PATH_IR_JSON", "").strip()
    cmd_file = os.environ.get("LENS_PATH_IR_FILE", "").strip()

    command: dict | None = None
    if cmd_json:
        command = json.loads(cmd_json)
        print("[Input] 使用 LENS_PATH_IR_JSON")
    elif cmd_file:
        with open(cmd_file, "r", encoding="utf-8") as f:
            command = json.load(f)
        print(f"[Input] 使用 LENS_PATH_IR_FILE={cmd_file}")
    if command is None:
        print("[Input] 未提供 Path IR，使用 fallback 协议命令。")
        command = _fallback_parse_path_ir()

    timing = RunTiming()
    result = run_command_pipeline(command, timing=timing)
    if result.execute is None:
        return

    max_total = float(os.environ.get("LENS_MAX_TOTAL_S", "10"))
    total_s = result.timing.motion_done_total_s()
    if total_s > max_total:
        print(
            f"[Timing] 超出 {max_total:.1f}s 预算。可再降低 LENS_PLAN_MAX_POINTS / "
            "LENS_IK_MAX_ITERS，或关闭 detect_bridge 释放 CPU。"
        )


if __name__ == "__main__":
    main()

