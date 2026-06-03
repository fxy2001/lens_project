from __future__ import annotations

import os
import threading
import time
import tkinter as tk
from dataclasses import dataclass
import json
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from left_arm_ik_slider import (
    DEFAULT_MODEL_PATH,
    IkConfig,
    LEFT_EE_BODY,
    LEFT_JOINT_NAMES,
    Ros2BridgeConfig,
    Ros2JointCommandPublisher,
    _center_bias,
    _clamp_to_limits,
    _joint_ids,
    _joint_indices,
    _left_actuator_ids,
    rpy_to_quat_wxyz,
    solve_left_ik_multi,
)
from right_arm_ik_slider import RIGHT_EE_BODY, RIGHT_JOINT_NAMES, _right_actuator_ids, solve_right_ik_multi


def _square_path_xy(s: float, half: float) -> tuple[float, float]:
    """
    Unit-progress square path centered at origin in XY plane.
    s in [0, 1). Returns (x, y).
    """
    s = float(s % 1.0)
    seg = int(s * 4.0)
    u = s * 4.0 - seg

    if seg == 0:  # bottom edge: left -> right
        return -half + 2.0 * half * u, -half
    if seg == 1:  # right edge: bottom -> top
        return half, -half + 2.0 * half * u
    if seg == 2:  # top edge: right -> left
        return half - 2.0 * half * u, half
    return -half, half - 2.0 * half * u  # left edge: top -> bottom


class DemoParams:
    def __init__(
        self,
        *,
        period_s: float,
        circle_radius: float,
        square_half: float,
        left_center: np.ndarray,
        right_center: np.ndarray,
        shape_rpy_deg: np.ndarray,
        trace_len: int,
        mit_kp: float,
        mit_kd: float,
        ee_rpy_deg: np.ndarray,
    ) -> None:
        self._lock = threading.Lock()
        self.period_s = float(period_s)
        self.circle_radius = float(circle_radius)
        self.square_half = float(square_half)
        self.left_center = np.asarray(left_center, dtype=float).copy()
        self.right_center = np.asarray(right_center, dtype=float).copy()
        self.shape_rpy_deg = np.asarray(shape_rpy_deg, dtype=float).copy()
        self.trace_len = int(trace_len)
        self.mit_kp = float(mit_kp)
        self.mit_kd = float(mit_kd)
        self.ee_rpy_deg = np.asarray(ee_rpy_deg, dtype=float).copy()

    def snapshot(self) -> tuple[float, float, float, np.ndarray, np.ndarray, np.ndarray, int, float, float, np.ndarray]:
        with self._lock:
            return (
                self.period_s,
                self.circle_radius,
                self.square_half,
                self.left_center.copy(),
                self.right_center.copy(),
                self.shape_rpy_deg.copy(),
                self.trace_len,
                self.mit_kp,
                self.mit_kd,
                self.ee_rpy_deg.copy(),
            )

    def set_scalar(self, key: str, val: float) -> None:
        with self._lock:
            if key == "period_s":
                self.period_s = max(1.0, float(val))
            elif key == "circle_radius":
                self.circle_radius = max(0.01, float(val))
            elif key == "square_half":
                self.square_half = max(0.01, float(val))
            elif key == "trace_len":
                self.trace_len = int(max(30, float(val)))
            elif key == "mit_kp":
                self.mit_kp = float(np.clip(val, 0.0, 300.0))
            elif key == "mit_kd":
                self.mit_kd = float(np.clip(val, 0.0, 60.0))

    def set_center_axis(self, side: str, idx: int, val: float) -> None:
        with self._lock:
            if side == "left":
                self.left_center[idx] = float(val)
            else:
                self.right_center[idx] = float(val)

    def set_rpy_axis(self, idx: int, val: float) -> None:
        with self._lock:
            self.shape_rpy_deg[idx] = float(val)

    def set_ee_rpy_axis(self, idx: int, val: float) -> None:
        with self._lock:
            self.ee_rpy_deg[idx] = float(val)

    def to_dict(self) -> dict:
        with self._lock:
            return {
                "period_s": self.period_s,
                "circle_radius": self.circle_radius,
                "square_half": self.square_half,
                "left_center": self.left_center.tolist(),
                "right_center": self.right_center.tolist(),
                "shape_rpy_deg": self.shape_rpy_deg.tolist(),
                "trace_len": int(self.trace_len),
                "mit_kp": self.mit_kp,
                "mit_kd": self.mit_kd,
                "ee_rpy_deg": self.ee_rpy_deg.tolist(),
            }

    def apply_dict(self, d: dict) -> None:
        with self._lock:
            self.period_s = float(d.get("period_s", self.period_s))
            self.circle_radius = float(d.get("circle_radius", self.circle_radius))
            self.square_half = float(d.get("square_half", self.square_half))
            if "left_center" in d and len(d["left_center"]) == 3:
                self.left_center[:] = np.asarray(d["left_center"], dtype=float)
            if "right_center" in d and len(d["right_center"]) == 3:
                self.right_center[:] = np.asarray(d["right_center"], dtype=float)
            if "shape_rpy_deg" in d and len(d["shape_rpy_deg"]) == 3:
                self.shape_rpy_deg[:] = np.asarray(d["shape_rpy_deg"], dtype=float)
            self.trace_len = int(d.get("trace_len", self.trace_len))
            self.mit_kp = float(d.get("mit_kp", self.mit_kp))
            self.mit_kd = float(d.get("mit_kd", self.mit_kd))
            if "ee_rpy_deg" in d and len(d["ee_rpy_deg"]) == 3:
                self.ee_rpy_deg[:] = np.asarray(d["ee_rpy_deg"], dtype=float)


def _start_control_ui(params: DemoParams) -> None:
    def run() -> None:
        root = tk.Tk()
        root.title("Dual Draw Controls (pose/size/speed)")
        root.geometry("620x760")

        def add_slider(parent, label, vmin, vmax, v0, resolution, cmd):
            frame = tk.Frame(parent)
            frame.pack(fill="x", padx=8, pady=3)
            tk.Label(frame, text=label, width=24, anchor="w").pack(side="left")
            s = tk.Scale(
                frame,
                from_=vmin,
                to=vmax,
                orient=tk.HORIZONTAL,
                resolution=resolution,
                length=360,
                command=lambda v: cmd(float(v)),
            )
            s.set(v0)
            s.pack(side="left", fill="x", expand=True)

        period_s, circle_r, square_half, left_center, right_center, shape_rpy_deg, trace_len, mit_kp, mit_kd, ee_rpy_deg = params.snapshot()

        tk.Label(root, text="Speed & Size", font=("TkDefaultFont", 10, "bold")).pack(anchor="w", padx=8, pady=(8, 2))
        add_slider(root, "Period (s) 速度", 1.0, 20.0, period_s, 0.1, lambda v: params.set_scalar("period_s", v))
        add_slider(root, "Circle radius (m) 圆半径", 0.01, 0.30, circle_r, 0.005, lambda v: params.set_scalar("circle_radius", v))
        add_slider(root, "Square half (m) 方形半边", 0.01, 0.30, square_half, 0.005, lambda v: params.set_scalar("square_half", v))
        add_slider(root, "Trace length 轨迹长度", 30, 1200, float(trace_len), 1.0, lambda v: params.set_scalar("trace_len", v))
        add_slider(root, "MIT Kp (for ROS/hw)", 0.0, 300.0, float(mit_kp), 0.5, lambda v: params.set_scalar("mit_kp", v))
        add_slider(root, "MIT Kd (for ROS/hw)", 0.0, 60.0, float(mit_kd), 0.1, lambda v: params.set_scalar("mit_kd", v))

        tk.Label(root, text="Left center 左臂中心", font=("TkDefaultFont", 10, "bold")).pack(anchor="w", padx=8, pady=(8, 2))
        add_slider(root, "Left X", 0.20, 0.80, float(left_center[0]), 0.005, lambda v: params.set_center_axis("left", 0, v))
        add_slider(root, "Left Y", 0.05, 0.45, float(left_center[1]), 0.005, lambda v: params.set_center_axis("left", 1, v))
        add_slider(root, "Left Z", 0.20, 0.80, float(left_center[2]), 0.005, lambda v: params.set_center_axis("left", 2, v))

        tk.Label(root, text="Right center 右臂中心", font=("TkDefaultFont", 10, "bold")).pack(anchor="w", padx=8, pady=(8, 2))
        add_slider(root, "Right X", 0.20, 0.80, float(right_center[0]), 0.005, lambda v: params.set_center_axis("right", 0, v))
        add_slider(root, "Right Y", -0.45, -0.05, float(right_center[1]), 0.005, lambda v: params.set_center_axis("right", 1, v))
        add_slider(root, "Right Z", 0.20, 0.80, float(right_center[2]), 0.005, lambda v: params.set_center_axis("right", 2, v))

        tk.Label(root, text="Shape pose 图形平面姿态 (deg)", font=("TkDefaultFont", 10, "bold")).pack(anchor="w", padx=8, pady=(8, 2))
        add_slider(root, "Shape Roll", -180, 180, float(shape_rpy_deg[0]), 1.0, lambda v: params.set_rpy_axis(0, v))
        add_slider(root, "Shape Pitch", -180, 180, float(shape_rpy_deg[1]), 1.0, lambda v: params.set_rpy_axis(1, v))
        add_slider(root, "Shape Yaw", -180, 180, float(shape_rpy_deg[2]), 1.0, lambda v: params.set_rpy_axis(2, v))

        tk.Label(root, text="EE pose 末端固定姿态 (deg)", font=("TkDefaultFont", 10, "bold")).pack(anchor="w", padx=8, pady=(8, 2))
        add_slider(root, "EE Roll", -180, 180, float(ee_rpy_deg[0]), 1.0, lambda v: params.set_ee_rpy_axis(0, v))
        add_slider(root, "EE Pitch", -180, 180, float(ee_rpy_deg[1]), 1.0, lambda v: params.set_ee_rpy_axis(1, v))
        add_slider(root, "EE Yaw", -180, 180, float(ee_rpy_deg[2]), 1.0, lambda v: params.set_ee_rpy_axis(2, v))

        tk.Label(root, text="目标轨迹: 左蓝/右橙；实际末端轨迹: 左绿/右红。", fg="gray").pack(padx=8, pady=10)
        root.mainloop()

    threading.Thread(target=run, daemon=True).start()


def _draw_trace_userscn(
    viewer: mujoco.viewer.Handle,
    left_target_trace: list[np.ndarray],
    right_target_trace: list[np.ndarray],
    left_actual_trace: list[np.ndarray],
    right_actual_trace: list[np.ndarray],
    left_target_pos: np.ndarray,
    right_target_pos: np.ndarray,
    left_actual_pos: np.ndarray,
    right_actual_pos: np.ndarray,
) -> None:
    uscn = viewer.user_scn
    if uscn is None:
        return
    mat = np.eye(3, dtype=float).reshape(-1)
    left_rgba = np.array([0.1, 0.7, 1.0, 0.55], dtype=float)
    right_rgba = np.array([1.0, 0.6, 0.1, 0.55], dtype=float)
    left_actual_rgba = np.array([0.2, 1.0, 0.2, 0.55], dtype=float)
    right_actual_rgba = np.array([1.0, 0.2, 0.2, 0.55], dtype=float)
    left_now_rgba = np.array([0.1, 0.7, 1.0, 0.95], dtype=float)      # target now
    right_now_rgba = np.array([1.0, 0.6, 0.1, 0.95], dtype=float)     # target now
    left_act_now_rgba = np.array([0.2, 1.0, 0.2, 0.95], dtype=float)  # actual now
    right_act_now_rgba = np.array([1.0, 0.2, 0.2, 0.95], dtype=float) # actual now
    r_trace = 0.006
    r_now = 0.012

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

        for p in left_target_trace[::2]:
            add_sphere(p, r_trace, left_rgba)
        for p in right_target_trace[::2]:
            add_sphere(p, r_trace, right_rgba)
        for p in left_actual_trace[::2]:
            add_sphere(p, r_trace, left_actual_rgba)
        for p in right_actual_trace[::2]:
            add_sphere(p, r_trace, right_actual_rgba)
        add_sphere(left_target_pos, r_now, left_now_rgba)
        add_sphere(right_target_pos, r_now, right_now_rgba)
        add_sphere(left_actual_pos, r_now, left_act_now_rgba)
        add_sphere(right_actual_pos, r_now, right_act_now_rgba)
        uscn.ngeom = w


def _settings_path() -> Path:
    custom = os.environ.get("LENS_DEMO_SETTINGS_FILE", "").strip()
    if custom:
        return Path(custom).expanduser().resolve()
    return (Path.home() / ".lens_dual_arm_draw_demo.json").resolve()


def _load_settings(params: DemoParams) -> None:
    p = _settings_path()
    if not p.exists():
        return
    try:
        with p.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            params.apply_dict(data)
            print(f"[Draw3D] loaded settings: {p}")
    except Exception as e:  # noqa: BLE001
        print(f"[Draw3D] warning: failed to load settings {p}: {e}")


def _save_settings(params: DemoParams) -> None:
    p = _settings_path()
    try:
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("w", encoding="utf-8") as f:
            json.dump(params.to_dict(), f, ensure_ascii=False, indent=2)
    except Exception as e:  # noqa: BLE001
        print(f"[Draw3D] warning: failed to save settings {p}: {e}")


@dataclass
class IkConfig3D:
    max_iters: int = 120
    pos_tol: float = 3e-5
    step_clip: float = 0.08
    base_damping: float = 2e-3
    singular_damping_gain: float = 1.5e-2
    nullspace_gain: float = 5e-2


def _solve_ik_once_3d(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    q_idx: np.ndarray,
    ee_body_id: int,
    q_init: np.ndarray,
    target_pos: np.ndarray,
    local_offset: np.ndarray,
    q_min: np.ndarray,
    q_max: np.ndarray,
    cfg: IkConfig3D,
) -> tuple[np.ndarray, float, float]:
    q = q_init.copy()
    n = q.shape[0]

    for _ in range(cfg.max_iters):
        data.qpos[q_idx] = q
        mujoco.mj_fwdPosition(model, data)
        tip_pos = _body_point_world(data, ee_body_id, local_offset)
        err = target_pos - tip_pos
        if float(np.linalg.norm(err)) < cfg.pos_tol:
            break

        jacp = np.zeros((3, model.nv), dtype=float)
        jacr = np.zeros((3, model.nv), dtype=float)
        mujoco.mj_jac(model, data, jacp, jacr, tip_pos, ee_body_id)
        J = jacp[:, q_idx]

        manip = float(np.sqrt(np.linalg.det(J @ J.T) + 1e-12))
        lam = cfg.base_damping + cfg.singular_damping_gain / (manip + 1e-4)
        lam = float(np.clip(lam, cfg.base_damping, 0.20))
        A = J @ J.T + (lam * lam) * np.eye(3)
        dq_task = J.T @ np.linalg.solve(A, err)

        bias = _center_bias(q, q_min, q_max)
        J_pinv = J.T @ np.linalg.inv(A)
        N = np.eye(n) - J_pinv @ J
        dq_null = cfg.nullspace_gain * (N @ bias)

        dq = np.clip(dq_task + dq_null, -cfg.step_clip, cfg.step_clip)
        q = _clamp_to_limits(q + dq, q_min, q_max)

    data.qpos[q_idx] = q
    mujoco.mj_fwdPosition(model, data)
    final_tip_pos = _body_point_world(data, ee_body_id, local_offset)
    final_err = float(np.linalg.norm(target_pos - final_tip_pos))
    jacp = np.zeros((3, model.nv), dtype=float)
    jacr = np.zeros((3, model.nv), dtype=float)
    mujoco.mj_jac(model, data, jacp, jacr, final_tip_pos, ee_body_id)
    sv = np.linalg.svd(jacp[:, q_idx], compute_uv=False)
    smax = float(np.max(sv)) if sv.size else 0.0
    smin = float(np.min(sv)) if sv.size else 0.0
    cond = smax / max(smin, 1e-9)
    return q, final_err, cond


def _solve_ik_multi_3d(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    q_idx: np.ndarray,
    ee_body_id: int,
    q_prev: np.ndarray,
    target_pos: np.ndarray,
    local_offset: np.ndarray,
    q_min: np.ndarray,
    q_max: np.ndarray,
    cfg: IkConfig3D,
) -> tuple[np.ndarray, float, float]:
    q_mid = 0.5 * (q_min + q_max)
    seeds = [q_prev.copy(), q_mid.copy(), q_prev.copy(), q_prev.copy()]
    if q_prev.shape[0] > 3:
        seeds[2][3] = q_min[3] + 0.15 * (q_max[3] - q_min[3])
        seeds[3][3] = q_min[3] + 0.85 * (q_max[3] - q_min[3])

    best = None
    best_cost = float("inf")
    for s in seeds:
        q_sol, err, cond = _solve_ik_once_3d(
            model=model,
            data=data,
            q_idx=q_idx,
            ee_body_id=ee_body_id,
            q_init=s,
            target_pos=target_pos,
            local_offset=local_offset,
            q_min=q_min,
            q_max=q_max,
            cfg=cfg,
        )
        smooth = float(np.linalg.norm(q_sol - q_prev))
        cost = 3.0 * err + 0.12 * smooth
        if cost < best_cost:
            best_cost = cost
            best = (q_sol, err, cond)
    assert best is not None
    return best


def _shape_rotmat_from_deg(shape_rpy_deg: np.ndarray) -> np.ndarray:
    q = rpy_to_quat_wxyz(np.deg2rad(shape_rpy_deg.astype(float)))
    R = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(R, q.astype(np.float64))
    return R.reshape(3, 3)


def _parse_vec3_env(name: str, default: tuple[float, float, float]) -> np.ndarray:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return np.array(default, dtype=float)
    try:
        vals = [float(x.strip()) for x in raw.split(",")]
        if len(vals) != 3:
            raise ValueError
        return np.array(vals, dtype=float)
    except Exception as exc:  # noqa: BLE001
        raise ValueError(f"Invalid {name}='{raw}', expected 'x,y,z'") from exc


def _body_point_world(data: mujoco.MjData, body_id: int, local_offset: np.ndarray) -> np.ndarray:
    R = data.xmat[body_id].reshape(3, 3)
    return data.xpos[body_id].copy() + R @ local_offset


def main() -> None:
    model_path = os.environ.get("LENS_DUAL_ARM_MJCF", DEFAULT_MODEL_PATH)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model not found: {model_path}")

    # Initial trajectory parameters (can be overridden by env vars and sliders at runtime).
    period_init = float(os.environ.get("LENS_DEMO_PERIOD", "5.0"))
    circle_radius_init = float(os.environ.get("LENS_DEMO_CIRCLE_R", "0.14"))
    square_half_init = float(os.environ.get("LENS_DEMO_SQUARE_HALF", "0.12"))
    left_center_init = np.array(
        [
            float(os.environ.get("LENS_DEMO_LEFT_X", "0.48")),
            float(os.environ.get("LENS_DEMO_LEFT_Y", "0.24")),
            float(os.environ.get("LENS_DEMO_LEFT_Z", "0.42")),
        ],
        dtype=float,
    )
    right_center_init = np.array(
        [
            float(os.environ.get("LENS_DEMO_RIGHT_X", "0.48")),
            float(os.environ.get("LENS_DEMO_RIGHT_Y", "-0.24")),
            float(os.environ.get("LENS_DEMO_RIGHT_Z", "0.42")),
        ],
        dtype=float,
    )
    # Shape plane orientation (for strict circle/square orientation in 3D space).
    front_roll_deg_init = float(os.environ.get("LENS_DEMO_SHAPE_ROLL_DEG", "0.0"))
    front_pitch_deg_init = float(os.environ.get("LENS_DEMO_SHAPE_PITCH_DEG", "0.0"))
    front_yaw_deg_init = float(os.environ.get("LENS_DEMO_SHAPE_YAW_DEG", "0.0"))
    trace_len_init = int(float(os.environ.get("LENS_DEMO_TRACE_LEN", "320")))
    ee_roll_init = float(os.environ.get("LENS_EE_ROLL_DEG", "0.0"))
    ee_pitch_init = float(os.environ.get("LENS_EE_PITCH_DEG", "-90.0"))
    ee_yaw_init = float(os.environ.get("LENS_EE_YAW_DEG", "0.0"))
    use_pose_ik = os.environ.get("LENS_USE_POSE_IK", "1").strip().lower() in ("1", "true", "yes", "on")
    # Tip offset in end-body local frame (m): use this instead of 7th-joint origin.
    # Tune according to your actuator geometry if needed.
    left_tip_local = _parse_vec3_env("LENS_LEFT_TIP_OFFSET", (0.0, 0.0, -0.08))
    right_tip_local = _parse_vec3_env("LENS_RIGHT_TIP_OFFSET", (0.0, 0.0, -0.08))

    params = DemoParams(
        period_s=period_init,
        circle_radius=circle_radius_init,
        square_half=square_half_init,
        left_center=left_center_init,
        right_center=right_center_init,
        shape_rpy_deg=np.array([front_roll_deg_init, front_pitch_deg_init, front_yaw_deg_init], dtype=float),
        trace_len=trace_len_init,
        mit_kp=float(os.environ.get("LENS_MIT_KP", "20.0")),
        mit_kd=float(os.environ.get("LENS_MIT_KD", "8.0")),
        ee_rpy_deg=np.array([ee_roll_init, ee_pitch_init, ee_yaw_init], dtype=float),
    )
    _load_settings(params)
    _start_control_ui(params)

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    cfg = IkConfig3D()
    pose_cfg = IkConfig()

    left_q_idx = _joint_indices(model, LEFT_JOINT_NAMES)
    left_j_ids = _joint_ids(model, LEFT_JOINT_NAMES)
    left_act_ids = _left_actuator_ids(model)
    left_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, LEFT_EE_BODY)
    if left_ee_id < 0:
        raise ValueError(f"Left EE body not found: {LEFT_EE_BODY}")

    right_q_idx = _joint_indices(model, RIGHT_JOINT_NAMES)
    right_j_ids = _joint_ids(model, RIGHT_JOINT_NAMES)
    right_act_ids = _right_actuator_ids(model)
    right_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, RIGHT_EE_BODY)
    if right_ee_id < 0:
        raise ValueError(f"Right EE body not found: {RIGHT_EE_BODY}")

    left_q_min = model.jnt_range[left_j_ids, 0].copy()
    left_q_max = model.jnt_range[left_j_ids, 1].copy()
    right_q_min = model.jnt_range[right_j_ids, 0].copy()
    right_q_max = model.jnt_range[right_j_ids, 1].copy()

    left_q_cur = 0.5 * (left_q_min + left_q_max)
    right_q_cur = 0.5 * (right_q_min + right_q_max)
    data.qpos[left_q_idx] = left_q_cur
    data.qpos[right_q_idx] = right_q_cur
    mujoco.mj_fwdPosition(model, data)

    ros_bridge = Ros2JointCommandPublisher(
        Ros2BridgeConfig(
            enabled=os.environ.get("LENS_ROS_BRIDGE", "0").strip().lower() in ("1", "true", "yes", "on"),
            topic=os.environ.get("LENS_ROS_TOPIC", "/joint_command"),
            hz=float(os.environ.get("LENS_ROS_HZ", "120")),
        )
    )
    all_joint_names = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES

    print("Dual-arm demo started: left draws CIRCLE, right draws SQUARE.")
    print(f"EE fixed-pose IK mode={'ON' if use_pose_ik else 'OFF(3D only)'}; EE RPY is slider-adjustable.")
    print(f"Tip offset local (left/right): {left_tip_local.tolist()} / {right_tip_local.tolist()}")

    target_fps = float(os.environ.get("LENS_SIM_FPS", "120"))
    frame_dt = 1.0 / target_fps
    steps_per_frame = max(1, int(round(frame_dt / model.opt.timestep)))
    t0 = time.time()
    last_print = 0.0
    last_save = 0.0
    can_add_marker = False
    left_target_trace: list[np.ndarray] = []
    right_target_trace: list[np.ndarray] = []
    left_actual_trace: list[np.ndarray] = []
    right_actual_trace: list[np.ndarray] = []

    with mujoco.viewer.launch_passive(model, data) as viewer:
        can_add_marker = hasattr(viewer, "add_marker")
        if not can_add_marker:
            print("[DrawDemo] viewer.add_marker unavailable, using user_scn trace rendering fallback.")
        try:
            while viewer.is_running():
                tic = time.time()
                period_s, circle_radius, square_half, left_center, right_center, shape_rpy_deg, trace_len, mit_kp, mit_kd, ee_rpy_deg = params.snapshot()
                elapsed = tic - t0
                phase = (elapsed / max(period_s, 1e-3)) % 1.0
                Rshape = _shape_rotmat_from_deg(shape_rpy_deg)
                ee_quat = rpy_to_quat_wxyz(np.deg2rad(ee_rpy_deg.astype(float)))

                # Left arm: circle in XY plane at fixed Z.
                th = 2.0 * np.pi * phase
                left_offset_local = np.array([circle_radius * np.cos(th), circle_radius * np.sin(th), 0.0], dtype=float)
                left_target_pos = left_center + Rshape @ left_offset_local

                # Right arm: square in XY plane at fixed Z.
                sx, sy = _square_path_xy(phase, square_half)
                right_offset_local = np.array([sx, sy, 0.0], dtype=float)
                right_target_pos = right_center + Rshape @ right_offset_local

                left_target_trace.append(left_target_pos.copy())
                right_target_trace.append(right_target_pos.copy())
                if len(left_target_trace) > trace_len:
                    left_target_trace = left_target_trace[-trace_len:]
                if len(right_target_trace) > trace_len:
                    right_target_trace = right_target_trace[-trace_len:]

                if use_pose_ik:
                    left_q_sol, left_err_p, _left_err_r, left_cond = solve_left_ik_multi(
                        model=model,
                        data=data,
                        q_idx=left_q_idx,
                        ee_body_id=left_ee_id,
                        q_prev=left_q_cur,
                        target_pos=left_target_pos,
                        target_quat=ee_quat,
                        q_min=left_q_min,
                        q_max=left_q_max,
                        cfg=pose_cfg,
                    )
                    right_q_sol, right_err_p, _right_err_r, right_cond = solve_right_ik_multi(
                        model=model,
                        data=data,
                        q_idx=right_q_idx,
                        ee_body_id=right_ee_id,
                        q_prev=right_q_cur,
                        target_pos=right_target_pos,
                        target_quat=ee_quat,
                        q_min=right_q_min,
                        q_max=right_q_max,
                        cfg=pose_cfg,
                    )
                else:
                    left_q_sol, left_err_p, left_cond = _solve_ik_multi_3d(
                        model=model,
                        data=data,
                        q_idx=left_q_idx,
                        ee_body_id=left_ee_id,
                        q_prev=left_q_cur,
                        target_pos=left_target_pos,
                        q_min=left_q_min,
                        q_max=left_q_max,
                        cfg=cfg,
                        local_offset=left_tip_local,
                    )
                    right_q_sol, right_err_p, right_cond = _solve_ik_multi_3d(
                        model=model,
                        data=data,
                        q_idx=right_q_idx,
                        ee_body_id=right_ee_id,
                        q_prev=right_q_cur,
                        target_pos=right_target_pos,
                        q_min=right_q_min,
                        q_max=right_q_max,
                        cfg=cfg,
                        local_offset=right_tip_local,
                    )

                left_q_cur = left_q_sol
                right_q_cur = right_q_sol

                data.ctrl[left_act_ids] = left_q_cur
                data.ctrl[right_act_ids] = right_q_cur
                ros_bridge.maybe_publish(tic, all_joint_names, np.concatenate([left_q_cur, right_q_cur]))

                for _ in range(steps_per_frame):
                    mujoco.mj_step(model, data)

                left_actual_pos = _body_point_world(data, left_ee_id, left_tip_local)
                right_actual_pos = _body_point_world(data, right_ee_id, right_tip_local)
                left_actual_trace.append(left_actual_pos.copy())
                right_actual_trace.append(right_actual_pos.copy())
                if len(left_actual_trace) > trace_len:
                    left_actual_trace = left_actual_trace[-trace_len:]
                if len(right_actual_trace) > trace_len:
                    right_actual_trace = right_actual_trace[-trace_len:]

                if can_add_marker:
                    try:
                        # Target now
                        viewer.add_marker(
                            pos=left_target_pos,
                            size=np.array([0.010, 0.010, 0.010], dtype=float),
                            rgba=np.array([0.1, 0.7, 1.0, 0.95], dtype=float),
                        )
                        viewer.add_marker(
                            pos=right_target_pos,
                            size=np.array([0.010, 0.010, 0.010], dtype=float),
                            rgba=np.array([1.0, 0.6, 0.1, 0.95], dtype=float),
                        )
                        # Actual now
                        viewer.add_marker(
                            pos=left_actual_pos,
                            size=np.array([0.010, 0.010, 0.010], dtype=float),
                            rgba=np.array([0.2, 1.0, 0.2, 0.95], dtype=float),
                        )
                        viewer.add_marker(
                            pos=right_actual_pos,
                            size=np.array([0.010, 0.010, 0.010], dtype=float),
                            rgba=np.array([1.0, 0.2, 0.2, 0.95], dtype=float),
                        )
                        # Target traces
                        for p in left_target_trace[::3]:
                            viewer.add_marker(
                                pos=p,
                                size=np.array([0.004, 0.004, 0.004], dtype=float),
                                rgba=np.array([0.1, 0.7, 1.0, 0.35], dtype=float),
                            )
                        for p in right_target_trace[::3]:
                            viewer.add_marker(
                                pos=p,
                                size=np.array([0.004, 0.004, 0.004], dtype=float),
                                rgba=np.array([1.0, 0.6, 0.1, 0.35], dtype=float),
                            )
                        # Actual traces
                        for p in left_actual_trace[::3]:
                            viewer.add_marker(
                                pos=p,
                                size=np.array([0.004, 0.004, 0.004], dtype=float),
                                rgba=np.array([0.2, 1.0, 0.2, 0.35], dtype=float),
                            )
                        for p in right_actual_trace[::3]:
                            viewer.add_marker(
                                pos=p,
                                size=np.array([0.004, 0.004, 0.004], dtype=float),
                                rgba=np.array([1.0, 0.2, 0.2, 0.35], dtype=float),
                            )
                    except Exception:
                        can_add_marker = False
                if not can_add_marker:
                    _draw_trace_userscn(
                        viewer,
                        left_target_trace,
                        right_target_trace,
                        left_actual_trace,
                        right_actual_trace,
                        left_target_pos,
                        right_target_pos,
                        left_actual_pos,
                        right_actual_pos,
                    )

                now = time.time()
                if now - last_print > 0.5:
                    last_print = now
                    print(
                        f"[Draw3D] L(err={left_err_p:.4f}m cond={left_cond:.1f}) "
                        f"R(err={right_err_p:.4f}m cond={right_cond:.1f}) "
                        f"shape_rpy_deg={np.array2string(shape_rpy_deg, precision=1)} "
                        f"ee_rpy_deg={np.array2string(ee_rpy_deg, precision=1)} "
                        f"PD(kp={mit_kp:.1f},kd={mit_kd:.1f})"
                    )
                if now - last_save > 1.0:
                    last_save = now
                    _save_settings(params)

                viewer.sync()
                time.sleep(max(0.0, frame_dt - (time.time() - tic)))
        finally:
            _save_settings(params)
            ros_bridge.close()


if __name__ == "__main__":
    main()

