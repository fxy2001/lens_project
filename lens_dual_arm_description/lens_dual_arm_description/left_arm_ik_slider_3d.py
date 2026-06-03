"""
左臂仅位置 IK 滑条验证（3D），与 left_arm_ik_slider.py（6D）并存。
"""
from __future__ import annotations

import os
import threading
import time
import tkinter as tk
from dataclasses import dataclass

import mujoco
import mujoco.viewer
import numpy as np

from left_arm_ik_slider import (
    DEFAULT_MODEL_PATH,
    LEFT_EE_BODY,
    LEFT_JOINT_NAMES,
    _center_bias,
    _clamp_to_limits,
    _joint_ids,
    _joint_indices,
    _left_actuator_ids,
)


@dataclass
class IkConfig3D:
    max_iters: int = 60
    pos_tol: float = 1e-4
    step_clip: float = 0.10
    base_damping: float = 3e-3
    singular_damping_gain: float = 2e-2
    nullspace_gain: float = 6e-2


class TargetXYZ:
    def __init__(self, init_xyz: np.ndarray) -> None:
        self._lock = threading.Lock()
        self._xyz = init_xyz.astype(float).copy()

    def get(self) -> np.ndarray:
        with self._lock:
            return self._xyz.copy()

    def set_axis(self, idx: int, value: float) -> None:
        with self._lock:
            self._xyz[idx] = float(value)


def solve_left_ik_once_3d(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    q_idx: np.ndarray,
    ee_body_id: int,
    q_init: np.ndarray,
    target_pos: np.ndarray,
    q_min: np.ndarray,
    q_max: np.ndarray,
    cfg: IkConfig3D,
) -> tuple[np.ndarray, float, float]:
    """DLS IK，仅约束末端位置。返回 (q, 位置误差范数, Cond(J))。"""
    q = q_init.copy()
    n = q.shape[0]

    for _ in range(cfg.max_iters):
        data.qpos[q_idx] = q
        mujoco.mj_fwdPosition(model, data)

        cur_pos = data.xpos[ee_body_id].copy()
        err = target_pos - cur_pos
        err_norm = float(np.linalg.norm(err))
        if err_norm < cfg.pos_tol:
            break

        jacp = np.zeros((3, model.nv), dtype=float)
        jacr = np.zeros((3, model.nv), dtype=float)
        mujoco.mj_jacBody(model, data, jacp, jacr, ee_body_id)
        J = jacp[:, q_idx]

        sv = np.linalg.svd(J, compute_uv=False)
        smax = float(np.max(sv)) if sv.size else 0.0
        smin = float(np.min(sv)) if sv.size else 0.0
        manip = float(np.sqrt(np.linalg.det(J @ J.T) + 1e-12))
        lam = cfg.base_damping + cfg.singular_damping_gain / (manip + 1e-4)
        lam = float(np.clip(lam, cfg.base_damping, 0.25))

        A = J @ J.T + (lam * lam) * np.eye(3)
        dq_task = J.T @ np.linalg.solve(A, err)

        bias = _center_bias(q, q_min, q_max)
        J_pinv = J.T @ np.linalg.inv(A)
        N = np.eye(n) - J_pinv @ J
        dq_null = cfg.nullspace_gain * (N @ bias)

        dq = dq_task + dq_null
        dq = np.clip(dq, -cfg.step_clip, cfg.step_clip)
        q = _clamp_to_limits(q + dq, q_min, q_max)

    data.qpos[q_idx] = q
    mujoco.mj_fwdPosition(model, data)
    final_err = float(np.linalg.norm(target_pos - data.xpos[ee_body_id]))
    jacp = np.zeros((3, model.nv), dtype=float)
    jacr = np.zeros((3, model.nv), dtype=float)
    mujoco.mj_jacBody(model, data, jacp, jacr, ee_body_id)
    sv = np.linalg.svd(jacp[:, q_idx], compute_uv=False)
    smax = float(np.max(sv)) if sv.size else 0.0
    smin = float(np.min(sv)) if sv.size else 0.0
    final_cond = smax / max(smin, 1e-9)
    return q, final_err, final_cond


def solve_left_ik_multi_3d(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    q_idx: np.ndarray,
    ee_body_id: int,
    q_prev: np.ndarray,
    target_pos: np.ndarray,
    q_min: np.ndarray,
    q_max: np.ndarray,
    cfg: IkConfig3D,
) -> tuple[np.ndarray, float, float]:
    q_mid = 0.5 * (q_min + q_max)
    seeds = [
        q_prev.copy(),
        q_mid.copy(),
        q_prev.copy(),
        q_prev.copy(),
    ]
    seeds[2][3] = q_min[3] + 0.15 * (q_max[3] - q_min[3])
    seeds[3][3] = q_min[3] + 0.85 * (q_max[3] - q_min[3])

    best = None
    best_cost = float("inf")
    for s in seeds:
        q_sol, err, cond = solve_left_ik_once_3d(
            model=model,
            data=data,
            q_idx=q_idx,
            ee_body_id=ee_body_id,
            q_init=s,
            target_pos=target_pos,
            q_min=q_min,
            q_max=q_max,
            cfg=cfg,
        )
        smooth = float(np.linalg.norm(q_sol - q_prev))
        limit_margin = np.minimum(q_sol - q_min, q_max - q_sol)
        limit_penalty = float(np.sum(np.exp(-8.0 * np.maximum(limit_margin, 0.0))))
        cost = 3.0 * err + 0.15 * smooth + 0.01 * limit_penalty
        if cost < best_cost:
            best_cost = cost
            best = (q_sol, err, cond)

    assert best is not None
    return best


def start_xyz_slider_ui(shared_target: TargetXYZ, init_xyz: np.ndarray) -> None:
    def run() -> None:
        root = tk.Tk()
        root.title("Left Arm IK Target XYZ (3D)")
        root.geometry("460x220")

        labels = ["X (m)", "Y (m)", "Z (m)"]
        mins = [-0.9, -0.9, 0.0]
        maxs = [0.9, 0.9, 1.5]
        scales: list[tk.Scale] = []

        for i in range(3):
            frame = tk.Frame(root)
            frame.pack(fill="x", padx=8, pady=4)
            tk.Label(frame, text=labels[i], width=8, anchor="w").pack(side="left")
            s = tk.Scale(
                frame,
                from_=mins[i],
                to=maxs[i],
                resolution=0.001,
                orient=tk.HORIZONTAL,
                length=320,
                command=lambda v, axis=i: shared_target.set_axis(axis, float(v)),
            )
            s.set(float(init_xyz[i]))
            s.pack(side="left", fill="x", expand=True)
            scales.append(s)

        def set_home() -> None:
            home = np.array([0.0, 0.1555, 0.386], dtype=float)
            for i in range(3):
                scales[i].set(float(home[i]))
                shared_target.set_axis(i, float(home[i]))

        btn = tk.Button(root, text="Home", command=set_home)
        btn.pack(pady=8)
        root.mainloop()

    t = threading.Thread(target=run, daemon=True)
    t.start()


def main() -> None:
    model_path = os.environ.get("LENS_DUAL_ARM_MJCF", DEFAULT_MODEL_PATH)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model not found: {model_path}")

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    cfg = IkConfig3D()

    left_q_idx = _joint_indices(model, LEFT_JOINT_NAMES)
    left_j_ids = _joint_ids(model, LEFT_JOINT_NAMES)
    left_act_ids = _left_actuator_ids(model)
    left_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, LEFT_EE_BODY)
    if left_ee_id < 0:
        raise ValueError(f"Left EE body not found: {LEFT_EE_BODY}")

    q_min = model.jnt_range[left_j_ids, 0].copy()
    q_max = model.jnt_range[left_j_ids, 1].copy()

    q_cur = 0.5 * (q_min + q_max)
    data.qpos[left_q_idx] = q_cur
    mujoco.mj_fwdPosition(model, data)
    init_target = data.xpos[left_ee_id].copy()

    shared_target = TargetXYZ(init_target)
    start_xyz_slider_ui(shared_target, init_target)

    print("左臂 3D IK（仅位置）已启动。")
    print("运行 6D 版本请使用: python left_arm_ik_slider.py")
    print("请在弹出的 XYZ 滑条窗口调节末端目标位置。")

    last_print = 0.0
    target_fps = 60.0
    frame_dt = 1.0 / target_fps
    steps_per_frame = max(1, int(round(frame_dt / model.opt.timestep)))

    with mujoco.viewer.launch_passive(model, data) as viewer:
        can_add_marker = hasattr(viewer, "add_marker")
        while viewer.is_running():
            tick = time.time()
            target_pos = shared_target.get()

            q_sol, err, cond = solve_left_ik_multi_3d(
                model=model,
                data=data,
                q_idx=left_q_idx,
                ee_body_id=left_ee_id,
                q_prev=q_cur,
                target_pos=target_pos,
                q_min=q_min,
                q_max=q_max,
                cfg=cfg,
            )
            q_cur = q_sol

            data.ctrl[left_act_ids] = q_cur

            if can_add_marker:
                try:
                    viewer.add_marker(
                        pos=target_pos,
                        size=np.array([0.02, 0.02, 0.02]),
                        rgba=np.array([1.0, 0.1, 0.1, 0.9]),
                    )
                    viewer.add_marker(
                        pos=data.xpos[left_ee_id].copy(),
                        size=np.array([0.018, 0.018, 0.018]),
                        rgba=np.array([0.1, 1.0, 0.1, 0.9]),
                    )
                except Exception:
                    can_add_marker = False

            for _ in range(steps_per_frame):
                mujoco.mj_step(model, data)

            now = time.time()
            if now - last_print > 0.5:
                last_print = now
                q_deg = np.rad2deg(q_cur)
                print(
                    f"\nTarget xyz: {np.array2string(target_pos, precision=4)}"
                    f"\nEE xyz    : {np.array2string(data.xpos[left_ee_id], precision=4)}"
                    f"\nErr norm  : {err:.6f} m"
                    f"\nCond(J)   : {cond:.2f}"
                    f"\nq (deg)   : {np.array2string(q_deg, precision=2)}"
                )
                if cond > 300.0:
                    print("Warning: 接近奇异点，已自动提高阻尼并限制步长。")
                if not can_add_marker:
                    print("Note: 当前 mujoco 版本不支持 viewer.add_marker，已自动关闭点标记显示。")

            viewer.sync()
            elapsed = time.time() - tick
            time.sleep(max(0.0, frame_dt - elapsed))


if __name__ == "__main__":
    main()
