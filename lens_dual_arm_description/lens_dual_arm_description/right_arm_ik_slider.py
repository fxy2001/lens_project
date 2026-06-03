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
    Ros2BridgeConfig,
    Ros2JointCommandPublisher,
    TargetSE3,
    IkConfig,
    _center_bias,
    _clamp_to_limits,
    _joint_ids,
    _joint_indices,
    _orientation_error_axis_world,
    _geodesic_angle_rad,
    quat_wxyz_to_rpy_mujoco,
    rpy_to_quat_wxyz,
)


RIGHT_JOINT_NAMES = [
    "Right_Shoulder_Pitch_Joint",
    "Right_Shoulder_Roll_Joint",
    "Right_Shoulder_Yaw_Joint",
    "Right_Elbow_Pitch_Joint",
    "Right_Wrist_Yaw_Joint",
    "Right_Wrist_Roll_Joint",
    "Right_Wrist_Pitch_Joint",
]
RIGHT_EE_BODY = "Right_Wrist_Pitch_Link"


def _right_actuator_ids(model: mujoco.MjModel) -> np.ndarray:
    ids = []
    for name in RIGHT_JOINT_NAMES:
        a_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if a_id < 0:
            raise ValueError(f"Actuator not found (expected position actuator): {name}")
        ids.append(a_id)
    return np.array(ids, dtype=int)


def solve_right_ik_once(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    q_idx: np.ndarray,
    ee_body_id: int,
    q_init: np.ndarray,
    target_pos: np.ndarray,
    target_quat: np.ndarray,
    q_min: np.ndarray,
    q_max: np.ndarray,
    cfg: IkConfig,
) -> tuple[np.ndarray, float, float, float]:
    q = q_init.copy()
    n = q.shape[0]
    tq = target_quat.astype(np.float64).copy()
    mujoco.mju_normalize4(tq)

    for _ in range(cfg.max_iters):
        data.qpos[q_idx] = q
        mujoco.mj_fwdPosition(model, data)

        cur_pos = data.xpos[ee_body_id].copy()
        err_pos = target_pos - cur_pos
        cq = data.xquat[ee_body_id].astype(np.float64).copy()
        mujoco.mju_normalize4(cq)
        err_rot = _orientation_error_axis_world(tq, cq)

        if float(np.linalg.norm(err_pos)) < cfg.pos_tol and _geodesic_angle_rad(tq, cq) < cfg.rot_tol:
            break

        jacp = np.zeros((3, model.nv), dtype=float)
        jacr = np.zeros((3, model.nv), dtype=float)
        mujoco.mj_jacBody(model, data, jacp, jacr, ee_body_id)
        J = np.vstack([jacp[:, q_idx], jacr[:, q_idx]])  # 6x7
        err = np.concatenate(
            [cfg.pos_weight * err_pos, cfg.rot_weight * err_rot], dtype=float
        )

        manip = float(np.sqrt(np.linalg.det(J @ J.T + 1e-12 * np.eye(6))))
        lam = cfg.base_damping + cfg.singular_damping_gain / (manip + 1e-4)
        lam = float(np.clip(lam, cfg.base_damping, 0.35))

        A = J @ J.T + (lam * lam) * np.eye(6)
        dq_task = J.T @ np.linalg.solve(A, err)

        bias = _center_bias(q, q_min, q_max)
        J_pinv = J.T @ np.linalg.inv(A)
        N = np.eye(n) - J_pinv @ J
        dq_null = cfg.nullspace_gain * (N @ bias)
        if cfg.wrist_nullspace_scale < 0.999:
            dq_null[4:7] *= float(cfg.wrist_nullspace_scale)

        dq = dq_task + dq_null
        dq = np.clip(dq, -cfg.step_clip, cfg.step_clip)
        q = _clamp_to_limits(q + dq, q_min, q_max)

    refine = int(cfg.orientation_refine_iters)
    if refine > 0:
        hold_w = float(cfg.refine_pos_hold_weight)
        for _ in range(refine):
            data.qpos[q_idx] = q
            mujoco.mj_fwdPosition(model, data)
            cur_pos = data.xpos[ee_body_id].copy()
            err_pos = target_pos - cur_pos
            cq = data.xquat[ee_body_id].astype(np.float64).copy()
            mujoco.mju_normalize4(cq)
            if _geodesic_angle_rad(tq, cq) < cfg.rot_tol:
                break
            err_rot = _orientation_error_axis_world(tq, cq)
            jacp = np.zeros((3, model.nv), dtype=float)
            jacr = np.zeros((3, model.nv), dtype=float)
            mujoco.mj_jacBody(model, data, jacp, jacr, ee_body_id)
            J = np.vstack([jacp[:, q_idx], jacr[:, q_idx]])
            err = np.concatenate([hold_w * err_pos, cfg.rot_weight * err_rot], dtype=float)
            manip = float(np.sqrt(np.linalg.det(J @ J.T + 1e-12 * np.eye(6))))
            lam = cfg.base_damping + cfg.singular_damping_gain / (manip + 1e-4)
            A = J @ J.T + (lam * lam) * np.eye(6)
            dq_task = J.T @ np.linalg.solve(A, err)
            bias = _center_bias(q, q_min, q_max)
            J_pinv = J.T @ np.linalg.inv(A)
            N = np.eye(n) - J_pinv @ J
            dq_null = cfg.nullspace_gain * (N @ bias)
            if cfg.wrist_nullspace_scale < 0.999:
                dq_null[4:7] *= float(cfg.wrist_nullspace_scale)
            dq = np.clip(dq_task + dq_null, -cfg.step_clip, cfg.step_clip)
            q = _clamp_to_limits(q + dq, q_min, q_max)

    data.qpos[q_idx] = q
    mujoco.mj_fwdPosition(model, data)
    final_pos = float(np.linalg.norm(target_pos - data.xpos[ee_body_id]))
    cq = data.xquat[ee_body_id].astype(np.float64).copy()
    mujoco.mju_normalize4(cq)
    final_rot = _geodesic_angle_rad(tq, cq)

    jacp = np.zeros((3, model.nv), dtype=float)
    jacr = np.zeros((3, model.nv), dtype=float)
    mujoco.mj_jacBody(model, data, jacp, jacr, ee_body_id)
    J6 = np.vstack([jacp[:, q_idx], jacr[:, q_idx]])
    sv = np.linalg.svd(J6, compute_uv=False)
    smax = float(np.max(sv)) if sv.size else 0.0
    smin = float(np.min(sv)) if sv.size else 0.0
    final_cond = smax / max(smin, 1e-9)
    return q, final_pos, final_rot, final_cond


def solve_right_ik_multi(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    q_idx: np.ndarray,
    ee_body_id: int,
    q_prev: np.ndarray,
    target_pos: np.ndarray,
    target_quat: np.ndarray,
    q_min: np.ndarray,
    q_max: np.ndarray,
    cfg: IkConfig,
) -> tuple[np.ndarray, float, float, float]:
    q_mid = 0.5 * (q_min + q_max)
    seeds = [
        q_prev.copy(),
        q_mid.copy(),
        q_prev.copy(),
        q_prev.copy(),
    ]
    # elbow-biased candidates
    seeds[2][3] = q_min[3] + 0.15 * (q_max[3] - q_min[3])
    seeds[3][3] = q_min[3] + 0.85 * (q_max[3] - q_min[3])

    best = None
    best_cost = float("inf")
    for s in seeds:
        q_sol, err_p, err_r, cond = solve_right_ik_once(
            model=model,
            data=data,
            q_idx=q_idx,
            ee_body_id=ee_body_id,
            q_init=s,
            target_pos=target_pos,
            target_quat=target_quat,
            q_min=q_min,
            q_max=q_max,
            cfg=cfg,
        )
        smooth = float(np.linalg.norm(q_sol - q_prev))
        limit_margin = np.minimum(q_sol - q_min, q_max - q_sol)
        limit_penalty = float(np.sum(np.exp(-8.0 * np.maximum(limit_margin, 0.0))))
        cost = 3.0 * err_p + 1.2 * err_r + 0.15 * smooth + 0.01 * limit_penalty
        if cost < best_cost:
            best_cost = cost
            best = (q_sol, err_p, err_r, cond)

    assert best is not None
    return best


def start_se3_slider_ui(shared_target: TargetSE3, init_xyz: np.ndarray, init_rpy: np.ndarray) -> None:
    def run() -> None:
        root = tk.Tk()
        root.title("Right Arm IK Target 6D (XYZ + RPY)")
        root.geometry("500x420")

        labels = ["X (m)", "Y (m)", "Z (m)", "Roll (deg)", "Pitch (deg)", "Yaw (deg)"]
        mins = [-0.9, -0.9, 0.0, -180.0, -180.0, -180.0]
        maxs = [0.9, 0.9, 1.5, 180.0, 180.0, 180.0]
        resolutions = [0.001, 0.001, 0.001, 0.5, 0.5, 0.5]
        scales: list[tk.Scale] = []

        for i in range(3):
            frame = tk.Frame(root)
            frame.pack(fill="x", padx=8, pady=3)
            tk.Label(frame, text=labels[i], width=12, anchor="w").pack(side="left")
            s = tk.Scale(
                frame,
                from_=mins[i],
                to=maxs[i],
                resolution=resolutions[i],
                orient=tk.HORIZONTAL,
                length=340,
                command=lambda v, axis=i: shared_target.set_xyz_axis(axis, float(v)),
            )
            s.set(float(init_xyz[i]))
            s.pack(side="left", fill="x", expand=True)
            scales.append(s)

        init_deg = np.rad2deg(init_rpy)
        for j in range(3):
            i = 3 + j
            frame = tk.Frame(root)
            frame.pack(fill="x", padx=8, pady=3)
            tk.Label(frame, text=labels[i], width=12, anchor="w").pack(side="left")
            s = tk.Scale(
                frame,
                from_=mins[i],
                to=maxs[i],
                resolution=resolutions[i],
                orient=tk.HORIZONTAL,
                length=340,
                command=lambda v, axis=j: shared_target.set_rpy_axis(axis, float(np.deg2rad(float(v)))),
            )
            s.set(float(init_deg[j]))
            s.pack(side="left", fill="x", expand=True)
            scales.append(s)

        # symmetric home (y negative for right)
        home_xyz = np.array([0.0, -0.1555, 0.386], dtype=float)
        home_rpy_deg = np.rad2deg(init_rpy)

        def set_home() -> None:
            for i in range(3):
                scales[i].set(float(home_xyz[i]))
                shared_target.set_xyz_axis(i, float(home_xyz[i]))
            for j in range(3):
                scales[3 + j].set(float(home_rpy_deg[j]))
                shared_target.set_rpy_axis(j, float(init_rpy[j]))

        tk.Button(root, text="Home (preset XYZ + start RPY)", command=set_home).pack(pady=8)
        tk.Label(
            root,
            text="RPY: MuJoCo euler 'xyz' → R = Rx(roll)·Ry(pitch)·Rz(yaw), world frame.",
            font=("TkDefaultFont", 8),
            fg="gray",
        ).pack(pady=(0, 6))
        root.mainloop()

    t = threading.Thread(target=run, daemon=True)
    t.start()


def main() -> None:
    model_path = os.environ.get("LENS_DUAL_ARM_MJCF", DEFAULT_MODEL_PATH)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model not found: {model_path}")

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    cfg = IkConfig()

    right_q_idx = _joint_indices(model, RIGHT_JOINT_NAMES)
    right_j_ids = _joint_ids(model, RIGHT_JOINT_NAMES)
    right_act_ids = _right_actuator_ids(model)
    right_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, RIGHT_EE_BODY)
    if right_ee_id < 0:
        raise ValueError(f"Right EE body not found: {RIGHT_EE_BODY}")

    q_min = model.jnt_range[right_j_ids, 0].copy()
    q_max = model.jnt_range[right_j_ids, 1].copy()

    q_cur = 0.5 * (q_min + q_max)
    data.qpos[right_q_idx] = q_cur
    mujoco.mj_fwdPosition(model, data)
    init_target = data.xpos[right_ee_id].copy()
    init_quat = data.xquat[right_ee_id].copy()
    init_rpy = quat_wxyz_to_rpy_mujoco(init_quat)

    shared_target = TargetSE3(init_target, init_rpy)
    start_se3_slider_ui(shared_target, init_target, init_rpy)
    ros_bridge = Ros2JointCommandPublisher(
        Ros2BridgeConfig(
            enabled=os.environ.get("LENS_ROS_BRIDGE", "0").strip().lower() in ("1", "true", "yes", "on"),
            topic=os.environ.get("LENS_ROS_TOPIC", "/joint_command"),
            hz=float(os.environ.get("LENS_ROS_HZ", "60")),
        )
    )

    print("右臂 6D IK 实时求解已启动。")
    print("请在弹出的窗口用 XYZ + Roll/Pitch/Yaw 调节末端目标位姿。")

    last_print = 0.0
    target_fps = 60.0
    frame_dt = 1.0 / target_fps
    steps_per_frame = max(1, int(round(frame_dt / model.opt.timestep)))

    with mujoco.viewer.launch_passive(model, data) as viewer:
        can_add_marker = hasattr(viewer, "add_marker")
        try:
            while viewer.is_running():
                tick = time.time()
                target_pos, target_quat = shared_target.get_pose()

                q_sol, err_p, err_r, cond = solve_right_ik_multi(
                    model=model,
                    data=data,
                    q_idx=right_q_idx,
                    ee_body_id=right_ee_id,
                    q_prev=q_cur,
                    target_pos=target_pos,
                    target_quat=target_quat,
                    q_min=q_min,
                    q_max=q_max,
                    cfg=cfg,
                )
                q_cur = q_sol
                data.ctrl[right_act_ids] = q_cur
                ros_bridge.maybe_publish(tick, RIGHT_JOINT_NAMES, q_cur)

                if can_add_marker:
                    try:
                        viewer.add_marker(
                            pos=target_pos,
                            size=np.array([0.02, 0.02, 0.02]),
                            rgba=np.array([1.0, 0.1, 0.1, 0.9]),
                        )
                        viewer.add_marker(
                            pos=data.xpos[right_ee_id].copy(),
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
                    if cond > 300.0:
                        print("Warning: 接近奇异点，已自动提高阻尼并限制步长。")

                viewer.sync()
                elapsed = time.time() - tick
                time.sleep(max(0.0, frame_dt - elapsed))
        finally:
            ros_bridge.close()


if __name__ == "__main__":
    main()

