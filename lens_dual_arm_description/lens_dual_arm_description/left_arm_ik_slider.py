from __future__ import annotations

import os
import threading
import time
import tkinter as tk
from dataclasses import dataclass

import mujoco
import mujoco.viewer
import numpy as np


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL_PATH = os.path.join(THIS_DIR, "mjcf", "mj_lens_dual_arm.xml")


LEFT_JOINT_NAMES = [
    "Left_Shoulder_Pitch_Joint",
    "Left_Shoulder_Roll_Joint",
    "Left_Shoulder_Yaw_Joint",
    "Left_Elbow_Pitch_Joint",
    "Left_Wrist_Yaw_Joint",
    "Left_Wrist_Roll_Joint",
    "Left_Wrist_Pitch_Joint",
]
LEFT_EE_BODY = "Left_Wrist_Pitch_Link"


@dataclass
class Ros2BridgeConfig:
    enabled: bool = False
    topic: str = "/joint_command"
    hz: float = 60.0


class Ros2JointCommandPublisher:
    """
    Optional ROS2 bridge:
    publish left-arm joint commands as JointState for real hardware controller.
    """

    def __init__(self, cfg: Ros2BridgeConfig) -> None:
        self.cfg = cfg
        self._ok = False
        self._last_t = 0.0
        self._rclpy = None
        self._node = None
        self._pub = None
        self._JointState = None
        if not cfg.enabled:
            return
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
        except Exception as e:  # noqa: BLE001
            print(f"[ROS Bridge] disabled: import failed ({e})")
            return
        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node("left_arm_ik6d_ros_bridge")
        self._pub = self._node.create_publisher(JointState, cfg.topic, 10)
        self._JointState = JointState
        self._ok = True
        print(f"[ROS Bridge] enabled, topic={cfg.topic}, hz={cfg.hz}")

    def maybe_publish(self, now_s: float, joint_names: list[str], q_cmd: np.ndarray) -> None:
        if not self._ok:
            return
        if self.cfg.hz > 0 and (now_s - self._last_t) < 1.0 / self.cfg.hz:
            return
        self._last_t = now_s
        msg = self._JointState()
        msg.name = list(joint_names)
        msg.position = [float(v) for v in np.asarray(q_cmd, dtype=float).reshape(-1)]
        self._pub.publish(msg)
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        if not self._ok:
            return
        try:
            self._node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


@dataclass
class IkConfig:
    max_iters: int = 80
    pos_tol: float = 1e-4
    rot_tol: float = 2e-3  # rad (~0.1 deg)
    step_clip: float = 0.10  # rad/iter
    base_damping: float = 3e-3
    singular_damping_gain: float = 2e-2
    nullspace_gain: float = 6e-2
    pos_weight: float = 1.0
    rot_weight: float = 1.0
    """Scale orientation rows in DLS (higher → enforce flange ⊥ plane along path)."""
    wrist_nullspace_scale: float = 1.0
    """0 = wrist joints not pulled to mid-range; all 7 joints still solve 6D pose together."""
    orientation_refine_iters: int = 0
    """Extra full 6D IK iterations (position hold + orientation) after main loop."""
    refine_pos_hold_weight: float = 5.0
    """During refine: scale position error to keep tip near target while fixing orientation."""


def rpy_to_quat_wxyz(rpy: np.ndarray) -> np.ndarray:
    """MuJoCo intrinsic 'xyz' euler = R = Rx(r)*Ry(p)*Rz(y)."""
    q = np.zeros(4, dtype=np.float64)
    mujoco.mju_euler2Quat(q, rpy.astype(np.float64), "xyz")
    return q


def quat_wxyz_to_rpy_mujoco(q: np.ndarray) -> np.ndarray:
    """Inverse of rpy_to_quat_wxyz for the same MuJoCo 'xyz' convention."""
    R = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(R, q.astype(np.float64))
    R = R.reshape(3, 3)
    sy = R[0, 2]
    pitch = float(np.arcsin(np.clip(sy, -1.0, 1.0)))
    cy = np.cos(pitch)
    if abs(cy) > 1e-8:
        roll = float(np.arctan2(-R[1, 2], R[2, 2]))
        yaw = float(np.arctan2(-R[0, 1], R[0, 0]))
    else:
        roll = float(np.arctan2(R[1, 0], R[1, 1]))
        yaw = 0.0
    return np.array([roll, pitch, yaw], dtype=np.float64)


def _rotmat_world_from_quat_wxyz(q: np.ndarray) -> np.ndarray:
    R = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(R, q.astype(np.float64))
    return R.reshape(3, 3)


def _orientation_error_axis_world(q_des: np.ndarray, q_cur: np.ndarray) -> np.ndarray:
    """
    World-frame orientation error vector consistent with mj_jacBody rotational Jacobian.
    Uses vee(R_des @ R_cur.T - (R_des @ R_cur.T).T) / 2 (same as 0.5 * unskew(R R_err^skew)).
    """
    Rd = _rotmat_world_from_quat_wxyz(q_des)
    Rc = _rotmat_world_from_quat_wxyz(q_cur)
    Re = Rd @ Rc.T
    return 0.5 * np.array(
        [Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]],
        dtype=np.float64,
    )


def _geodesic_angle_rad(q_des: np.ndarray, q_cur: np.ndarray) -> float:
    """Angle between orientations (radians), from trace(R_des @ R_cur.T)."""
    Re = _rotmat_world_from_quat_wxyz(q_des) @ _rotmat_world_from_quat_wxyz(q_cur).T
    c = (float(np.trace(Re)) - 1.0) * 0.5
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


class TargetSE3:
    """Thread-safe target pose: position (m) + RPY (rad), MuJoCo euler 'xyz'."""

    def __init__(self, init_xyz: np.ndarray, init_rpy: np.ndarray) -> None:
        self._lock = threading.Lock()
        self._xyz = init_xyz.astype(float).copy()
        self._rpy = init_rpy.astype(float).copy()

    def get_pose(self) -> tuple[np.ndarray, np.ndarray]:
        with self._lock:
            return self._xyz.copy(), rpy_to_quat_wxyz(self._rpy)

    def set_xyz_axis(self, idx: int, value: float) -> None:
        with self._lock:
            self._xyz[idx] = float(value)

    def set_rpy_axis(self, idx: int, value: float) -> None:
        with self._lock:
            self._rpy[idx] = float(value)


def _joint_indices(model: mujoco.MjModel, joint_names: list[str]) -> np.ndarray:
    idx = []
    for name in joint_names:
        j_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if j_id < 0:
            raise ValueError(f"Joint not found: {name}")
        idx.append(int(model.jnt_qposadr[j_id]))
    return np.array(idx, dtype=int)


def _joint_ids(model: mujoco.MjModel, joint_names: list[str]) -> np.ndarray:
    ids = []
    for name in joint_names:
        j_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if j_id < 0:
            raise ValueError(f"Joint not found: {name}")
        ids.append(j_id)
    return np.array(ids, dtype=int)


def _left_actuator_ids(model: mujoco.MjModel) -> np.ndarray:
    ids = []
    for name in LEFT_JOINT_NAMES:
        a_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if a_id < 0:
            raise ValueError(f"Actuator not found (expected position actuator): {name}")
        ids.append(a_id)
    return np.array(ids, dtype=int)


def _clamp_to_limits(q: np.ndarray, q_min: np.ndarray, q_max: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(q, q_min), q_max)


def _center_bias(q: np.ndarray, q_min: np.ndarray, q_max: np.ndarray) -> np.ndarray:
    q_mid = 0.5 * (q_min + q_max)
    span = np.maximum(q_max - q_min, 1e-6)
    return (q_mid - q) / span


def solve_left_ik_once(
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
    """
    Damped least-squares IK for 6D pose (position + orientation).
    Returns: (q_solution, final_pos_error_norm, final_rot_error_norm, final_cond(J))
    """
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

        if float(np.linalg.norm(err_pos)) < cfg.pos_tol and _geodesic_angle_rad(
            tq, cq
        ) < cfg.rot_tol:
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


def solve_left_ik_multi(
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
    """
    Multi-solution handling:
    - Build several seeds (current, elbow-folded variants)
    - Solve each with local DLS IK
    - Choose best by residual + smoothness (close to previous q)
    """
    q_mid = 0.5 * (q_min + q_max)
    seeds = [
        q_prev.copy(),
        q_mid.copy(),
        q_prev.copy(),
        q_prev.copy(),
    ]
    # Two elbow-biased candidates to capture different branches.
    seeds[2][3] = q_min[3] + 0.15 * (q_max[3] - q_min[3])  # elbow relatively extended
    seeds[3][3] = q_min[3] + 0.85 * (q_max[3] - q_min[3])  # elbow relatively folded

    best = None
    best_cost = float("inf")
    for s in seeds:
        q_sol, err_p, err_r, cond = solve_left_ik_once(
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


def start_se3_slider_ui(
    shared_target: TargetSE3,
    init_xyz: np.ndarray,
    init_rpy: np.ndarray,
) -> None:
    def run() -> None:
        root = tk.Tk()
        root.title("Left Arm IK Target 6D (XYZ + RPY)")
        root.geometry("500x420")

        labels = [
            "X (m)",
            "Y (m)",
            "Z (m)",
            "Roll (deg)",
            "Pitch (deg)",
            "Yaw (deg)",
        ]
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
                command=lambda v, axis=j: shared_target.set_rpy_axis(
                    axis, float(np.deg2rad(float(v)))
                ),
            )
            s.set(float(init_deg[j]))
            s.pack(side="left", fill="x", expand=True)
            scales.append(s)

        home_xyz = np.array([0.0, 0.1555, 0.386], dtype=float)
        home_rpy_deg = np.rad2deg(init_rpy)

        def set_home() -> None:
            for i in range(3):
                scales[i].set(float(home_xyz[i]))
                shared_target.set_xyz_axis(i, float(home_xyz[i]))
            for j in range(3):
                scales[3 + j].set(float(home_rpy_deg[j]))
                shared_target.set_rpy_axis(j, float(init_rpy[j]))

        btn = tk.Button(root, text="Home (preset XYZ + start RPY)", command=set_home)
        btn.pack(pady=8)
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

    left_q_idx = _joint_indices(model, LEFT_JOINT_NAMES)
    left_j_ids = _joint_ids(model, LEFT_JOINT_NAMES)
    left_act_ids = _left_actuator_ids(model)
    left_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, LEFT_EE_BODY)
    if left_ee_id < 0:
        raise ValueError(f"Left EE body not found: {LEFT_EE_BODY}")

    q_min = model.jnt_range[left_j_ids, 0].copy()
    q_max = model.jnt_range[left_j_ids, 1].copy()

    # Initialize left arm at middle range and compute initial EE position.
    q_cur = 0.5 * (q_min + q_max)
    data.qpos[left_q_idx] = q_cur
    mujoco.mj_fwdPosition(model, data)
    init_target = data.xpos[left_ee_id].copy()
    init_quat = data.xquat[left_ee_id].copy()
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

    print("左臂 6D IK 实时求解已启动。")
    print("请在弹出的窗口用 XYZ + Roll/Pitch/Yaw 调节末端目标位姿。")
    print("终端会实时输出位置/姿态误差、关节角和奇异性指标。")

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

                q_sol, err_p, err_r, cond = solve_left_ik_multi(
                    model=model,
                    data=data,
                    q_idx=left_q_idx,
                    ee_body_id=left_ee_id,
                    q_prev=q_cur,
                    target_pos=target_pos,
                    target_quat=target_quat,
                    q_min=q_min,
                    q_max=q_max,
                    cfg=cfg,
                )
                q_cur = q_sol

                # Position actuators: set target joint angle directly.
                data.ctrl[left_act_ids] = q_cur
                ros_bridge.maybe_publish(tick, LEFT_JOINT_NAMES, q_cur)

                # Marker drawing API differs across mujoco versions.
                # If unsupported, skip visualization markers and keep IK running.
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
                    ee_q = data.xquat[left_ee_id].copy()
                    print(
                        f"\nTarget xyz : {np.array2string(target_pos, precision=4)}"
                        f"\nEE xyz     : {np.array2string(data.xpos[left_ee_id], precision=4)}"
                        f"\nPos err    : {err_p:.6f} m"
                        f"\nRot err    : {err_r:.6f} rad  (≈ {np.rad2deg(err_r):.3f} deg)"
                        f"\nTarget quat: {np.array2string(target_quat, precision=4)} (wxyz)"
                        f"\nEE quat    : {np.array2string(ee_q, precision=4)} (wxyz)"
                        f"\nCond(J6)   : {cond:.2f}"
                        f"\nq (deg)    : {np.array2string(q_deg, precision=2)}"
                    )
                    if cond > 300.0:
                        print("Warning: 接近奇异点，已自动提高阻尼并限制步长。")
                    if not can_add_marker:
                        print("Note: 当前 mujoco 版本不支持 viewer.add_marker，已自动关闭点标记显示。")

                viewer.sync()
                elapsed = time.time() - tick
                time.sleep(max(0.0, frame_dt - elapsed))
        finally:
            ros_bridge.close()


if __name__ == "__main__":
    main()

