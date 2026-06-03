from __future__ import annotations

import os
import threading
import time
import tkinter as tk
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
    TargetSE3,
    _joint_ids,
    _joint_indices,
    _left_actuator_ids,
    quat_wxyz_to_rpy_mujoco,
    solve_left_ik_multi,
)
from right_arm_ik_slider import (
    RIGHT_EE_BODY,
    RIGHT_JOINT_NAMES,
    _right_actuator_ids,
    solve_right_ik_multi,
)


ALL_JOINTS = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES
THIS_DIR = Path(__file__).resolve().parent
DEFAULT_CALIB_YAML = THIS_DIR / "calibration_mapping.yaml"


def _yaml_dump_calibration(path: Path, names: list[str], scale: np.ndarray, offset: np.ndarray, trim: np.ndarray) -> None:
    invert_names = [n for n, s in zip(names, scale) if s < 0.0]
    lines = [
        "joint_calibration:",
        "  joint_names:",
    ]
    for n in names:
        lines.append(f"    - {n}")
    lines += [
        "  scale:",
    ]
    for v in scale:
        lines.append(f"    - {float(v):.8f}")
    lines += [
        "  offset:",
    ]
    for v in offset:
        lines.append(f"    - {float(v):.8f}")
    lines += [
        "  trim:",
    ]
    for v in trim:
        lines.append(f"    - {float(v):.8f}")
    lines += [
        "  invert_joint_names:",
    ]
    for n in invert_names:
        lines.append(f"    - {n}")
        lines += [
        "",
        "joint_controller_node:",
        "  ros__parameters:",
        f"    invert_joint_names: \"{','.join(invert_names)}\"",
        "",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


class AppState:
    def __init__(self, q0_left: np.ndarray, q0_right: np.ndarray) -> None:
        self._lock = threading.Lock()
        self.mode = "direct"  # direct | ik
        self.arm = "left"  # left | right
        self.q_direct_left = q0_left.copy()
        self.q_direct_right = q0_right.copy()
        self.ik_target_left: TargetSE3 | None = None
        self.ik_target_right: TargetSE3 | None = None
        self.scale = np.ones(14, dtype=float)
        self.offset = np.zeros(14, dtype=float)
        self.trim = np.zeros(14, dtype=float)
        self.latest_actual = np.zeros(14, dtype=float)
        self.have_actual = False
        self.sim_zero = None
        self.act_zero = None
        self.sim_target = None
        self.act_target = None
        self.save_path = DEFAULT_CALIB_YAML
        self.status = "Ready"

    def snapshot(self):
        with self._lock:
            return (
                self.mode,
                self.arm,
                self.q_direct_left.copy(),
                self.q_direct_right.copy(),
                self.scale.copy(),
                self.offset.copy(),
                self.trim.copy(),
                self.latest_actual.copy(),
                self.have_actual,
                self.status,
            )

    def set_status(self, s: str) -> None:
        with self._lock:
            self.status = s


class ActualJointStateListener:
    def __init__(self, names: list[str], enabled: bool) -> None:
        self.names = names
        self.enabled = enabled
        self.latest = np.zeros(len(names), dtype=float)
        self.have = False
        self._ok = False
        self._rclpy = None
        self._node = None
        self._sub = None
        if not enabled:
            return
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
        except Exception:
            return
        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node("dual_arm_actual_listener")
        idx = {n: i for i, n in enumerate(names)}

        def cb(msg):
            if not msg.name or not msg.position:
                return
            out = self.latest.copy()
            for n, p in zip(msg.name, msg.position):
                j = idx.get(n)
                if j is not None:
                    out[j] = float(p)
            self.latest = out
            self.have = True

        self._sub = self._node.create_subscription(JointState, "/joint_states", cb, 10)
        self._ok = True

    def spin_once(self) -> None:
        if self._ok:
            self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        if not self._ok:
            return
        try:
            self._node.destroy_node()
        except Exception:
            pass


def build_ui(state: AppState, q_min_left: np.ndarray, q_max_left: np.ndarray, q_min_right: np.ndarray, q_max_right: np.ndarray) -> None:
    def run():
        root = tk.Tk()
        root.title("Dual Arm Calibration Console")
        root.geometry("1380x920")

        mode_var = tk.StringVar(value="direct")
        arm_var = tk.StringVar(value="left")
        status_var = tk.StringVar(value="Ready")

        def set_mode():
            state.mode = mode_var.get()

        def set_arm():
            state.arm = arm_var.get()

        top = tk.Frame(root)
        top.pack(fill="x", padx=8, pady=6)
        tk.Label(top, text="Control Mode:", font=("TkDefaultFont", 10, "bold")).pack(side="left")
        tk.Radiobutton(top, text="7-Joint Direct", variable=mode_var, value="direct", command=set_mode).pack(side="left", padx=8)
        tk.Radiobutton(top, text="6D IK", variable=mode_var, value="ik", command=set_mode).pack(side="left", padx=8)
        tk.Label(top, text="Active Arm:", font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=(20, 0))
        tk.Radiobutton(top, text="Left", variable=arm_var, value="left", command=set_arm).pack(side="left", padx=6)
        tk.Radiobutton(top, text="Right", variable=arm_var, value="right", command=set_arm).pack(side="left", padx=6)

        # One-click home
        def on_home():
            state.q_direct_left[:] = 0.0
            state.q_direct_right[:] = 0.0
            if state.ik_target_left is not None:
                state.ik_target_left.set_xyz_axis(0, 0.0)
                state.ik_target_left.set_xyz_axis(1, 0.1555)
                state.ik_target_left.set_xyz_axis(2, 0.386)
                state.ik_target_left.set_rpy_axis(0, 0.0)
                state.ik_target_left.set_rpy_axis(1, 0.0)
                state.ik_target_left.set_rpy_axis(2, 0.0)
            if state.ik_target_right is not None:
                state.ik_target_right.set_xyz_axis(0, 0.0)
                state.ik_target_right.set_xyz_axis(1, -0.1555)
                state.ik_target_right.set_xyz_axis(2, 0.386)
                state.ik_target_right.set_rpy_axis(0, 0.0)
                state.ik_target_right.set_rpy_axis(1, 0.0)
                state.ik_target_right.set_rpy_axis(2, 0.0)
            state.set_status("Home applied")

        tk.Button(top, text="一键回0", command=on_home, bg="#E8F5E9").pack(side="right")

        main = tk.Frame(root)
        main.pack(fill="both", expand=True, padx=8, pady=4)
        col_left = tk.Frame(main)
        col_left.pack(side="left", fill="both", expand=True, padx=(0, 6))
        col_right = tk.Frame(main)
        col_right.pack(side="left", fill="both", expand=True, padx=(6, 0))

        def add_slider_entry(
            parent,
            label: str,
            vmin: float,
            vmax: float,
            res: float,
            init: float,
            on_change,
            *,
            show_rad_deg: bool = False,
        ):
            row = tk.Frame(parent)
            row.pack(fill="x", pady=2)
            tk.Label(row, text=label, width=16, anchor="w").pack(side="left")
            scale = tk.Scale(
                row,
                from_=vmin,
                to=vmax,
                resolution=res,
                orient=tk.HORIZONTAL,
                length=280,
                showvalue=False,
                command=lambda v: _on_scale(float(v)),
            )
            scale.pack(side="left", fill="x", expand=True)
            ent_var = tk.StringVar(value=f"{init:.4f}")
            ent = tk.Entry(row, textvariable=ent_var, width=10)
            ent.pack(side="left", padx=6)
            unit_var = tk.StringVar(value="")
            unit_lbl = tk.Label(row, textvariable=unit_var, width=22, anchor="w", fg="gray")
            unit_lbl.pack(side="left")

            updating = {"flag": False}

            def _refresh_units(v: float):
                if show_rad_deg:
                    unit_var.set(f"{v:+.3f} rad | {np.rad2deg(v):+.1f} deg")
                else:
                    unit_var.set(f"{v:+.4f}")

            def _set_value(v: float):
                v = float(np.clip(v, vmin, vmax))
                updating["flag"] = True
                scale.set(v)
                ent_var.set(f"{v:.4f}")
                _refresh_units(v)
                updating["flag"] = False
                on_change(v)

            def _on_scale(v: float):
                if updating["flag"]:
                    return
                updating["flag"] = True
                ent_var.set(f"{v:.4f}")
                _refresh_units(v)
                updating["flag"] = False
                on_change(v)

            def _on_entry(_event=None):
                try:
                    v = float(ent_var.get())
                except Exception:
                    ent_var.set(f"{float(scale.get()):.4f}")
                    return
                _set_value(v)

            ent.bind("<Return>", _on_entry)
            ent.bind("<FocusOut>", _on_entry)
            scale.set(init)
            _refresh_units(float(init))
            return scale, ent

        # Direct control sliders
        direct_box = tk.LabelFrame(col_left, text="7-Joint Direct (active arm)", padx=6, pady=6)
        direct_box.pack(fill="x", padx=0, pady=6)

        direct_scales: list[tk.Scale] = []
        for i in range(7):
            def mk_cmd(j):
                def _cmd(v):
                    val = float(v)
                    if arm_var.get() == "left":
                        state.q_direct_left[j] = val
                    else:
                        state.q_direct_right[j] = val
                return _cmd

            s, _ = add_slider_entry(
                direct_box,
                f"J{i+1}",
                -3.14,
                3.14,
                0.001,
                0.0,
                mk_cmd(i),
                show_rad_deg=True,
            )
            direct_scales.append(s)

        # IK 6D sliders
        ik_box = tk.LabelFrame(col_left, text="6D IK Target (active arm)", padx=6, pady=6)
        ik_box.pack(fill="x", padx=0, pady=6)
        labels = [("X", -0.9, 0.9, 0.001), ("Y", -0.9, 0.9, 0.001), ("Z", 0.0, 1.4, 0.001),
                  ("Roll(deg)", -180, 180, 0.5), ("Pitch(deg)", -180, 180, 0.5), ("Yaw(deg)", -180, 180, 0.5)]

        for i, (name, mn, mx, res) in enumerate(labels):
            def mk_ik_cmd(k):
                def _cmd(v):
                    val = float(v)
                    tgt = state.ik_target_left if arm_var.get() == "left" else state.ik_target_right
                    if tgt is None:
                        return
                    if k < 3:
                        tgt.set_xyz_axis(k, val)
                    else:
                        tgt.set_rpy_axis(k - 3, np.deg2rad(val))
                return _cmd

            add_slider_entry(
                ik_box,
                name,
                mn,
                mx,
                res,
                0.0,
                mk_ik_cmd(i),
                show_rad_deg=(i >= 3),
            )

        # 14 trims
        trim_box = tk.LabelFrame(col_right, text="14-joint trim 微调(实际对齐仿真)", padx=6, pady=6)
        trim_box.pack(fill="both", expand=True, padx=0, pady=6)

        for i, n in enumerate(ALL_JOINTS):
            def mk_trim_cmd(j):
                def _cmd(v):
                    state.trim[j] = float(v)
                return _cmd

            add_slider_entry(
                trim_box,
                n,
                -0.5,
                0.5,
                0.0005,
                0.0,
                mk_trim_cmd(i),
                show_rad_deg=True,
            )

        # Calibration buttons
        cal_box = tk.LabelFrame(col_right, text="Calibration: 零点+目标点标定", padx=6, pady=6)
        cal_box.pack(fill="x", padx=0, pady=6)

        def capture_zero():
            state.sim_zero = np.concatenate([state.q_direct_left.copy(), state.q_direct_right.copy()])
            if state.have_actual:
                state.act_zero = state.latest_actual.copy()
                state.set_status("Captured ZERO(sim+actual)")
            else:
                state.set_status("Captured ZERO(sim). Waiting /joint_states for actual.")

        def capture_target_and_fit():
            state.sim_target = np.concatenate([state.q_direct_left.copy(), state.q_direct_right.copy()])
            if not state.have_actual:
                state.set_status("No /joint_states actual yet.")
                return
            state.act_target = state.latest_actual.copy()
            if state.sim_zero is None or state.act_zero is None:
                state.set_status("Capture ZERO pair first.")
                return
            ds = state.sim_target - state.sim_zero
            da = state.act_target - state.act_zero
            scale = np.ones(14, dtype=float)
            valid = np.abs(ds) > 1e-4
            scale[valid] = da[valid] / ds[valid]
            scale[~valid] = 1.0
            offset = state.act_zero - scale * state.sim_zero
            state.scale = scale
            state.offset = offset
            state.set_status("Fitted scale/offset from ZERO+TARGET.")

        def save_yaml():
            path = Path(state.save_path)
            _yaml_dump_calibration(path, ALL_JOINTS, state.scale, state.offset, state.trim)
            state.set_status(f"Saved: {path}")

        tk.Button(cal_box, text="1) 捕获零点对齐", command=capture_zero).pack(side="left", padx=6)
        tk.Button(cal_box, text="2) 捕获目标点并标定", command=capture_target_and_fit).pack(side="left", padx=6)
        tk.Button(cal_box, text="写入YAML", command=save_yaml, bg="#E3F2FD").pack(side="left", padx=6)

        path_row = tk.Frame(cal_box)
        path_row.pack(fill="x", pady=6)
        tk.Label(path_row, text="YAML路径:", width=10).pack(side="left")
        pvar = tk.StringVar(value=str(state.save_path))
        ent = tk.Entry(path_row, textvariable=pvar, width=80)
        ent.pack(side="left", fill="x", expand=True)

        def on_path(*_):
            state.save_path = Path(pvar.get())

        pvar.trace_add("write", on_path)

        status = tk.Label(root, textvariable=status_var, fg="blue")
        status.pack(fill="x", padx=8, pady=4)

        def poll():
            _, _, _, _, _, _, _, _, _, s = state.snapshot()
            status_var.set(s)
            # refresh direct sliders to selected arm values
            q = state.q_direct_left if arm_var.get() == "left" else state.q_direct_right
            for i in range(7):
                if abs(float(direct_scales[i].get()) - float(q[i])) > 1e-6:
                    direct_scales[i].set(float(q[i]))
            root.after(100, poll)

        poll()
        root.mainloop()

    threading.Thread(target=run, daemon=True).start()


def main() -> None:
    model_path = os.environ.get("LENS_DUAL_ARM_MJCF", DEFAULT_MODEL_PATH)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model not found: {model_path}")

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    ik_cfg = IkConfig()

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

    q_min_left = model.jnt_range[left_j_ids, 0].copy()
    q_max_left = model.jnt_range[left_j_ids, 1].copy()
    q_min_right = model.jnt_range[right_j_ids, 0].copy()
    q_max_right = model.jnt_range[right_j_ids, 1].copy()

    q_left = np.zeros(7, dtype=float)
    q_right = np.zeros(7, dtype=float)
    data.qpos[left_q_idx] = q_left
    data.qpos[right_q_idx] = q_right
    mujoco.mj_fwdPosition(model, data)

    left_xyz0 = data.xpos[left_ee_id].copy()
    left_rpy0 = quat_wxyz_to_rpy_mujoco(data.xquat[left_ee_id].copy())
    right_xyz0 = data.xpos[right_ee_id].copy()
    right_rpy0 = quat_wxyz_to_rpy_mujoco(data.xquat[right_ee_id].copy())

    state = AppState(q_left, q_right)
    state.ik_target_left = TargetSE3(left_xyz0, left_rpy0)
    state.ik_target_right = TargetSE3(right_xyz0, right_rpy0)

    ros_pub = Ros2JointCommandPublisher(
        Ros2BridgeConfig(
            enabled=os.environ.get("LENS_ROS_BRIDGE", "1").strip().lower() in ("1", "true", "yes", "on"),
            topic=os.environ.get("LENS_ROS_TOPIC", "/joint_command"),
            hz=float(os.environ.get("LENS_ROS_HZ", "120")),
        )
    )
    actual_listener = ActualJointStateListener(ALL_JOINTS, enabled=True)

    build_ui(state, q_min_left, q_max_left, q_min_right, q_max_right)
    print("Calibration console started.")
    print("Modes: direct 7-joint / 6D IK, switchable in real-time.")
    print(f"YAML default output: {DEFAULT_CALIB_YAML}")

    fps = float(os.environ.get("LENS_SIM_FPS", "120"))
    frame_dt = 1.0 / max(fps, 1.0)
    steps_per_frame = max(1, int(round(frame_dt / model.opt.timestep)))

    with mujoco.viewer.launch_passive(model, data) as viewer:
        try:
            while viewer.is_running():
                tic = time.time()
                mode, arm, qd_l, qd_r, scale, offset, trim, _, _, _ = state.snapshot()
                actual_listener.spin_once()
                if actual_listener.have:
                    state.latest_actual = actual_listener.latest.copy()
                    state.have_actual = True

                if mode == "direct":
                    q_left = np.clip(qd_l, q_min_left, q_max_left)
                    q_right = np.clip(qd_r, q_min_right, q_max_right)
                else:
                    # IK mode only controls active arm, other arm keeps current direct values.
                    if arm == "left":
                        tgt_pos, tgt_quat = state.ik_target_left.get_pose()
                        q_left, *_ = solve_left_ik_multi(
                            model=model,
                            data=data,
                            q_idx=left_q_idx,
                            ee_body_id=left_ee_id,
                            q_prev=q_left,
                            target_pos=tgt_pos,
                            target_quat=tgt_quat,
                            q_min=q_min_left,
                            q_max=q_max_left,
                            cfg=ik_cfg,
                        )
                        q_right = np.clip(qd_r, q_min_right, q_max_right)
                    else:
                        tgt_pos, tgt_quat = state.ik_target_right.get_pose()
                        q_right, *_ = solve_right_ik_multi(
                            model=model,
                            data=data,
                            q_idx=right_q_idx,
                            ee_body_id=right_ee_id,
                            q_prev=q_right,
                            target_pos=tgt_pos,
                            target_quat=tgt_quat,
                            q_min=q_min_right,
                            q_max=q_max_right,
                            cfg=ik_cfg,
                        )
                        q_left = np.clip(qd_l, q_min_left, q_max_left)

                q_sim = np.concatenate([q_left, q_right])
                q_hw = scale * q_sim + offset + trim

                data.ctrl[left_act_ids] = q_left
                data.ctrl[right_act_ids] = q_right
                ros_pub.maybe_publish(tic, ALL_JOINTS, q_hw)

                for _ in range(steps_per_frame):
                    mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(max(0.0, frame_dt - (time.time() - tic)))
        finally:
            ros_pub.close()
            actual_listener.close()


if __name__ == "__main__":
    main()

