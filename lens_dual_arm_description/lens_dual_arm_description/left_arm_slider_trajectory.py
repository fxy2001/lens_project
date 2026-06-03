"""
左臂：滑条设定末端目标位置 → 从当前构型关节空间规划（RRT-Connect）→ 自动执行轨迹。

与 `left_arm_path_planner.py` 的区别：单键「规划并移动」，无需先 Plan 再 Execute；
目标关节由 **3D 位置 IK** 求得（仅约束末端位置，成功率高）。若 RRT 未连接成功，则尝试
关节空间直线插值（全程在限位内时作为回退路径）。
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
    LEFT_EE_BODY,
    LEFT_JOINT_NAMES,
    _joint_ids,
    _joint_indices,
    _left_actuator_ids,
)
from left_arm_ik_slider_3d import IkConfig3D, solve_left_ik_multi_3d
from left_arm_path_planner import (
    DEFAULT_MODEL_PATH,
    PlannerConfig,
    densify_path,
    rrt_connect_plan,
    shortcut_smooth,
)
from left_arm_plan_collision import LeftArmBodyCollisionChecker, print_joint_space_plan_failure
from motor_can_protocol import MitRanges, parse_int_list_csv
from realtime_bridge import RealTimeBridge


@dataclass
class Ros2BridgeConfig:
    enabled: bool = False
    topic: str = "/joint_command"
    hz: float = 60.0


class Ros2JointCommandPublisher:
    """
    可选 ROS2 桥接：将当前左臂关节命令发布为 JointState，供真机执行节点订阅。
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
        self._node = Node("left_arm_slider_ros_bridge")
        self._pub = self._node.create_publisher(JointState, cfg.topic, 10)
        self._JointState = JointState
        self._ok = True
        print(f"[ROS Bridge] enabled, topic={cfg.topic}, hz={cfg.hz}")

    @property
    def enabled(self) -> bool:
        return self._ok

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
        # 轻量驱动回调队列，避免积压
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


def _left_arm_geom_ids(model: mujoco.MjModel) -> list[int]:
    """属于 Left_* 刚体链的几何体 id，用于虚影绘制。"""
    left_bodies: set[int] = set()
    for b in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        if name and name.startswith("Left_"):
            left_bodies.add(b)
    return [g for g in range(model.ngeom) if model.geom_bodyid[g] in left_bodies]


def _copy_mjv_geom_to(dst, src) -> None:
    """将 mjvGeom 可视化字段复制到 user_scn 槽位（mesh 需带 dataid/matid）。"""
    dst.type = int(src.type)
    dst.dataid = int(src.dataid)
    dst.objtype = int(src.objtype)
    dst.objid = int(src.objid)
    dst.category = int(src.category)
    dst.matid = int(src.matid)
    dst.texcoord = int(src.texcoord)
    dst.segid = int(src.segid)
    dst.size[:] = src.size
    dst.pos[:] = src.pos
    dst.mat[:] = src.mat
    dst.emission = float(src.emission)
    dst.specular = float(src.specular)
    dst.shininess = float(src.shininess)
    dst.reflectance = float(src.reflectance)
    dst.modelrbound = float(src.modelrbound)


def _build_left_arm_ghost_overlay(
    viewer: mujoco.viewer.Handle,
    model: mujoco.MjModel,
    data_ghost: mujoco.MjData,
    ghost_tmp_scn: mujoco.MjvScene,
    left_geom_ids: set[int],
    rgba: np.ndarray,
) -> None:
    """
    用 mjv_updateScene 生成完整抽象几何，再筛出左臂复制到 user_scn。
    passive viewer 要求：改 user_scn 须在 viewer.lock() 内，并在 sync() 前完成。
    """
    uscn = viewer.user_scn
    if uscn is None:
        return
    mujoco.mjv_updateScene(
        model,
        data_ghost,
        viewer.opt,
        viewer.perturb,
        viewer.cam,
        mujoco.mjtCatBit.mjCAT_ALL.value,
        ghost_tmp_scn,
    )
    uscn.ngeom = 0
    w = 0
    obj_geom = int(mujoco.mjtObj.mjOBJ_GEOM)
    for i in range(ghost_tmp_scn.ngeom):
        sg = ghost_tmp_scn.geoms[i]
        if int(sg.objtype) != obj_geom:
            continue
        if int(sg.objid) not in left_geom_ids:
            continue
        if w >= uscn.maxgeom:
            break
        dg = uscn.geoms[w]
        _copy_mjv_geom_to(dg, sg)
        dg.rgba[:] = rgba
        dg.transparent = 1
        w += 1
    uscn.ngeom = w


def _try_straight_joint_path(
    q_a: np.ndarray,
    q_b: np.ndarray,
    q_min: np.ndarray,
    q_max: np.ndarray,
    max_step: float,
    collision_checker: LeftArmBodyCollisionChecker | None = None,
    collision_samples: int = 8,
    collision_max_step: float = 0.06,
    collision_max_samples: int = 256,
) -> list[np.ndarray] | None:
    """若 q_a→q_b 的关节空间直线段全程在限位内（且可选通过身体避障），则返回离散路径。"""
    d = float(np.linalg.norm(q_b - q_a))
    if d < 1e-9:
        if collision_checker is not None and not collision_checker.is_valid(q_a):
            return None
        return [q_a.copy()]
    n = max(1, int(np.ceil(d / max_step)))
    out: list[np.ndarray] = []
    q_prev = q_a.copy()
    if collision_checker is not None and not collision_checker.is_valid(q_prev):
        return None
    out.append(q_prev.copy())
    for k in range(1, n + 1):
        t = k / n
        q = q_a + (q_b - q_a) * t
        if not (np.all(q >= q_min) and np.all(q <= q_max)):
            return None
        if collision_checker is not None and not collision_checker.segment_valid_by_step(
            q_prev,
            q,
            max_step=collision_max_step,
            min_samples=collision_samples,
            max_samples=collision_max_samples,
        ):
            return None
        out.append(q.copy())
        q_prev = q
    return out


class UiState:
    def __init__(self, init_goal: np.ndarray) -> None:
        self.goal_xyz = init_goal.astype(float).copy()
        self._lock = threading.Lock()
        self.request_move = False
        self.busy = False
        self.status = "就绪：拖动滑条后点「规划并移动」"

    def set_goal_axis(self, idx: int, val: float) -> None:
        with self._lock:
            self.goal_xyz[idx] = float(val)

    def try_request_move(self) -> bool:
        with self._lock:
            if self.busy:
                return False
            self.request_move = True
            return True

    def take_move_request(self) -> bool:
        with self._lock:
            if not self.request_move or self.busy:
                return False
            self.request_move = False
            self.busy = True
            return True

    def snapshot_goal(self) -> np.ndarray:
        with self._lock:
            return self.goal_xyz.copy()

    def set_status(self, s: str) -> None:
        with self._lock:
            self.status = s

    def get_status(self) -> str:
        with self._lock:
            return self.status

    def finish_busy(self) -> None:
        with self._lock:
            self.busy = False


def start_ui(ui: UiState) -> None:
    def run() -> None:
        root = tk.Tk()
        root.title("左臂 — 滑条目标 + 轨迹移动")
        root.geometry("520x300")

        g0 = ui.snapshot_goal()
        labels = ["目标 X (m)", "目标 Y (m)", "目标 Z (m)"]
        mins = [-0.9, -0.9, 0.0]
        maxs = [0.9, 0.9, 1.5]
        scales: list[tk.Scale] = []

        for i in range(3):
            frame = tk.Frame(root)
            frame.pack(fill="x", padx=8, pady=4)
            tk.Label(frame, text=labels[i], width=12, anchor="w").pack(side="left")
            s = tk.Scale(
                frame,
                from_=mins[i],
                to=maxs[i],
                resolution=0.001,
                orient=tk.HORIZONTAL,
                length=360,
                command=lambda v, axis=i: ui.set_goal_axis(axis, float(v)),
            )
            s.set(float(g0[i]))
            s.pack(side="left", fill="x", expand=True)
            scales.append(s)

        status_var = tk.StringVar(value=ui.get_status())

        def on_move() -> None:
            if not ui.try_request_move():
                ui.set_status("忙：请等待当前轨迹执行完成")
            else:
                ui.set_status("已请求规划…")
            status_var.set(ui.get_status())

        def on_home_goal() -> None:
            home = np.array([0.0, 0.1555, 0.386], dtype=float)
            for i, sc in enumerate(scales):
                sc.set(float(home[i]))
                ui.set_goal_axis(i, float(home[i]))
            ui.set_status("目标已设为 Home 预设")
            status_var.set(ui.get_status())

        btn_row = tk.Frame(root)
        btn_row.pack(fill="x", padx=8, pady=8)
        tk.Button(btn_row, text="规划并移动", width=14, command=on_move).pack(
            side="left", padx=4
        )
        tk.Button(btn_row, text="目标 Home", width=10, command=on_home_goal).pack(
            side="left", padx=4
        )
        tk.Label(btn_row, textvariable=status_var, anchor="w").pack(
            side="left", padx=8, fill="x", expand=True
        )

        def poll() -> None:
            status_var.set(ui.get_status())
            root.after(200, poll)

        poll()
        root.mainloop()

    threading.Thread(target=run, daemon=True).start()


def main() -> None:
    model_path = os.environ.get("LENS_DUAL_ARM_MJCF", DEFAULT_MODEL_PATH)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model not found: {model_path}")

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    ik_cfg = IkConfig3D()
    planner_cfg = PlannerConfig(
        max_iters=6000,
        step_size=0.14,
        connect_threshold=0.14,
        goal_sample_rate=0.28,
    )
    rng = np.random.default_rng(20260412)
    bridge_protocol = os.environ.get("LENS_BRIDGE_PROTOCOL", "can").strip().lower()
    can_ids_env = os.environ.get("LENS_LEFT_CAN_IDS", "1,2,3,4,5,6,7")
    left_can_ids = parse_int_list_csv(can_ids_env, 7)
    bridge_print_hz = float(os.environ.get("LENS_BRIDGE_PRINT_HZ", os.environ.get("LENS_CAN_PRINT_HZ", "30")))
    mit_ranges = MitRanges()
    mit_kp = float(os.environ.get("LENS_MIT_KP", "20"))
    mit_kd = float(os.environ.get("LENS_MIT_KD", "8"))
    mit_vel = float(os.environ.get("LENS_MIT_VEL", "0"))
    mit_tor = float(os.environ.get("LENS_MIT_TOR", "0"))
    bridge = RealTimeBridge(
        protocol=bridge_protocol,
        left_joint_names=LEFT_JOINT_NAMES,
        left_can_ids=left_can_ids,
        print_hz=bridge_print_hz,
        mit_ranges=mit_ranges,
        mit_vel=mit_vel,
        mit_kp=mit_kp,
        mit_kd=mit_kd,
        mit_tor=mit_tor,
        ethercat_ifname=os.environ.get("LENS_ETHERCAT_IFNAME", "enp3s0"),
        actuator_config_path=os.environ.get(
            "LENS_ACTUATOR_CONFIG",
            os.path.join(
                os.path.dirname(DEFAULT_MODEL_PATH),
                "..",
                "actuator_SDK",
                "actuator_SDK",
                "actuator_config.json",
            ),
        ),
        actuator_params_path=os.environ.get(
            "LENS_ACTUATOR_PARAMS",
            os.path.join(
                os.path.dirname(DEFAULT_MODEL_PATH),
                "..",
                "actuator_SDK",
                "actuator_SDK",
                "actuator_params_config.json",
            ),
        ),
        ethercat_send_frequency=int(os.environ.get("LENS_ETHERCAT_SEND_FREQUENCY", "500")),
    )
    ros_bridge = Ros2JointCommandPublisher(
        Ros2BridgeConfig(
            enabled=os.environ.get("LENS_ROS_BRIDGE", "0").strip().lower() in ("1", "true", "yes", "on"),
            topic=os.environ.get("LENS_ROS_TOPIC", "/joint_command"),
            hz=float(os.environ.get("LENS_ROS_HZ", "60")),
        )
    )

    q_idx = _joint_indices(model, LEFT_JOINT_NAMES)
    j_ids = _joint_ids(model, LEFT_JOINT_NAMES)
    act_ids = _left_actuator_ids(model)
    ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, LEFT_EE_BODY)
    if ee_id < 0:
        raise ValueError(f"Left EE body not found: {LEFT_EE_BODY}")

    q_min = model.jnt_range[j_ids, 0].copy()
    q_max = model.jnt_range[j_ids, 1].copy()

    q_cur = 0.5 * (q_min + q_max)
    data.qpos[q_idx] = q_cur
    mujoco.mj_fwdPosition(model, data)
    goal_init = data.xpos[ee_id].copy()

    collision_checker = LeftArmBodyCollisionChecker(
        model, q_idx, clearance_min=planner_cfg.clearance_min
    )
    collision_checker.set_qpos_template(data.qpos.copy())

    ui = UiState(goal_init)
    start_ui(ui)

    trajectory: list[np.ndarray] = []
    traj_ptr = 0
    executing = False
    # 到达目标后保持关节指令为 IK 解 q_goal，避免每帧 ctrl=qpos 产生漂移
    q_hold: np.ndarray | None = None
    pending_q_hold: np.ndarray | None = None

    data_ghost = mujoco.MjData(model)
    ghost_geom_ids = _left_arm_geom_ids(model)
    left_geom_set = set(ghost_geom_ids)
    ghost_tmp_scn = mujoco.MjvScene(model, maxgeom=model.ngeom + 64)
    ghost_rgba = np.array([0.28, 0.72, 1.0, 0.52], dtype=np.float32)
    last_ghost_t = 0.0
    ghost_hz = 25.0

    print("左臂滑条轨迹：在窗口设目标 XYZ，点「规划并移动」。")
    print("起点为当前仿真关节角；目标仅约束末端位置（3D IK 求目标关节）。")
    print("拖动滑条时显示浅蓝色半透明左臂虚影（IK 预览）；执行轨迹时关闭虚影。")

    target_fps = 60.0
    frame_dt = 1.0 / target_fps
    steps_per_frame = max(1, int(round(frame_dt / model.opt.timestep)))

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            tick = time.time()

            with viewer.lock():
                if ui.take_move_request():
                    q_start = data.qpos[q_idx].copy()
                    data.qpos[q_idx] = q_start
                    mujoco.mj_fwdPosition(model, data)
                    collision_checker.set_qpos_template(data.qpos.copy())
                    goal_xyz = ui.snapshot_goal()

                    ui.set_status("规划中 (3D IK + 路径)…")

                    q_goal, ik_ep, _cond = solve_left_ik_multi_3d(
                        model=model,
                        data=data,
                        q_idx=q_idx,
                        ee_body_id=ee_id,
                        q_prev=q_start,
                        target_pos=goal_xyz,
                        q_min=q_min,
                        q_max=q_max,
                        cfg=ik_cfg,
                    )
                    data.qpos[q_idx] = q_start
                    mujoco.mj_fwdPosition(model, data)

                    if ik_ep > 0.08:
                        print(
                            f"\n[IK 失败] 位置残差 {ik_ep:.4f} m > 0.08 | "
                            f"目标 XYZ: {np.array2string(goal_xyz, precision=4)}\n",
                            flush=True,
                        )
                        ui.set_status(
                            f"IK 失败（末端位置残差 {ik_ep:.3f} m>0.08），请改近一点"
                        )
                        ui.finish_busy()
                    else:
                        raw = rrt_connect_plan(
                            q_start,
                            q_goal,
                            q_min,
                            q_max,
                            planner_cfg,
                            rng,
                            collision_checker=collision_checker,
                        )
                        if raw is None:
                            raw = _try_straight_joint_path(
                                q_start,
                                q_goal,
                                q_min,
                                q_max,
                                max_step=0.12,
                                collision_checker=collision_checker,
                                collision_samples=planner_cfg.collision_samples,
                                collision_max_step=planner_cfg.collision_max_step,
                                collision_max_samples=planner_cfg.collision_max_samples,
                            )
                            if raw is None:
                                print_joint_space_plan_failure(
                                    goal_xyz,
                                    ik_ep,
                                    q_start,
                                    q_goal,
                                    q_min,
                                    q_max,
                                    collision_checker,
                                    max_iters=planner_cfg.max_iters,
                                    step_size=planner_cfg.step_size,
                                    connect_threshold=planner_cfg.connect_threshold,
                                    clearance_min=planner_cfg.clearance_min,
                                    collision_samples=planner_cfg.collision_samples,
                                    straight_max_step=0.12,
                                )
                                ui.set_status(
                                    "RRT 失败且关节直线插值不可行，请改目标或换起点（详见终端）"
                                )
                                ui.finish_busy()
                            else:
                                ui.set_status("RRT 失败，已用关节直线插值回退")
                        if raw is not None:
                            sm = shortcut_smooth(
                                raw,
                                q_min,
                                q_max,
                                planner_cfg.smoothing_trials,
                                rng,
                                collision_checker=collision_checker,
                                collision_samples=planner_cfg.collision_samples,
                                collision_max_step=planner_cfg.collision_max_step,
                                collision_max_samples=planner_cfg.collision_max_samples,
                            )
                            trajectory = densify_path(sm, max_step=0.025)
                            traj_ptr = 0
                            executing = True
                            pending_q_hold = q_goal.copy()
                            ui.set_status(f"执行中… ({len(trajectory)} 点)")

                if executing and traj_ptr < len(trajectory):
                    data.ctrl[act_ids] = trajectory[traj_ptr]
                    traj_ptr += 1
                    if traj_ptr >= len(trajectory):
                        executing = False
                        trajectory = []
                        if pending_q_hold is not None:
                            q_hold = pending_q_hold.copy()
                            pending_q_hold = None
                        ui.set_status(
                            "完成并保持目标姿态。可改目标再次「规划并移动」"
                        )
                        ui.finish_busy()
                else:
                    if q_hold is not None:
                        data.ctrl[act_ids] = q_hold
                    else:
                        data.ctrl[act_ids] = data.qpos[q_idx].copy()

                # 实时输出「真实协议控制下发」（CAN 或 EtherCAT）
                bridge.maybe_emit(time.time(), data.ctrl[act_ids].copy())
                # 可选 ROS2 发布：把同一份轨迹命令同步给真机执行链路
                ros_bridge.maybe_publish(time.time(), LEFT_JOINT_NAMES, data.ctrl[act_ids].copy())

                for _ in range(steps_per_frame):
                    mujoco.mj_step(model, data)

                if viewer.user_scn is not None:
                    if executing:
                        viewer.user_scn.ngeom = 0
                    else:
                        now = time.time()
                        if now - last_ghost_t >= 1.0 / ghost_hz:
                            last_ghost_t = now
                            goal_xyz = ui.snapshot_goal()
                            qp_save = data.qpos.copy()
                            q_pv, ep, _ = solve_left_ik_multi_3d(
                                model=model,
                                data=data,
                                q_idx=q_idx,
                                ee_body_id=ee_id,
                                q_prev=data.qpos[q_idx].copy(),
                                target_pos=goal_xyz,
                                q_min=q_min,
                                q_max=q_max,
                                cfg=ik_cfg,
                            )
                            data.qpos[:] = qp_save
                            mujoco.mj_forward(model, data)
                            if ep <= 0.08:
                                data_ghost.qpos[:] = qp_save
                                data_ghost.qpos[q_idx] = q_pv
                                mujoco.mj_forward(model, data_ghost)
                                _build_left_arm_ghost_overlay(
                                    viewer,
                                    model,
                                    data_ghost,
                                    ghost_tmp_scn,
                                    left_geom_set,
                                    ghost_rgba,
                                )
                            else:
                                viewer.user_scn.ngeom = 0

            viewer.sync()
            elapsed = time.time() - tick
            time.sleep(max(0.0, frame_dt - elapsed))

    ros_bridge.close()


if __name__ == "__main__":
    main()
