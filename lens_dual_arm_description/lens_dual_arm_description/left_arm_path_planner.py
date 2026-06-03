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
    IkConfig,
    _joint_ids,
    _joint_indices,
    _left_actuator_ids,
    solve_left_ik_multi,
)
from left_arm_plan_collision import LeftArmBodyCollisionChecker, print_joint_space_plan_failure
from motor_can_protocol import MitRanges, parse_int_list_csv
from realtime_bridge import RealTimeBridge


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL_PATH = os.path.join(THIS_DIR, "mjcf", "mj_lens_dual_arm.xml")


@dataclass
class PlannerConfig:
    max_iters: int = 2500
    step_size: float = 0.12
    connect_threshold: float = 0.10
    goal_sample_rate: float = 0.25
    smoothing_trials: int = 120
    # 路径段碰撞采样点数（含端点）；与 collision_max_step 二选一（优先 max_step）
    collision_samples: int = 8
    # 按关节空间距离自适应采样：让相邻采样点间 ||dq|| ≤ collision_max_step
    # 解决大角度变化时采样太稀导致“穿过身体区域”
    collision_max_step: float = 0.06
    collision_max_samples: int = 256
    # mj_geomDistance 符号距离下限（略负可容忍网格离散误差）
    clearance_min: float = -0.012


class SharedState:
    def __init__(self, init_goal: np.ndarray) -> None:
        self._lock = threading.Lock()
        self.goal_xyz = init_goal.copy()
        self.request_plan = False
        self.request_execute = False
        self.last_plan_ok = False
        self.plan_status = "idle"

    def set_goal_axis(self, idx: int, val: float) -> None:
        with self._lock:
            self.goal_xyz[idx] = float(val)

    def snapshot(self) -> tuple[np.ndarray, bool, bool, bool, str]:
        with self._lock:
            return (
                self.goal_xyz.copy(),
                self.request_plan,
                self.request_execute,
                self.last_plan_ok,
                self.plan_status,
            )

    def consume_plan_request(self) -> bool:
        with self._lock:
            req = self.request_plan
            self.request_plan = False
            return req

    def consume_execute_request(self) -> bool:
        with self._lock:
            req = self.request_execute
            self.request_execute = False
            return req

    def set_plan_result(self, ok: bool, status: str) -> None:
        with self._lock:
            self.last_plan_ok = ok
            self.plan_status = status


def _distance(q1: np.ndarray, q2: np.ndarray) -> float:
    return float(np.linalg.norm(q1 - q2))


def _steer(q_from: np.ndarray, q_to: np.ndarray, step: float) -> np.ndarray:
    d = q_to - q_from
    n = np.linalg.norm(d)
    if n <= step:
        return q_to.copy()
    return q_from + d / (n + 1e-12) * step


def _within_limits(q: np.ndarray, q_min: np.ndarray, q_max: np.ndarray) -> bool:
    return bool(np.all(q >= q_min) and np.all(q <= q_max))


def _sample_q(
    rng: np.random.Generator,
    q_min: np.ndarray,
    q_max: np.ndarray,
    goal_q: np.ndarray,
    goal_sample_rate: float,
) -> np.ndarray:
    if float(rng.random()) < goal_sample_rate:
        return goal_q.copy()
    return rng.uniform(q_min, q_max)


def _nearest(tree: list[np.ndarray], q: np.ndarray) -> int:
    d = [np.linalg.norm(n - q) for n in tree]
    return int(np.argmin(d))


def _extract_path(
    tree_a: list[np.ndarray],
    parent_a: list[int],
    idx_a: int,
    tree_b: list[np.ndarray],
    parent_b: list[int],
    idx_b: int,
) -> list[np.ndarray]:
    pa = []
    i = idx_a
    while i != -1:
        pa.append(tree_a[i])
        i = parent_a[i]
    pa.reverse()

    pb = []
    j = idx_b
    while j != -1:
        pb.append(tree_b[j])
        j = parent_b[j]
    return pa + pb


def rrt_connect_plan(
    q_start: np.ndarray,
    q_goal: np.ndarray,
    q_min: np.ndarray,
    q_max: np.ndarray,
    cfg: PlannerConfig,
    rng: np.random.Generator,
    collision_checker: LeftArmBodyCollisionChecker | None = None,
) -> list[np.ndarray] | None:
    if collision_checker is not None:
        if not collision_checker.is_valid(q_start) or not collision_checker.is_valid(q_goal):
            return None

    if _distance(q_start, q_goal) < cfg.connect_threshold:
        if collision_checker is not None and not collision_checker.segment_valid_by_step(
            q_start,
            q_goal,
            max_step=cfg.collision_max_step,
            min_samples=cfg.collision_samples,
            max_samples=cfg.collision_max_samples,
        ):
            return None
        return [q_start.copy(), q_goal.copy()]

    tree_a = [q_start.copy()]
    parent_a = [-1]
    tree_b = [q_goal.copy()]
    parent_b = [-1]

    for _ in range(cfg.max_iters):
        q_rand = _sample_q(rng, q_min, q_max, q_goal, cfg.goal_sample_rate)

        idx_near_a = _nearest(tree_a, q_rand)
        q_new_a = _steer(tree_a[idx_near_a], q_rand, cfg.step_size)
        if not _within_limits(q_new_a, q_min, q_max):
            continue
        if collision_checker is not None and not collision_checker.segment_valid_by_step(
            tree_a[idx_near_a],
            q_new_a,
            max_step=cfg.collision_max_step,
            min_samples=cfg.collision_samples,
            max_samples=cfg.collision_max_samples,
        ):
            continue
        tree_a.append(q_new_a)
        parent_a.append(idx_near_a)
        idx_new_a = len(tree_a) - 1

        idx_near_b = _nearest(tree_b, q_new_a)
        q_cur = tree_b[idx_near_b].copy()
        parent_idx = idx_near_b
        connected = False

        while True:
            q_next = _steer(q_cur, q_new_a, cfg.step_size)
            if not _within_limits(q_next, q_min, q_max):
                break
            if collision_checker is not None and not collision_checker.segment_valid_by_step(
                q_cur,
                q_next,
                max_step=cfg.collision_max_step,
                min_samples=cfg.collision_samples,
                max_samples=cfg.collision_max_samples,
            ):
                break
            tree_b.append(q_next)
            parent_b.append(parent_idx)
            parent_idx = len(tree_b) - 1
            q_cur = q_next
            if _distance(q_cur, q_new_a) < cfg.connect_threshold:
                connected = True
                break
            if np.allclose(q_cur, q_new_a):
                break

        if connected:
            return _extract_path(tree_a, parent_a, idx_new_a, tree_b, parent_b, parent_idx)

        tree_a, tree_b = tree_b, tree_a
        parent_a, parent_b = parent_b, parent_a

    return None


def shortcut_smooth(
    path: list[np.ndarray],
    q_min: np.ndarray,
    q_max: np.ndarray,
    trials: int,
    rng: np.random.Generator,
    collision_checker: LeftArmBodyCollisionChecker | None = None,
    collision_samples: int = 8,
    collision_max_step: float = 0.06,
    collision_max_samples: int = 256,
) -> list[np.ndarray]:
    if len(path) < 3:
        return path
    p = path[:]
    for _ in range(trials):
        if len(p) < 3:
            break
        i = int(rng.integers(0, len(p) - 2))
        j = int(rng.integers(i + 2, len(p)))
        qi, qj = p[i], p[j]
        if not _within_limits(qi, q_min, q_max) or not _within_limits(qj, q_min, q_max):
            continue
        if collision_checker is not None and not collision_checker.segment_valid_by_step(
            qi,
            qj,
            max_step=collision_max_step,
            min_samples=collision_samples,
            max_samples=collision_max_samples,
        ):
            continue
        p = p[: i + 1] + p[j:]
    return p


def densify_path(path: list[np.ndarray], max_step: float = 0.03) -> list[np.ndarray]:
    dense = [path[0]]
    for i in range(1, len(path)):
        a = path[i - 1]
        b = path[i]
        d = np.linalg.norm(b - a)
        n = max(1, int(np.ceil(d / max_step)))
        for k in range(1, n + 1):
            dense.append(a + (b - a) * (k / n))
    return dense


def start_ui(shared: SharedState) -> None:
    def run() -> None:
        root = tk.Tk()
        root.title("Left Arm Path Planner")
        root.geometry("560x280")

        labels = ["Goal X (m)", "Goal Y (m)", "Goal Z (m)"]
        mins = [-0.9, -0.9, 0.0]
        maxs = [0.9, 0.9, 1.5]
        goal0 = shared.goal_xyz.copy()

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
                length=380,
                command=lambda v, axis=i: shared.set_goal_axis(axis, float(v)),
            )
            s.set(float(goal0[i]))
            s.pack(side="left", fill="x", expand=True)

        status_var = tk.StringVar(value="status: idle")

        def on_plan() -> None:
            with shared._lock:
                shared.request_plan = True
            status_var.set("status: planning...")

        def on_execute() -> None:
            with shared._lock:
                shared.request_execute = True
            status_var.set("status: executing...")

        btn_frame = tk.Frame(root)
        btn_frame.pack(fill="x", padx=8, pady=8)
        tk.Button(btn_frame, text="Plan", width=12, command=on_plan).pack(side="left", padx=4)
        tk.Button(btn_frame, text="Execute", width=12, command=on_execute).pack(side="left", padx=4)
        tk.Label(btn_frame, textvariable=status_var, anchor="w").pack(side="left", padx=10)

        def poll_status() -> None:
            _, _, _, _, st = shared.snapshot()
            status_var.set(f"status: {st}")
            root.after(250, poll_status)

        poll_status()
        root.mainloop()

    threading.Thread(target=run, daemon=True).start()


def main() -> None:
    model_path = os.environ.get("LENS_DUAL_ARM_MJCF", DEFAULT_MODEL_PATH)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model not found: {model_path}")

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    ik_cfg = IkConfig()
    planner_cfg = PlannerConfig()
    rng = np.random.default_rng(20260409)
    bridge_protocol = os.environ.get("LENS_BRIDGE_PROTOCOL", "can").strip().lower()
    can_ids_env = os.environ.get("LENS_LEFT_CAN_IDS", "1,2,3,4,5,6,7")
    left_can_ids = parse_int_list_csv(can_ids_env, 7)
    bridge_print_hz = float(os.environ.get("LENS_BRIDGE_PRINT_HZ", os.environ.get("LENS_CAN_PRINT_HZ", "30")))
    mit_ranges = MitRanges()
    mit_kp = float(os.environ.get("LENS_MIT_KP", "80"))
    mit_kd = float(os.environ.get("LENS_MIT_KD", "2"))
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
            os.path.join(THIS_DIR, "..", "actuator_SDK", "actuator_SDK", "actuator_config.json"),
        ),
        actuator_params_path=os.environ.get(
            "LENS_ACTUATOR_PARAMS",
            os.path.join(THIS_DIR, "..", "actuator_SDK", "actuator_SDK", "actuator_params_config.json"),
        ),
        ethercat_send_frequency=int(os.environ.get("LENS_ETHERCAT_SEND_FREQUENCY", "500")),
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

    shared = SharedState(goal_init)
    start_ui(shared)

    trajectory: list[np.ndarray] = []
    traj_ptr = 0
    last_print = 0.0

    print("路径规划已启动：先在 UI 点击 Plan，再点击 Execute。")

    target_fps = 60.0
    frame_dt = 1.0 / target_fps
    steps_per_frame = max(1, int(round(frame_dt / model.opt.timestep)))

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            tick = time.time()

            if shared.consume_plan_request():
                goal_xyz, _, _, _, _ = shared.snapshot()
                data.qpos[q_idx] = q_cur
                mujoco.mj_fwdPosition(model, data)
                collision_checker.set_qpos_template(data.qpos.copy())
                hold_quat = data.xquat[ee_id].copy()
                q_goal, ik_err_p, ik_err_r, cond = solve_left_ik_multi(
                    model=model,
                    data=data,
                    q_idx=q_idx,
                    ee_body_id=ee_id,
                    q_prev=q_cur,
                    target_pos=goal_xyz,
                    target_quat=hold_quat,
                    q_min=q_min,
                    q_max=q_max,
                    cfg=ik_cfg,
                )
                if ik_err_p > 0.03 or ik_err_r > 0.05:
                    print(
                        f"\n[IK 失败] pos={ik_err_p:.4f} m, rot={ik_err_r:.4f} rad | "
                        f"目标 XYZ: {np.array2string(goal_xyz, precision=4)}\n",
                        flush=True,
                    )
                    shared.set_plan_result(
                        False,
                        f"IK fail (pos={ik_err_p:.3f}m, rot={ik_err_r:.3f}rad)",
                    )
                else:
                    raw = rrt_connect_plan(
                        q_cur,
                        q_goal,
                        q_min,
                        q_max,
                        planner_cfg,
                        rng,
                        collision_checker=collision_checker,
                    )
                    if raw is None:
                        print_joint_space_plan_failure(
                            goal_xyz,
                            float(ik_err_p),
                            q_cur,
                            q_goal,
                            q_min,
                            q_max,
                            collision_checker,
                            max_iters=planner_cfg.max_iters,
                            step_size=planner_cfg.step_size,
                            connect_threshold=planner_cfg.connect_threshold,
                            clearance_min=planner_cfg.clearance_min,
                            collision_samples=planner_cfg.collision_samples,
                            straight_max_step=planner_cfg.step_size,
                            ik_already_passed=True,
                        )
                        shared.set_plan_result(False, "RRT fail (limits/collision), see terminal")
                    else:
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
                        shared.set_plan_result(True, f"planned {len(trajectory)} points, cond={cond:.1f}")

            if shared.consume_execute_request():
                if not trajectory:
                    shared.set_plan_result(False, "no trajectory, plan first")
                else:
                    shared.set_plan_result(True, "executing")

            if trajectory and traj_ptr < len(trajectory):
                q_cmd = trajectory[traj_ptr]
                data.ctrl[act_ids] = q_cmd
                traj_ptr += 1
                if traj_ptr >= len(trajectory):
                    q_cur = trajectory[-1].copy()
                    shared.set_plan_result(True, "done")
            else:
                data.ctrl[act_ids] = q_cur

            # 实时输出「真实协议控制下发」（CAN 或 EtherCAT）
            bridge.maybe_emit(time.time(), data.ctrl[act_ids].copy())

            for _ in range(steps_per_frame):
                mujoco.mj_step(model, data)

            q_cur = data.qpos[q_idx].copy()
            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                ee = data.xpos[ee_id].copy()
                print(
                    f"\nEE xyz: {np.array2string(ee, precision=4)}"
                    f"\nq(deg): {np.array2string(np.rad2deg(q_cur), precision=1)}"
                    f"\ntraj: {traj_ptr}/{len(trajectory) if trajectory else 0}"
                )

            viewer.sync()
            elapsed = time.time() - tick
            time.sleep(max(0.0, frame_dt - elapsed))


if __name__ == "__main__":
    main()

