from __future__ import annotations

import os
import time
from typing import Callable, Tuple

import mujoco
import mujoco.viewer
import numpy as np

from mj_kinematics_env import DualArmKinematicsEnv, KinematicsEnvConfig


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL_PATH = os.path.join(THIS_DIR, "mjcf", "mj_lens_dual_arm.xml")


def _try_import_user_fk() -> Tuple[
    Callable[[np.ndarray], Tuple[np.ndarray, np.ndarray]] | None,
    Callable[[np.ndarray], Tuple[np.ndarray, np.ndarray]] | None,
]:
    try:
        from user_fk import fk_left, fk_right  # type: ignore

        return fk_left, fk_right
    except Exception:
        return None, None


def _quat_angle_error(q_sim: np.ndarray, q_user: np.ndarray) -> float:
    q_sim = q_sim / np.linalg.norm(q_sim)
    q_user = q_user / np.linalg.norm(q_user)
    dot = float(np.clip(abs(np.dot(q_sim, q_user)), -1.0, 1.0))
    return float(2.0 * np.arccos(dot))


def main() -> None:
    model_path = os.environ.get("LENS_DUAL_ARM_MJCF", DEFAULT_MODEL_PATH)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model not found: {model_path}")

    # Use the helper env just to get consistent joint indices + EE body names.
    env = DualArmKinematicsEnv(config=KinematicsEnvConfig(model_path=model_path))
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    fk_left, fk_right = _try_import_user_fk()

    left_ee_body = "Left_Wrist_Pitch_Link"
    right_ee_body = "Right_Wrist_Pitch_Link"
    left_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, left_ee_body)
    right_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, right_ee_body)
    if left_ee_id < 0 or right_ee_id < 0:
        raise RuntimeError("End-effector body not found in MJCF (unexpected).")

    # Map joint names -> qpos indices in this loaded model (must match env order).
    def joint_indices(joint_names: list[str]) -> np.ndarray:
        idx: list[int] = []
        for name in joint_names:
            j_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if j_id < 0:
                raise ValueError(f"Joint not found in model: {name}")
            idx.append(int(model.jnt_qposadr[j_id]))
        return np.array(idx, dtype=int)

    left_qpos_idx = joint_indices(env.left_joint_names)
    right_qpos_idx = joint_indices(env.right_joint_names)

    last_print_t = 0.0
    target_fps = 60.0
    frame_dt = 1.0 / target_fps
    # Keep simulation close to realtime: do multiple sim steps per rendered frame.
    steps_per_frame = max(1, int(round(frame_dt / model.opt.timestep)))

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Let the user drive joint targets using the built-in actuator sliders.
        # We just step the simulation and overlay FK markers/errors.
        while viewer.is_running():
            step_start = time.time()
            for _ in range(steps_per_frame):
                mujoco.mj_step(model, data)

            # Clear previous markers
            try:
                viewer.user_scn.ngeom = 0  # type: ignore[attr-defined]
            except Exception:
                pass

            # Read current joint positions (what the sim actually is at)
            q_left = data.qpos[left_qpos_idx].copy()
            q_right = data.qpos[right_qpos_idx].copy()

            # Simulated EE pose
            sim_left_pos = data.xpos[left_ee_id].copy()
            sim_left_quat = data.xquat[left_ee_id].copy()
            sim_right_pos = data.xpos[right_ee_id].copy()
            sim_right_quat = data.xquat[right_ee_id].copy()

            # User FK prediction (if implemented)
            user_left = None
            user_right = None
            if fk_left is not None:
                try:
                    user_left = fk_left(q_left)
                except Exception:
                    user_left = None
            if fk_right is not None:
                try:
                    user_right = fk_right(q_right)
                except Exception:
                    user_right = None

            # Visualize: markers at user-predicted EE positions
            if user_left is not None:
                pos, _quat = user_left
                viewer.add_marker(pos=pos, size=np.array([0.02, 0.02, 0.02]), rgba=np.array([1.0, 0.2, 0.2, 1.0]))
            if user_right is not None:
                pos, _quat = user_right
                viewer.add_marker(pos=pos, size=np.array([0.02, 0.02, 0.02]), rgba=np.array([0.2, 0.4, 1.0, 1.0]))

            # Console output (throttled)
            now = time.time()
            if now - last_print_t > 2.0:
                last_print_t = now

                def fmt_vec(v: np.ndarray) -> str:
                    return np.array2string(v, precision=4, floatmode="fixed")

                print("\n=== FK 验证（仿真 vs 你的 FK）===")
                print("Left q:", fmt_vec(q_left))
                print("  sim pos :", fmt_vec(sim_left_pos))
                if user_left is not None:
                    u_pos, u_quat = user_left
                    pos_err = sim_left_pos - u_pos
                    ang_err = _quat_angle_error(sim_left_quat, u_quat)
                    print("  user pos:", fmt_vec(u_pos))
                    print("  pos err :", fmt_vec(pos_err), " norm=", float(np.linalg.norm(pos_err)))
                    print("  ang err(rad):", ang_err)
                else:
                    print("  user FK: 未加载/未实现/运行报错（请实现 user_fk.fk_left）")

                print("Right q:", fmt_vec(q_right))
                print("  sim pos :", fmt_vec(sim_right_pos))
                if user_right is not None:
                    u_pos, u_quat = user_right
                    pos_err = sim_right_pos - u_pos
                    ang_err = _quat_angle_error(sim_right_quat, u_quat)
                    print("  user pos:", fmt_vec(u_pos))
                    print("  pos err :", fmt_vec(pos_err), " norm=", float(np.linalg.norm(pos_err)))
                    print("  ang err(rad):", ang_err)
                else:
                    print("  user FK: 未加载/未实现/运行报错（请实现 user_fk.fk_right）")

            # Keep realtime-ish pacing
            viewer.sync()
            elapsed = time.time() - step_start
            time.sleep(max(0.0, frame_dt - elapsed))


if __name__ == "__main__":
    main()

