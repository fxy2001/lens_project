from __future__ import annotations

import os
import time

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
    start_se3_slider_ui as start_left_se3_slider_ui,
)
from right_arm_ik_slider import (
    RIGHT_EE_BODY,
    RIGHT_JOINT_NAMES,
    _right_actuator_ids,
    solve_right_ik_multi,
    start_se3_slider_ui as start_right_se3_slider_ui,
)


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

    left_target = data.xpos[left_ee_id].copy()
    left_rpy = quat_wxyz_to_rpy_mujoco(data.xquat[left_ee_id].copy())
    left_shared = TargetSE3(left_target, left_rpy)
    start_left_se3_slider_ui(left_shared, left_target, left_rpy)

    right_target = data.xpos[right_ee_id].copy()
    right_rpy = quat_wxyz_to_rpy_mujoco(data.xquat[right_ee_id].copy())
    right_shared = TargetSE3(right_target, right_rpy)
    start_right_se3_slider_ui(right_shared, right_target, right_rpy)

    ros_bridge = Ros2JointCommandPublisher(
        Ros2BridgeConfig(
            enabled=os.environ.get("LENS_ROS_BRIDGE", "0").strip().lower() in ("1", "true", "yes", "on"),
            topic=os.environ.get("LENS_ROS_TOPIC", "/joint_command"),
            hz=float(os.environ.get("LENS_ROS_HZ", "60")),
        )
    )

    print("双臂 6D IK 融合控制已启动。")
    print("会同时弹出左右臂两个 6D 目标滑条窗口。")
    print("设置 LENS_ROS_BRIDGE=1 可将 14 关节命令发布到 /joint_command。")

    last_print = 0.0
    target_fps = 60.0
    frame_dt = 1.0 / target_fps
    steps_per_frame = max(1, int(round(frame_dt / model.opt.timestep)))

    all_joint_names = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES

    with mujoco.viewer.launch_passive(model, data) as viewer:
        try:
            while viewer.is_running():
                tick = time.time()

                left_target_pos, left_target_quat = left_shared.get_pose()
                right_target_pos, right_target_quat = right_shared.get_pose()

                left_q_sol, left_err_p, left_err_r, left_cond = solve_left_ik_multi(
                    model=model,
                    data=data,
                    q_idx=left_q_idx,
                    ee_body_id=left_ee_id,
                    q_prev=left_q_cur,
                    target_pos=left_target_pos,
                    target_quat=left_target_quat,
                    q_min=left_q_min,
                    q_max=left_q_max,
                    cfg=cfg,
                )
                right_q_sol, right_err_p, right_err_r, right_cond = solve_right_ik_multi(
                    model=model,
                    data=data,
                    q_idx=right_q_idx,
                    ee_body_id=right_ee_id,
                    q_prev=right_q_cur,
                    target_pos=right_target_pos,
                    target_quat=right_target_quat,
                    q_min=right_q_min,
                    q_max=right_q_max,
                    cfg=cfg,
                )

                left_q_cur = left_q_sol
                right_q_cur = right_q_sol

                data.ctrl[left_act_ids] = left_q_cur
                data.ctrl[right_act_ids] = right_q_cur

                both_q = np.concatenate([left_q_cur, right_q_cur])
                ros_bridge.maybe_publish(tick, all_joint_names, both_q)

                for _ in range(steps_per_frame):
                    mujoco.mj_step(model, data)

                now = time.time()
                if now - last_print > 0.5:
                    last_print = now
                    print(
                        f"[DualIK] L(pos={left_err_p:.4f}m rot={np.rad2deg(left_err_r):.2f}deg cond={left_cond:.1f}) "
                        f"R(pos={right_err_p:.4f}m rot={np.rad2deg(right_err_r):.2f}deg cond={right_cond:.1f})"
                    )

                viewer.sync()
                elapsed = time.time() - tick
                time.sleep(max(0.0, frame_dt - elapsed))
        finally:
            ros_bridge.close()


if __name__ == "__main__":
    main()

