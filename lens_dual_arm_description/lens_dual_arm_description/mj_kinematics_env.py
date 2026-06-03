import os
from dataclasses import dataclass
from typing import Literal, Tuple

import mujoco
import numpy as np


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL_PATH = os.path.join(THIS_DIR, "mjcf", "mj_lens_dual_arm.xml")


ArmSide = Literal["left", "right"]


@dataclass
class KinematicsEnvConfig:
    model_path: str = DEFAULT_MODEL_PATH
    """Path to the MuJoCo model file (MJCF recommended)."""


class DualArmKinematicsEnv:
    """
    Minimal MuJoCo environment for forward/inverse kinematics verification.

    Usage:
        env = DualArmKinematicsEnv()
        q_left = np.zeros(7)
        pose = env.forward_kinematics("left", q_left)
    """

    def __init__(self, config: KinematicsEnvConfig | None = None) -> None:
        self.config = config or KinematicsEnvConfig()
        if not os.path.exists(self.config.model_path):
            raise FileNotFoundError(f"Model not found: {self.config.model_path}")

        try:
            self.model = mujoco.MjModel.from_xml_path(self.config.model_path)
        except ValueError as exc:
            raise ValueError(
                f"Failed to load model: {self.config.model_path}\n"
                "建议优先使用 MJCF 文件（例如 mjcf/mj_lens_dual_arm.xml）。\n"
                "如果使用 URDF，需确保 mesh 路径是 MuJoCo 可直接访问的相对/绝对路径。"
            ) from exc
        self.data = mujoco.MjData(self.model)

        # joint order for each arm (7 DoF per arm)
        self.left_joint_names = [
            "Left_Shoulder_Pitch_Joint",
            "Left_Shoulder_Roll_Joint",
            "Left_Shoulder_Yaw_Joint",
            "Left_Elbow_Pitch_Joint",
            "Left_Wrist_Yaw_Joint",
            "Left_Wrist_Roll_Joint",
            "Left_Wrist_Pitch_Joint",
        ]
        self.right_joint_names = [
            "Right_Shoulder_Pitch_Joint",
            "Right_Shoulder_Roll_Joint",
            "Right_Shoulder_Yaw_Joint",
            "Right_Elbow_Pitch_Joint",
            "Right_Wrist_Yaw_Joint",
            "Right_Wrist_Roll_Joint",
            "Right_Wrist_Pitch_Joint",
        ]

        # end-effector bodies (you can change to sites if you add them)
        self.left_ee_body = "Left_Wrist_Pitch_Link"
        self.right_ee_body = "Right_Wrist_Pitch_Link"

        # cache joint indices
        self._left_qpos_indices = self._get_joint_indices(self.left_joint_names)
        self._right_qpos_indices = self._get_joint_indices(self.right_joint_names)

    # ------------------------------------------------------------------
    # internal helpers
    # ------------------------------------------------------------------
    def _get_joint_indices(self, joint_names: list[str]) -> np.ndarray:
        indices: list[int] = []
        for name in joint_names:
            j_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if j_id < 0:
                raise ValueError(f"Joint not found in model: {name}")
            indices.append(self.model.jnt_qposadr[j_id])
        return np.array(indices, dtype=int)

    def _get_ee_pose(self, side: ArmSide) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return EE pose in world frame.

        Returns:
            position: (3,) xyz
            orientation: (4,) wxyz quaternion
        """
        body_name = self.left_ee_body if side == "left" else self.right_ee_body
        b_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if b_id < 0:
            raise ValueError(f"End-effector body not found: {body_name}")

        pos = self.data.xpos[b_id].copy()
        quat = self.data.xquat[b_id].copy()
        return pos, quat

    def _set_joint_positions(self, side: ArmSide, q: np.ndarray) -> None:
        if q.shape != (7,):
            raise ValueError(f"Expected q shape (7,), got {q.shape}")

        if side == "left":
            indices = self._left_qpos_indices
        else:
            indices = self._right_qpos_indices

        self.data.qpos[indices] = q
        mujoco.mj_fwdPosition(self.model, self.data)

    # ------------------------------------------------------------------
    # public API for FK/IK verification
    # ------------------------------------------------------------------
    def forward_kinematics(
        self, side: ArmSide, q: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Run FK in MuJoCo and return EE pose.

        Args:
            side: "left" or "right"
            q: (7,) joint angles, order defined in self.left/right_joint_names
        """
        self._set_joint_positions(side, q)
        return self._get_ee_pose(side)

    def check_fk_error(
        self,
        side: ArmSide,
        q: np.ndarray,
        target_pos: np.ndarray,
        target_quat: np.ndarray | None = None,
    ) -> dict:
        """
        Compare your FK result with MuJoCo.

        Args:
            side: "left" or "right"
            q: (7,) joint angles
            target_pos: (3,) position from your FK implementation
            target_quat: optional (4,) wxyz quaternion from your FK
        """
        sim_pos, sim_quat = self.forward_kinematics(side, q)

        pos_err = sim_pos - target_pos
        result: dict = {
            "sim_pos": sim_pos,
            "target_pos": target_pos,
            "pos_err": pos_err,
            "pos_err_norm": float(np.linalg.norm(pos_err)),
        }

        if target_quat is not None:
            # relative orientation error via quaternion difference
            # q_err = sim * conj(target)
            t = target_quat / np.linalg.norm(target_quat)
            s = sim_quat / np.linalg.norm(sim_quat)
            q_err = np.array(
                [
                    s[0] * t[0] + s[1] * t[1] + s[2] * t[2] + s[3] * t[3],
                    s[0] * -t[1] + s[1] * t[0] + s[2] * -t[3] - s[3] * -t[2],
                    s[0] * -t[2] - s[1] * -t[3] + s[2] * t[0] + s[3] * -t[1],
                    s[0] * -t[3] + s[1] * -t[2] - s[2] * -t[1] + s[3] * t[0],
                ]
            )
            ang_err = 2 * np.arccos(np.clip(abs(q_err[0]), -1.0, 1.0))
            result.update(
                {
                    "sim_quat": sim_quat,
                    "target_quat": target_quat,
                    "ang_err": float(ang_err),
                }
            )

        return result


def main() -> None:
    """
    Simple demo: random joint angles and FK check for left arm.

    Replace `target_pos` / `target_quat` with the outputs of your own FK
    implementation to compute the error with respect to MuJoCo.
    """
    env = DualArmKinematicsEnv()

    # example joint vector within rough limits
    q_left = np.array([0.0, 0.5, 0.0, -0.5, 0.0, 0.0, 0.0])
    sim_pos, sim_quat = env.forward_kinematics("left", q_left)

    print("Sim EE position (left):", sim_pos)
    print("Sim EE orientation (quat, wxyz):", sim_quat)

    # here you would plug in your FK results
    fk_result = env.check_fk_error("left", q_left, target_pos=sim_pos, target_quat=sim_quat)
    print("FK self-check (should be near zero error):")
    for k, v in fk_result.items():
        print(f"  {k}: {v}")


if __name__ == "__main__":
    main()

