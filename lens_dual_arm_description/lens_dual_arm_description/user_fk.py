from __future__ import annotations

from typing import Tuple

import numpy as np


def fk_left(q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    你需要实现：左臂 7DoF 的正运动学（FK）。

    Args:
        q: (7,) 关节角（弧度），顺序必须和 MJCF 一致：
           [
             Left_Shoulder_Pitch_Joint,
             Left_Shoulder_Roll_Joint,
             Left_Shoulder_Yaw_Joint,
             Left_Elbow_Pitch_Joint,
             Left_Wrist_Yaw_Joint,
             Left_Wrist_Roll_Joint,
             Left_Wrist_Pitch_Joint,
           ]

    Returns:
        pos: (3,) 末端在 world 坐标系的位置 xyz
        quat: (4,) 末端在 world 坐标系的四元数 (w, x, y, z)
    """
    raise NotImplementedError("请在 user_fk.py 中实现 fk_left(q)")


def fk_right(q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    右臂 FK（关节顺序同理）：
           [
             Right_Shoulder_Pitch_Joint,
             Right_Shoulder_Roll_Joint,
             Right_Shoulder_Yaw_Joint,
             Right_Elbow_Pitch_Joint,
             Right_Wrist_Yaw_Joint,
             Right_Wrist_Roll_Joint,
             Right_Wrist_Pitch_Joint,
           ]
    """
    raise NotImplementedError("请在 user_fk.py 中实现 fk_right(q)")

