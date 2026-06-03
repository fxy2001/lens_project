"""
Joint position limits for planning, validation, and /joint_command checks.

Convention: same as MuJoCo MJCF, /joint_states feedback, and /joint_command
(execution frame). URDF numeric values differ on some joints due to axis sign.
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence

# mj_lens_dual_arm.xml ranges (radians), execution / joint_states convention.
JOINT_LIMITS_RAD: dict[str, tuple[float, float]] = {
    "Left_Shoulder_Pitch_Joint": (-3.14, 3.14),
    "Left_Shoulder_Roll_Joint": (-0.3, 3.2),
    "Left_Shoulder_Yaw_Joint": (-3.14, 3.14),
    "Left_Elbow_Pitch_Joint": (-1.8, 0.2),
    "Left_Wrist_Yaw_Joint": (-3.14, 3.14),
    "Left_Wrist_Roll_Joint": (-1.57, 1.57),
    "Left_Wrist_Pitch_Joint": (-1.57, 1.57),
    "Right_Shoulder_Pitch_Joint": (-3.14, 3.14),
    "Right_Shoulder_Roll_Joint": (-3.2, 0.3),
    "Right_Shoulder_Yaw_Joint": (-3.14, 3.14),
    "Right_Elbow_Pitch_Joint": (-1.8, 0.2),
    "Right_Wrist_Yaw_Joint": (-3.14, 3.14),
    "Right_Wrist_Roll_Joint": (-1.57, 1.57),
    "Right_Wrist_Pitch_Joint": (-1.57, 1.57),
}

ALL_ARM_JOINT_NAMES: tuple[str, ...] = tuple(JOINT_LIMITS_RAD.keys())


class JointLimitError(ValueError):
    """Raised when a joint angle is outside the allowed range."""

    code = "JOINT_LIMIT_VIOLATION"


def limits_for_joint_names(joint_names: Sequence[str]) -> tuple[list[float], list[float]]:
    lo: list[float] = []
    hi: list[float] = []
    for name in joint_names:
        if name not in JOINT_LIMITS_RAD:
            raise JointLimitError(f"JOINT_LIMIT_VIOLATION: unknown joint {name!r}")
        a, b = JOINT_LIMITS_RAD[name]
        lo.append(float(a))
        hi.append(float(b))
    return lo, hi


def _is_finite(x: float) -> bool:
    return math.isfinite(x)


def validate_joint_positions(
    joint_names: Sequence[str],
    positions: Sequence[float],
    *,
    frame_index: int | None = None,
    margin_rad: float = 0.0,
) -> None:
    if len(joint_names) != len(positions):
        raise JointLimitError(
            f"JOINT_LIMIT_VIOLATION: name/position length mismatch "
            f"({len(joint_names)} vs {len(positions)})"
        )

    prefix = ""
    if frame_index is not None:
        prefix = f"frame={frame_index}: "

    for name, raw_q in zip(joint_names, positions):
        if name not in JOINT_LIMITS_RAD:
            raise JointLimitError(
                f"{prefix}JOINT_LIMIT_VIOLATION: unknown joint {name!r}"
            )
        q = float(raw_q)
        if not _is_finite(q):
            raise JointLimitError(
                f"{prefix}JOINT_LIMIT_VIOLATION: {name} non-finite value {raw_q!r}"
            )
        lo, hi = JOINT_LIMITS_RAD[name]
        lo_eff = lo + margin_rad
        hi_eff = hi - margin_rad
        if hi_eff < lo_eff:
            lo_eff, hi_eff = lo, hi
        if q < lo_eff - 1e-9 or q > hi_eff + 1e-9:
            raise JointLimitError(
                f"{prefix}JOINT_LIMIT_VIOLATION: {name}={q:+.4f} rad "
                f"({math.degrees(q):+.2f} deg) outside "
                f"[{lo:+.4f}, {hi:+.4f}] rad "
                f"({math.degrees(lo):+.1f}, {math.degrees(hi):+.1f} deg)"
            )


def validate_joint_trajectory(
    joint_names: Sequence[str],
    q_refs: Iterable[Sequence[float]],
    *,
    margin_rad: float = 0.0,
) -> None:
    for i, q in enumerate(q_refs):
        validate_joint_positions(joint_names, q, frame_index=i, margin_rad=margin_rad)
