"""
左臂关节空间规划用的「身体」避障：基于 MuJoCo mj_geomDistance 的符号距离，
检测左臂碰撞组几何与躯干(root)、右臂及左臂非相邻连杆之间是否过近。

说明：
- 模型中碰撞网格为 geom group=0（绿色 visual 为 group=1），地面 world 不参与身体避障。
- clearance_min 为允许的最小符号距离（略负可容忍网格/网格离散误差）；更小（更深穿透）视为无效。
"""
from __future__ import annotations

import mujoco
import numpy as np


LEFT_ARM_BODY_CHAIN: tuple[str, ...] = (
    "Left_Shoulder_Pitch_Link",
    "Left_Shoulder_Roll_Link",
    "Left_Shoulder_Yaw_Link",
    "Left_Elbow_Pitch_Link",
    "Left_Wrist_Yaw_Link",
    "Left_Wrist_Roll_Link",
    "Left_Wrist_Pitch_Link",
)


def _body_name(model: mujoco.MjModel, body_id: int) -> str:
    n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    return n or ""


def _left_arm_chain_index(body_name: str) -> int:
    try:
        return LEFT_ARM_BODY_CHAIN.index(body_name)
    except ValueError:
        return -1


class LeftArmBodyCollisionChecker:
    """
    在固定整臂 qpos 模板下，仅改变左臂 7 关节角，用几何距离做无碰撞（含裕度）检验。
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        left_qpos_idx: np.ndarray,
        clearance_min: float = -0.012,
    ) -> None:
        self.model = model
        self.left_qpos_idx = np.asarray(left_qpos_idx, dtype=int)
        self.clearance_min = float(clearance_min)
        self._scratch = mujoco.MjData(model)
        self._pairs: list[tuple[int, int]] = []
        self._build_pairs()

    def _build_pairs(self) -> None:
        left_geoms: list[tuple[int, int]] = []  # (geom_id, chain_idx)
        obstacle_geoms: list[int] = []

        for g in range(self.model.ngeom):
            if int(self.model.geom_group[g]) != 0:
                continue
            bid = int(self.model.geom_bodyid[g])
            bname = _body_name(self.model, bid)
            if bname == "world":
                continue
            if bname.startswith("Left_"):
                ci = _left_arm_chain_index(bname)
                if ci >= 0:
                    left_geoms.append((g, ci))
            elif bname == "root" or bname.startswith("Right_"):
                obstacle_geoms.append(g)

        pairs: list[tuple[int, int]] = []
        # 左臂 vs 躯干 / 右臂
        for g1, _ in left_geoms:
            for g2 in obstacle_geoms:
                if g1 != g2:
                    pairs.append((g1, g2))
        # 左臂非相邻连杆互检（|i-j|>=2）
        for i, (g1, c1) in enumerate(left_geoms):
            for g2, c2 in left_geoms[i + 1 :]:
                if abs(c1 - c2) >= 2:
                    pairs.append((g1, g2))

        # 去重 (unordered)
        seen: set[tuple[int, int]] = set()
        uniq: list[tuple[int, int]] = []
        for a, b in pairs:
            key = (a, b) if a < b else (b, a)
            if key in seen:
                continue
            seen.add(key)
            uniq.append((a, b))
        self._pairs = uniq

    def set_qpos_template(self, qpos_full: np.ndarray) -> None:
        self._scratch.qpos[:] = np.asarray(qpos_full, dtype=float)

    def is_valid(self, q_left: np.ndarray) -> bool:
        self._scratch.qpos[self.left_qpos_idx] = np.asarray(q_left, dtype=float).reshape(-1)
        mujoco.mj_forward(self.model, self._scratch)
        dmax = 50.0
        thr = self.clearance_min
        for g1, g2 in self._pairs:
            dist = mujoco.mj_geomDistance(self.model, self._scratch, g1, g2, dmax, None)
            if float(dist) < thr:
                return False
        return True

    def segment_valid(self, q_a: np.ndarray, q_b: np.ndarray, n_samples: int) -> bool:
        if n_samples < 2:
            return self.is_valid(q_b)
        for s in range(n_samples + 1):
            t = s / float(n_samples)
            q = q_a + (q_b - q_a) * t
            if not self.is_valid(q):
                return False
        return True

    def segment_valid_by_step(
        self,
        q_a: np.ndarray,
        q_b: np.ndarray,
        *,
        max_step: float,
        min_samples: int = 2,
        max_samples: int = 256,
    ) -> bool:
        """
        按关节空间距离自适应采样：让相邻采样点间的 ||dq|| ≤ max_step。
        适用于大幅关节变化时避免“采样太稀穿模”。
        """
        d = float(np.linalg.norm(q_b - q_a))
        if d < 1e-12:
            return self.is_valid(q_a)
        n = int(np.ceil(d / max(1e-9, float(max_step))))
        n = max(int(min_samples), min(int(max_samples), n))
        return self.segment_valid(q_a, q_b, n)

    def min_signed_distance_detail(self, q_left: np.ndarray) -> tuple[float, int, int]:
        """
        当前左臂关节角下，所有检测对中的最小符号距离及对应 (geom_id, geom_id)。
        用于调试：若最小值 < clearance_min 则 is_valid 为 False。
        """
        self._scratch.qpos[self.left_qpos_idx] = np.asarray(q_left, dtype=float).reshape(-1)
        mujoco.mj_forward(self.model, self._scratch)
        dmax = 50.0
        best = float("inf")
        pair = (-1, -1)
        for g1, g2 in self._pairs:
            dist = float(mujoco.mj_geomDistance(self.model, self._scratch, g1, g2, dmax, None))
            if dist < best:
                best = dist
                pair = (g1, g2)
        if not self._pairs:
            return float("nan"), -1, -1
        return best, pair[0], pair[1]

    def format_pose_debug(self, q_left: np.ndarray, label: str = "") -> str:
        """一行可读诊断（终端打印）。"""
        dmin, g1, g2 = self.min_signed_distance_detail(q_left)
        thr = self.clearance_min
        ok = bool(np.isfinite(dmin) and dmin >= thr)
        head = f"{label} " if label else ""
        if g1 < 0:
            return f"{head}无检测几何对（模型异常）"
        gn1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g1) or f"geom{g1}"
        gn2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g2) or f"geom{g2}"
        stat = "OK" if ok else "碰撞裕度不足"
        return (
            f"{head}min_dist={dmin:.5f}m (阈>={thr:.5f}) {stat} | 最紧对: {gn1} <-> {gn2}"
        )

    def first_invalid_on_segment(
        self,
        q_a: np.ndarray,
        q_b: np.ndarray,
        n_samples: int,
    ) -> tuple[float, np.ndarray] | None:
        """
        若线段上存在无效姿态，返回 (参数 t∈[0,1], q_at_t)；否则 None。
        """
        if n_samples < 2:
            if not self.is_valid(q_b):
                return 1.0, np.asarray(q_b, dtype=float)
            return None
        for s in range(n_samples + 1):
            t = s / float(n_samples)
            q = q_a + (q_b - q_a) * t
            if not self.is_valid(q):
                return t, q.copy()
        return None

    def first_invalid_on_segment_by_step(
        self,
        q_a: np.ndarray,
        q_b: np.ndarray,
        *,
        max_step: float,
        min_samples: int = 2,
        max_samples: int = 256,
    ) -> tuple[float, np.ndarray] | None:
        d = float(np.linalg.norm(q_b - q_a))
        if d < 1e-12:
            if not self.is_valid(q_a):
                return 0.0, np.asarray(q_a, dtype=float)
            return None
        n = int(np.ceil(d / max(1e-9, float(max_step))))
        n = max(int(min_samples), min(int(max_samples), n))
        return self.first_invalid_on_segment(q_a, q_b, n)


def print_joint_space_plan_failure(
    goal_xyz: np.ndarray,
    ik_ep: float,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    q_min: np.ndarray,
    q_max: np.ndarray,
    collision_checker: LeftArmBodyCollisionChecker,
    *,
    max_iters: int,
    step_size: float,
    connect_threshold: float,
    clearance_min: float,
    collision_samples: int,
    straight_max_step: float = 0.12,
    ik_ep_label: str = "IK 位置残差 (m)",
    ik_success_threshold: float = 0.08,
    ik_already_passed: bool = False,
) -> None:
    """
    RRT 与关节直线回退均失败时，在终端打印诊断（起点/目标 min_dist、直线段冲突位置、RRT 参数建议）。
    ik_already_passed=True 时不打印 IK 阈值说明（用于 6D IK 已过关仅 RRT 失败的情形）。
    """
    print("\n========== 规划失败诊断 ==========", flush=True)
    print(f"目标 XYZ (m): {np.array2string(np.asarray(goal_xyz), precision=4)}", flush=True)
    if ik_already_passed:
        print(
            "IK 已满足脚本阈值；下列为关节空间（身体避障 / 限位 / RRT）失败分析。",
            flush=True,
        )
    else:
        print(
            f"{ik_ep_label}: {ik_ep:.5f}  (≤{ik_success_threshold} 视为 IK 成功)",
            flush=True,
        )
    print("[起点] " + collision_checker.format_pose_debug(q_start), flush=True)
    print("[目标关节] " + collision_checker.format_pose_debug(q_goal), flush=True)

    qs_ok = collision_checker.is_valid(q_start)
    qg_ok = collision_checker.is_valid(q_goal)

    if not qs_ok:
        print(
            "原因摘要: 起点姿态已不满足身体避障（min_dist < clearance_min）。"
            "请先在仿真中移到更开阔的构型。",
            flush=True,
        )
    elif not qg_ok:
        print(
            "原因摘要: IK 得到的目标关节不满足身体避障。"
            "请调整目标或先移动手臂。",
            flush=True,
        )
    else:
        d_q = float(np.linalg.norm(q_goal - q_start))
        print(
            f"起、终点避障均通过；||q_goal - q_start|| = {d_q:.4f} rad（关节空间）",
            flush=True,
        )
        n_lin = max(1, int(np.ceil(d_q / straight_max_step)))
        limit_fail_t: float | None = None
        for k in range(n_lin + 1):
            t = k / float(n_lin)
            q = q_start + (q_goal - q_start) * t
            if not (np.all(q >= q_min) and np.all(q <= q_max)):
                limit_fail_t = t
                break
        if limit_fail_t is not None:
            print(
                f"关节直线插值在 t≈{limit_fail_t:.3f} 处超出关节限位。",
                flush=True,
            )
        else:
            fine_step = max(1e-6, float(straight_max_step) * 0.5)
            fi = collision_checker.first_invalid_on_segment_by_step(
                q_start, q_goal, max_step=fine_step, min_samples=24, max_samples=256
            )
            if fi is not None:
                t_bad, q_bad = fi
                print(
                    f"关节直线插值在 t≈{t_bad:.3f} 处与身体几何冲突:",
                    flush=True,
                )
                print("  " + collision_checker.format_pose_debug(q_bad), flush=True)
            else:
                print(
                    "直线插值细采样未检出冲突（失败更可能来自 RRT 迭代/随机性）。",
                    flush=True,
                )

        print(
            f"RRT 参数: max_iters={max_iters}, step_size={step_size}, "
            f"connect_threshold={connect_threshold}, "
            f"clearance_min={clearance_min}, collision_samples={collision_samples}",
            flush=True,
        )
        print(
            "可尝试: 增大 max_iters 或 connect_threshold；略放宽 clearance_min（如 -0.018）；"
            "或缩小目标与当前构型的距离。",
            flush=True,
        )
    print("====================================\n", flush=True)
