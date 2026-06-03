#!/usr/bin/env bash
# Path IR → 真机：目标 1s 内「命令受理 → 真机动作完成」（规划+执行总墙钟）
#
# 终端 A: sudo -E scripts/run_joint_controller.sh
# 终端 B:
#   scripts/run_llm_trajectory_fast.sh          # 首次 ~0.5–0.9s（含 IK）
#   scripts/run_llm_trajectory_fast.sh          # 同姿态重复 → LiftCache 命中，更快
#
# 姿态要求高时: export LENS_MAX_TOTAL_S=3 LENS_IK_MAX_ITERS=50
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="${ROOT}/lens_dual_arm_description"
IR_FILE="${LENS_PATH_IR_FILE:-${SIM_DIR}/llm_demo/ir.json}"
if [[ "${IR_FILE}" != /* ]]; then
  IR_FILE="${ROOT}/${IR_FILE#./}"
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${ROOT}/lens_dual_arm/install/setup.bash" ]]; then
  source "${ROOT}/lens_dual_arm/install/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# --- 1s 端到端预算（勿用 0.55s ros_reserve，会把 motion 压没）---
export LENS_SKIP_DRAW_RANGE_FIT=1
export LENS_FLANGE_PERP_XY=1
export LENS_FLANGE_XY_NORMAL="${LENS_FLANGE_XY_NORMAL:-down}"
export LENS_ENFORCE_TOOL_DOWN="${LENS_ENFORCE_TOOL_DOWN:-1}"
export LENS_FLANGE_PERP_XZ=0
export LENS_FLANGE_PERP_YZ=0
export LENS_MAX_TOTAL_S="${LENS_MAX_TOTAL_S:-1}"
export LENS_FAST_EXECUTE=1
export LENS_AUTO_EXECUTE=1
export LENS_PLAN_CACHE=1
export LENS_LIFT_PLAN_CACHE=1
export LENS_PLAN_MAX_POINTS="${LENS_PLAN_MAX_POINTS:-3}"
export LENS_CARTESIAN_LIFT_POINTS="${LENS_CARTESIAN_LIFT_POINTS:-3}"
export LENS_IK_MAX_ITERS="${LENS_IK_MAX_ITERS:-35}"
export LENS_IK_ROT_TOL="${LENS_IK_ROT_TOL:-0.005}"
export LENS_IK_ROT_WEIGHT="${LENS_IK_ROT_WEIGHT:-10}"
export LENS_IK_ORIENTATION_REFINE_ITERS="${LENS_IK_ORIENTATION_REFINE_ITERS:-18}"
export LENS_IK_WRIST_NULLSPACE_SCALE="${LENS_IK_WRIST_NULLSPACE_SCALE:-0}"
export LENS_IK_MULTI_WHEN_LOCKED="${LENS_IK_MULTI_WHEN_LOCKED:-1}"
export LENS_SETTLE_IK_ITERS="${LENS_SETTLE_IK_ITERS:-40}"
export LENS_SETTLE_ROT_TOL="${LENS_SETTLE_ROT_TOL:-0.002}"
export LENS_SETTLE_Z_MIN="${LENS_SETTLE_Z_MIN:-0.88}"
export LENS_IK_WARM_START=1
export LENS_IK_START_AT_HOME=0
export LENS_SKIP_IK_GUARD=1
export LENS_SETTLE_ORIENTATION=1
export LENS_ENFORCE_JOINT_LIMITS="${LENS_ENFORCE_JOINT_LIMITS:-1}"

export LENS_EXEC_ROS_RESERVE_S="${LENS_EXEC_ROS_RESERVE_S:-0.08}"
export LENS_PLAN_RESERVE_S="${LENS_PLAN_RESERVE_S:-0.42}"
export LENS_EXEC_HOLD_S="${LENS_EXEC_HOLD_S:-0.02}"
export LENS_MIN_MOTION_S="${LENS_MIN_MOTION_S:-0.35}"
export LENS_MISC_RESERVE_S="${LENS_MISC_RESERVE_S:-0.02}"
export LENS_EXEC_KEEPALIVE_HZ="${LENS_EXEC_KEEPALIVE_HZ:-30}"

export LENS_ROS_BRIDGE=1
export LENS_ROS_TOPIC="${LENS_ROS_TOPIC:-/joint_command}"

export LENS_PATH_IR_FILE="${IR_FILE}"

cd "${SIM_DIR}"
echo "[run] IR=${IR_FILE} 目标: 命令→真机完成 ≤ ${LENS_MAX_TOTAL_S}s"
echo "[run] 首次 IK 后同姿态会 LiftCache；改姿态请: rm -rf ~/.lens_path_ir_plan_cache/"
exec python3 llm_demo/dual_arm_llm_trajectory_demo.py
