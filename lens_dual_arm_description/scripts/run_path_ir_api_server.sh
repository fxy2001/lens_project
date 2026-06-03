#!/usr/bin/env bash
# Headless HTTP API for external Path IR input (no MuJoCo viewer).
#
# Vision colleague (minimal JSON, auto-convert to Path IR):
#   curl -X POST http://<robot-ip>:8000/vision/trajectory \
#     -H 'Content-Type: application/json' \
#     -d @lens_dual_arm_description/llm_demo/vision_simple_example.json
#
# Full Path IR:
#   curl -X POST http://<robot-ip>:8000/trajectory \
#     -H 'Content-Type: application/json' \
#     -d @lens_dual_arm_description/llm_demo/ir.json
#
# Terminal A (real robot driver, keep running):
#   sudo -E scripts/run_joint_controller.sh
#
# Terminal B (this API):
#   scripts/run_path_ir_api_server.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="${ROOT}/lens_dual_arm_description"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${ROOT}/lens_dual_arm/install/setup.bash" ]]; then
  source "${ROOT}/lens_dual_arm/install/setup.bash"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

export LENS_FASTAPI_HOST="${LENS_FASTAPI_HOST:-0.0.0.0}"
export LENS_FASTAPI_PORT="${LENS_FASTAPI_PORT:-8000}"

export LENS_MAX_TOTAL_S="${LENS_MAX_TOTAL_S:-6}"
export LENS_EXEC_ROS_RESERVE_S="${LENS_EXEC_ROS_RESERVE_S:-0.25}"
export LENS_PLAN_RESERVE_S="${LENS_PLAN_RESERVE_S:-0.8}"
export LENS_EXEC_HOLD_S="${LENS_EXEC_HOLD_S:-0.05}"
export LENS_MIN_MOTION_S="${LENS_MIN_MOTION_S:-0.8}"
export LENS_EXEC_KEEPALIVE_HZ="${LENS_EXEC_KEEPALIVE_HZ:-40}"
export LENS_EXEC_MAX_DQ_RAD="${LENS_EXEC_MAX_DQ_RAD:-0.08}"
export LENS_SKIP_DRAW_RANGE_FIT=1
export LENS_FLANGE_PERP_XY=1
export LENS_FLANGE_XY_NORMAL="${LENS_FLANGE_XY_NORMAL:-down}"
export LENS_ENFORCE_TOOL_DOWN="${LENS_ENFORCE_TOOL_DOWN:-1}"
export LENS_FLANGE_PERP_XZ=0
export LENS_FLANGE_PERP_YZ=0
export LENS_FAST_EXECUTE=1
export LENS_AUTO_EXECUTE=1
export LENS_PLAN_CACHE=1
export LENS_PLAN_MAX_POINTS="${LENS_PLAN_MAX_POINTS:-20}"
export LENS_IK_MAX_ITERS="${LENS_IK_MAX_ITERS:-50}"
export LENS_IK_ROT_TOL="${LENS_IK_ROT_TOL:-0.005}"
export LENS_IK_ROT_WEIGHT="${LENS_IK_ROT_WEIGHT:-10}"
export LENS_IK_ORIENTATION_REFINE_ITERS="${LENS_IK_ORIENTATION_REFINE_ITERS:-18}"
export LENS_IK_WRIST_NULLSPACE_SCALE="${LENS_IK_WRIST_NULLSPACE_SCALE:-0}"
export LENS_IK_MULTI_WHEN_LOCKED="${LENS_IK_MULTI_WHEN_LOCKED:-1}"
export LENS_IK_WARM_START=1
export LENS_IK_START_AT_HOME=0
export LENS_SKIP_IK_GUARD=1

export LENS_ROS_BRIDGE="${LENS_ROS_BRIDGE:-1}"
export LENS_ROS_TOPIC="${LENS_ROS_TOPIC:-/joint_command}"

cd "${SIM_DIR}"
echo "[run] Path IR API http://${LENS_FASTAPI_HOST}:${LENS_FASTAPI_PORT}"
echo "[run] LENS_ROS_BRIDGE=${LENS_ROS_BRIDGE} topic=${LENS_ROS_TOPIC}"
exec python3 llm_demo/fastapi_path_ir_api_server.py
