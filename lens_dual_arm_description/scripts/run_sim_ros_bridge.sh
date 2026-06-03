#!/usr/bin/env bash
# 仿真 + ROS2 桥接示例（左臂 IK 滑条）
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="${ROOT}/lens_dual_arm_description"
VENV="${SIM_DIR}/.venv"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${ROOT}/lens_dual_arm/install/setup.bash" ]]; then
  source "${ROOT}/lens_dual_arm/install/setup.bash"
fi
if [[ -f "${VENV}/bin/activate" ]]; then
  source "${VENV}/bin/activate"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ -z "${RMW_IMPLEMENTATION:-}" ]]; then
  if [[ -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  elif [[ -f /opt/ros/humble/lib/librmw_fastrtps_cpp.so ]]; then
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  else
    unset RMW_IMPLEMENTATION
  fi
fi
export LENS_ROS_BRIDGE="${LENS_ROS_BRIDGE:-1}"
export LENS_ROS_TOPIC="${LENS_ROS_TOPIC:-/joint_command}"
export LENS_ROS_HZ="${LENS_ROS_HZ:-20}"

cd "${SIM_DIR}"
exec python left_arm_ik_slider.py
