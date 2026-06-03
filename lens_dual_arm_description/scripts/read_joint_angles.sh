#!/usr/bin/env bash
# 读取当前机械臂关节角（弧度），需 joint_controller 已发布 /joint_states
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROOT}/lens_dual_arm"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${WS}/install/setup.bash" ]]; then
  source "${WS}/install/setup.bash"
fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

_topic_info() {
  # 避免 grep --color=auto 别名在管道里导致 grep -q 误判
  command ros2 topic info /joint_states 2>/dev/null
}

_pub_count="$(_topic_info | command grep -E '^Publisher count:' | awk '{print $3}')"
if [[ -z "${_pub_count}" || "${_pub_count}" == "0" ]]; then
  echo "错误: /joint_states 尚无发布者（Publisher count=${_pub_count:-?}）。" >&2
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}（须与 joint_controller 一致，默认 0）" >&2
  _topic_info >&2 || true
  echo "请先启动且仅启动一次驱动: sudo -E scripts/run_joint_controller.sh" >&2
  echo "日志需含: EtherCAT control enabled / Publishing /joint_states" >&2
  echo "勿出现: fallback to topic-only mode" >&2
  exit 1
fi
unset _topic_info _pub_count

exec python3 "${ROOT}/lens_dual_arm_description/llm_demo/read_joint_angles.py" "$@"
