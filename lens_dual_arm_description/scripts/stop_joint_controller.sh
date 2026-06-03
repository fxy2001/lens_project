#!/usr/bin/env bash
# 安全停止 joint_controller，释放 EtherCAT 网卡与 flock 锁
set -euo pipefail

LOCK_FILE="${LENS_JOINT_CONTROLLER_LOCK:-/tmp/lens_joint_controller.lock}"
IFNAME="${ETHERCAT_IFNAME:-eth1}"

# 只匹配真机节点进程，勿用 "joint_controller" 宽泛匹配（会误伤本 stop 脚本）
PATTERN_EXEC='joint_controller_executable'
PATTERN_ROS2='ros2 run joint_controller_node'

_count_running() {
  local n_exec n_ros
  n_exec="$(pgrep -cf "${PATTERN_EXEC}" 2>/dev/null || echo 0)"
  n_ros="$(pgrep -cf "${PATTERN_ROS2}" 2>/dev/null || echo 0)"
  echo $((n_exec + n_ros))
}

_list_running() {
  pgrep -af "${PATTERN_EXEC}" 2>/dev/null || true
  pgrep -af "${PATTERN_ROS2}" 2>/dev/null || true
}

if [[ "$(id -u)" -eq 0 ]]; then
  PKILL=(pkill)
  IP=(ip)
  RM=(rm -f)
else
  PKILL=(sudo pkill)
  IP=(sudo ip)
  RM=(sudo rm -f)
fi

echo "[stop] 正在结束 joint_controller 相关进程..."
"${PKILL[@]}" -TERM -f "${PATTERN_EXEC}" 2>/dev/null || true
sleep 2
"${PKILL[@]}" -KILL -f "${PATTERN_EXEC}" 2>/dev/null || true
"${PKILL[@]}" -TERM -f "${PATTERN_ROS2}" 2>/dev/null || true
sleep 1
"${PKILL[@]}" -KILL -f "${PATTERN_ROS2}" 2>/dev/null || true

remaining="$(_count_running)"
if [[ "${remaining}" != "0" ]]; then
  echo "[stop] 警告: 仍有 ${remaining} 个 joint_controller 进程未退出:" >&2
  _list_running >&2 || true
  exit 1
fi

"${RM[@]}" "${LOCK_FILE}"
echo "[stop] 已删除锁文件: ${LOCK_FILE}"

if "${IP[@]}" link show "${IFNAME}" &>/dev/null; then
  echo "[stop] 复位网卡 ${IFNAME} (down → up)..."
  "${IP[@]}" link set "${IFNAME}" down 2>/dev/null || true
  sleep 2
  "${IP[@]}" link set "${IFNAME}" up 2>/dev/null || true
  sleep 2
  "${IP[@]}" -br link show "${IFNAME}" || true
fi

echo "[stop] 完成。等待 3s 后再启动: sudo -E scripts/run_joint_controller.sh"
sleep 3
