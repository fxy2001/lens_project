#!/usr/bin/env bash
# 启动 joint_controller_node（EtherCAT 真机 / ROS2 话题）
#
# 网卡与 RMW 默认值参考 read/LANSI机械臂 enthercat.md（S100/Humble 段）：
#   - Humble：unset RMW_IMPLEMENTATION（或自动选已安装的 fastrtps）
#   - S100 aarch64 示例网卡：eth1；若网卡带 PROMISC 标志则优先选该口（常见 EtherCAT）
#
# 用法：
#   sudo -E scripts/run_joint_controller.sh
#   ETHERCAT_IFNAME=eth1 sudo -E scripts/run_joint_controller.sh
#
# 停止（勿直接 Ctrl+C 后立刻重启，易残留进程占住 eth1）：
#   sudo scripts/stop_joint_controller.sh
set -euo pipefail

# EtherCAT 网卡只能被一个主站占用；重复启动会导致 OP 失败、workcounter=0
LOCK_FILE="${LENS_JOINT_CONTROLLER_LOCK:-/tmp/lens_joint_controller.lock}"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "[run] 错误: 已有 joint_controller 在运行（锁: ${LOCK_FILE}）" >&2
  echo "[run] 请先结束旧进程: sudo pkill -f joint_controller_executable" >&2
  echo "[run] 等待 3s 后只启动一次: sudo -E scripts/run_joint_controller.sh" >&2
  exit 1
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROOT}/lens_dual_arm"
PARAMS="${WS}/install/joint_controller_node/share/joint_controller_node/config/ethercat_control.yaml"
RUNTIME_LIBS="${WS}/third_party_runtime_libs"
ARM_SDK="${ROOT}/actuator_SDK_ARM"

detect_ethercat_ifname() {
  if [[ -n "${ETHERCAT_IFNAME:-}" ]]; then
    echo "${ETHERCAT_IFNAME}"
    return
  fi
  # 优先选有物理链路（LOWER_UP）的网口 — 实测 EtherCAT 从站在 eth1
  local linked_if
  linked_if="$(ip -br link 2>/dev/null | awk '$0 ~ /LOWER_UP/ && $1 !~ /^(lo|wlan)/ {print $1; exit}')"
  if [[ -n "${linked_if}" ]]; then
    echo "${linked_if}"
    return
  fi
  # 其次：带 PROMISC 的网口（常见 EtherCAT 专用口）
  local promisc_if
  promisc_if="$(ip -br link 2>/dev/null | awk '$0 ~ /<[^>]*PROMISC/ {print $1; exit}')"
  if [[ -n "${promisc_if}" ]]; then
    echo "${promisc_if}"
    return
  fi
  # read/LANSI机械臂 enthercat.md — S100 Humble 示例
  if ip link show eth1 &>/dev/null; then
    echo "eth1"
    return
  fi
  echo "eth0"
}

ensure_iface_up() {
  local ifname="$1"
  [[ -n "${ifname}" ]] || return 0
  if ! ip link show "${ifname}" &>/dev/null; then
    echo "[run] warning: interface ${ifname} not found"
    return 1
  fi
  local state
  state="$(ip -br link show "${ifname}" 2>/dev/null | awk '{print $2}')"
  if [[ "${state}" != "UP" ]]; then
    echo "[run] bringing ${ifname} up (was ${state})"
    ip link set "${ifname}" up
  fi
  local wait_s="${ETHERCAT_LINK_WAIT_S:-8}"
  local i
  for ((i = 0; i < wait_s; i++)); do
    if ip -br link show "${ifname}" 2>/dev/null | grep -q "LOWER_UP"; then
      echo "[run] ${ifname} link ready (LOWER_UP)"
      return 0
    fi
    sleep 1
  done
  echo "[run] warning: ${ifname} has no carrier after ${wait_s}s — check EtherCAT cable / robot power"
  return 1
}

setup_rmw() {
  if [[ -n "${RMW_IMPLEMENTATION:-}" ]]; then
    return
  fi
  local ros_lib="/opt/ros/humble/lib"
  if [[ -f "${ros_lib}/librmw_cyclonedds_cpp.so" ]]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  elif [[ -f "${ros_lib}/librmw_fastrtps_cpp.so" ]]; then
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  else
    unset RMW_IMPLEMENTATION
  fi
}

if [[ ! -f "${WS}/install/setup.bash" ]]; then
  echo "工作区未编译。请先运行: scripts/build_joint_controller.sh"
  exit 1
fi

# 避免 Ctrl+C 后直接重启导致 EtherCAT 网卡仍被占用、外部指令无效
if pgrep -f joint_controller_executable >/dev/null 2>&1; then
  echo "[run] 检测到旧 joint_controller，先执行 stop_joint_controller.sh ..."
  "${ROOT}/scripts/stop_joint_controller.sh" || true
fi

IFNAME="$(detect_ethercat_ifname)"
ensure_iface_up "${IFNAME}" || true
setup_rmw

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"
mkdir -p "${ROS_LOG_DIR}"

# aarch64 真机构建使用系统 fmt/spdlog；x86 可能需要 third_party_runtime_libs
if [[ "$(uname -m)" == "x86_64" ]]; then
  export LD_LIBRARY_PATH="${RUNTIME_LIBS}:${LD_LIBRARY_PATH:-}"
fi

EXTRA_ARGS=()
# 左臂肩 pitch + 肘 pitch（实测）；关闭: export LENS_INVERT_JOINT_NAMES=""
export LENS_INVERT_JOINT_NAMES="${LENS_INVERT_JOINT_NAMES:-Left_Shoulder_Pitch_Joint,Left_Elbow_Pitch_Joint,Left_Wrist_Yaw_Joint,Left_Wrist_Roll_Joint,Left_Wrist_Pitch_Joint}"
if [[ -n "${LENS_INVERT_JOINT_NAMES}" ]]; then
  EXTRA_ARGS+=(-p "invert_joint_names:=${LENS_INVERT_JOINT_NAMES}")
fi
if [[ -f "${ARM_SDK}/actuator_config.json" ]]; then
  EXTRA_ARGS+=(-p "actuator_config_file:=${ARM_SDK}/actuator_config.json")
fi
if [[ -f "${ARM_SDK}/actuator_params_config.json" ]]; then
  EXTRA_ARGS+=(-p "actuator_params_file:=${ARM_SDK}/actuator_params_config.json")
fi

echo "[run] joint_controller_executable"
echo "[run]   ethercat_ifname=${IFNAME}"
echo "[run]   RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default>}"
echo "[run]   ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[run]   invert_joint_names=${LENS_INVERT_JOINT_NAMES:-<off>}"
echo "[run] 本机网卡: $(ip -br link 2>/dev/null | tr '\n' ' ')"

exec ros2 run joint_controller_node joint_controller_executable \
  --ros-args \
  --params-file "${PARAMS}" \
  -p "ethercat_ifname:=${IFNAME}" \
  "${EXTRA_ARGS[@]}"
