#!/usr/bin/env bash
# 编译 joint_controller_node
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROOT}/lens_dual_arm"
ARCH="$(uname -m)"
AARCH64_SRC="${ROOT}/actuator_controller_aarch64/actuator_controller_aarch64"
X86_SDK="${ROOT}/actuator_SDK_X86"
ARM_SDK="${ROOT}/actuator_SDK_ARM"

sdk_object_arch() {
  local lib="$1"
  local tmp
  tmp="$(mktemp)"
  readelf -h "${lib}" >"${tmp}" 2>/dev/null || true
  if grep -q 'Machine:.*AArch64' "${tmp}"; then
    echo "ARM aarch64"
  elif grep -q 'Machine:.*X86-64' "${tmp}"; then
    echo "x86-64"
  else
    echo "unknown"
  fi
  rm -f "${tmp}"
}

arch_matches_host() {
  local info="$1"
  case "${ARCH}" in
    aarch64|arm64)
      [[ "${info}" == *"aarch64"* || "${info}" == *"ARM"* ]]
      ;;
    x86_64|amd64)
      [[ "${info}" == *"x86-64"* || "${info}" == *"x86_64"* ]]
      ;;
    *)
      false
      ;;
  esac
}

prepare_arm_sdk() {
  if [[ ! -d "${AARCH64_SRC}" ]]; then
    return 1
  fi
  if [[ ! -f "${AARCH64_SRC}/libactuator_controller.a" || ! -f "${AARCH64_SRC}/libsoem.a" ]]; then
    return 1
  fi
  mkdir -p "${ARM_SDK}"
  cp -a "${AARCH64_SRC}/." "${ARM_SDK}/"
  if [[ -f "${X86_SDK}/actuator_config.json" ]]; then
    cp -a "${X86_SDK}/actuator_config.json" "${ARM_SDK}/"
  fi
  if [[ -f "${X86_SDK}/actuator_params_config.json" ]]; then
    cp -a "${X86_SDK}/actuator_params_config.json" "${ARM_SDK}/"
  fi
  echo "${ARM_SDK}"
}

resolve_sdk_dir() {
  local candidates=()
  if [[ "${ARCH}" == "aarch64" || "${ARCH}" == "arm64" ]]; then
    if prepared="$(prepare_arm_sdk 2>/dev/null)"; then
      candidates+=("${prepared}")
    fi
    if [[ -d "${AARCH64_SRC}" ]]; then
      candidates+=("${AARCH64_SRC}")
    fi
  fi
  candidates+=("${X86_SDK}")

  local dir info
  for dir in "${candidates[@]}"; do
    [[ -f "${dir}/libactuator_controller.a" ]] || continue
    info="$(sdk_object_arch "${dir}/libactuator_controller.a")"
    if arch_matches_host "${info}"; then
      echo "[build] 使用 SDK: ${dir} (${info})" >&2
      echo "${dir}"
      return 0
    fi
  done
  return 1
}

# ROS setup 脚本会读取未定义变量；source 前临时关闭 nounset
set +u
source /opt/ros/humble/setup.bash
set -u
cd "${WS}"

CMAKE_ARGS=()
WANT_SDK=0
if [[ "${ENABLE_ACTUATOR_SDK:-auto}" == "1" || "${ENABLE_ACTUATOR_SDK:-auto}" == "ON" ]]; then
  WANT_SDK=1
elif [[ "${ENABLE_ACTUATOR_SDK:-auto}" == "auto" ]]; then
  WANT_SDK=1
fi

if [[ "${WANT_SDK}" -eq 1 ]]; then
  if SDK_DIR="$(resolve_sdk_dir)"; then
    CMAKE_ARGS+=(
      -DENABLE_ACTUATOR_SDK=ON
      "-DACTUATOR_SDK_DIR=${SDK_DIR}"
    )
  elif [[ "${ENABLE_ACTUATOR_SDK:-auto}" == "1" || "${ENABLE_ACTUATOR_SDK:-auto}" == "ON" ]]; then
    echo "[build] 错误: 未找到与本机 ${ARCH} 匹配的 SDK"
    echo "[build] 请检查 actuator_controller_aarch64/ 或 actuator_SDK_X86/"
    exit 1
  else
    echo "[build] 未找到匹配 SDK，以话题模式编译"
  fi
fi

colcon build --packages-up-to joint_controller_node --cmake-clean-cache --cmake-args "${CMAKE_ARGS[@]}"
echo ""
echo "编译完成。启动: scripts/run_joint_controller.sh"
