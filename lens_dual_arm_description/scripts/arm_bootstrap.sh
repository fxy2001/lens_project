#!/usr/bin/env bash
# ARM(aarch64) 一键自举：整理 SDK、编译 joint_controller_node、输出运行命令模板。
#
# 用法（在仓库根目录执行）：
#   ./scripts/arm_bootstrap.sh
#
# 可选环境变量：
#   ROS_DISTRO_NAME=jazzy|humble|iron ...
#   ETHERCAT_IFNAME=enp12s0
#   MIT_KP=20
#   MIT_KD=8
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARCH="$(uname -m || true)"
if [[ "${ARCH}" != "aarch64" && "${ARCH}" != "arm64" ]]; then
  echo "[arm_bootstrap] WARNING: current arch is '${ARCH}', expected aarch64/arm64."
  echo "[arm_bootstrap] You can still run, but this script is intended for ARM machines."
fi

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
ETHERCAT_IFNAME="${ETHERCAT_IFNAME:-eth0}"
MIT_KP="${MIT_KP:-20}"
MIT_KD="${MIT_KD:-8}"

ARM_SDK_DIR="${ROOT}/actuator_SDK_ARM"
AARCH64_SRC="${ROOT}/actuator_controller_aarch64/actuator_controller_aarch64"
X86_JSON_DIR="${ROOT}/actuator_SDK_X86"

echo "[arm_bootstrap] root=${ROOT}"
echo "[arm_bootstrap] ROS_DISTRO_NAME=${ROS_DISTRO_NAME}"
echo "[arm_bootstrap] ETHERCAT_IFNAME=${ETHERCAT_IFNAME}"
echo "[arm_bootstrap] MIT_KP=${MIT_KP} MIT_KD=${MIT_KD}"

if [[ ! -d "${AARCH64_SRC}" ]]; then
  echo "[arm_bootstrap] ERROR: missing ARM SDK directory: ${AARCH64_SRC}"
  exit 2
fi

if [[ ! -f "${AARCH64_SRC}/libactuator_controller.a" || ! -f "${AARCH64_SRC}/libsoem.a" ]]; then
  echo "[arm_bootstrap] ERROR: ARM SDK missing required .a files under: ${AARCH64_SRC}"
  ls -la "${AARCH64_SRC}" || true
  exit 2
fi

if [[ ! -f "${X86_JSON_DIR}/actuator_config.json" ]]; then
  echo "[arm_bootstrap] ERROR: missing JSON config: ${X86_JSON_DIR}/actuator_config.json"
  echo "[arm_bootstrap] Provide actuator_config.json (arch-independent) to proceed."
  exit 2
fi

mkdir -p "${ARM_SDK_DIR}"

echo "[arm_bootstrap] preparing ${ARM_SDK_DIR}"
cp -a "${AARCH64_SRC}/." "${ARM_SDK_DIR}/"

if [[ ! -f "${ARM_SDK_DIR}/actuator_config.json" ]]; then
  cp -a "${X86_JSON_DIR}/actuator_config.json" "${ARM_SDK_DIR}/"
fi
if [[ -f "${X86_JSON_DIR}/actuator_params_config.json" && ! -f "${ARM_SDK_DIR}/actuator_params_config.json" ]]; then
  cp -a "${X86_JSON_DIR}/actuator_params_config.json" "${ARM_SDK_DIR}/"
fi

echo "[arm_bootstrap] SDK ready:"
ls -la "${ARM_SDK_DIR}" | sed -n '1,120p'

WS="${ROOT}/lens_dual_arm"
if [[ ! -d "${WS}" ]]; then
  echo "[arm_bootstrap] ERROR: missing ROS workspace: ${WS}"
  exit 2
fi

echo "[arm_bootstrap] building ROS workspace (joint_controller_node)..."
set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u
cd "${WS}"

colcon build --packages-up-to joint_controller_node \
  --cmake-args \
    -DENABLE_ACTUATOR_SDK=ON \
    -DACTUATOR_SDK_DIR="${ARM_SDK_DIR}"

EXE="${WS}/install/joint_controller_node/lib/joint_controller_node/joint_controller_executable"
if [[ ! -f "${EXE}" ]]; then
  echo "[arm_bootstrap] ERROR: build finished but executable not found:"
  echo "  ${EXE}"
  exit 3
fi
echo "[arm_bootstrap] OK: ${EXE}"

RUN_CMD_FILE="${ROOT}/dist/arm_run_joint_controller.sh"
mkdir -p "${ROOT}/dist"
cat > "${RUN_CMD_FILE}" <<EOF
#!/usr/bin/env bash
set -euo pipefail
ROOT="${ROOT}"
ROS_DISTRO_NAME="\${ROS_DISTRO_NAME:-${ROS_DISTRO_NAME}}"
ETHERCAT_IFNAME="\${ETHERCAT_IFNAME:-${ETHERCAT_IFNAME}}"
MIT_KP="\${MIT_KP:-${MIT_KP}}"
MIT_KD="\${MIT_KD:-${MIT_KD}}"

sudo -E bash -lc "
  set -e
  source /opt/ros/\${ROS_DISTRO_NAME}/setup.bash
  source ${WS}/install/setup.bash
  export ROS_LOG_DIR=/tmp/ros_log
  mkdir -p /tmp/ros_log
  ros2 run joint_controller_node joint_controller_executable \\
    --ros-args \\
    --params-file ${WS}/src/lens_arm_controller/src/lens_arm_controller/config/ethercat_control.yaml \\
    -p ethercat_ifname:=\${ETHERCAT_IFNAME} \\
    -p actuator_config_file:=${ARM_SDK_DIR}/actuator_config.json \\
    -p actuator_params_file:=${ARM_SDK_DIR}/actuator_params_config.json \\
    -p mit_kp:=\${MIT_KP} -p mit_kd:=\${MIT_KD}
"
EOF
chmod +x "${RUN_CMD_FILE}"

echo "[arm_bootstrap] generated run helper:"
echo "  ${RUN_CMD_FILE}"
echo
echo "[arm_bootstrap] next steps:"
echo "  1) edit ETHERCAT_IFNAME if needed: ip -br link"
echo "  2) run controller: ${RUN_CMD_FILE}"

