#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SDK_DIR_DEFAULT="$(cd "${WS_ROOT}/.." && pwd)/actuator_SDK_X86"
SDK_DIR="${1:-${SDK_DIR_DEFAULT}}"
IMAGE_NAME="lens-ethercat-build:jz-fmt8-spdlog110"

if [[ ! -d "${SDK_DIR}" ]]; then
  echo "SDK dir not found: ${SDK_DIR}"
  echo "Usage: $0 /abs/path/to/actuator_SDK_X86"
  exit 1
fi
if [[ ! -f "${SDK_DIR}/libactuator_controller.a" ]]; then
  echo "Missing ${SDK_DIR}/libactuator_controller.a"
  exit 1
fi
if [[ ! -f "${SDK_DIR}/libsoem.a" ]]; then
  echo "Missing ${SDK_DIR}/libsoem.a"
  exit 1
fi

docker build -t "${IMAGE_NAME}" -f "${SCRIPT_DIR}/Dockerfile" "${SCRIPT_DIR}"

docker run --rm -it \
  -v "${WS_ROOT}:/work/lens_dual_arm" \
  -v "${SDK_DIR}:/work/actuator_SDK_X86:ro" \
  "${IMAGE_NAME}" \
  bash -lc '
    set -euo pipefail
    # ROS setup scripts may read unset vars; disable nounset temporarily.
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
    cd /work/lens_dual_arm
    colcon build --packages-up-to joint_controller_node \
      --cmake-clean-cache \
      --cmake-args \
        -DENABLE_ACTUATOR_SDK=ON \
        -DACTUATOR_SDK_DIR=/work/actuator_SDK_X86 \
        -DCMAKE_PREFIX_PATH=/opt/third_party:/opt/ros/jazzy
  '

EXE_PATH="${WS_ROOT}/install/joint_controller_node/lib/joint_controller_node/joint_controller_executable"
if [[ -f "${EXE_PATH}" ]]; then
  echo "Build finished. Executable:"
  ls -lah "${EXE_PATH}"
else
  echo "Build finished but executable not found at expected path:"
  echo "  ${EXE_PATH}"
  echo "Check logs under ${WS_ROOT}/log/latest_build/joint_controller_node/"
  exit 2
fi
