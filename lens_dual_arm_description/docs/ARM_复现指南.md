# ARM（aarch64）机器上“复制过去完美复现”指南

本文面向场景：

- 你在 **x86_64** 开发、修改代码；
- 对方机器是 **ARM64 / aarch64**（例如 Jetson / RK / ARM 工控机）；
- 你希望通过「拷贝项目目录」的方式，让对方 **完全复刻**：仿真、ROS2、编译控制器、EtherCAT 真机联调。

> 若对方机器允许联网且装有 Docker，也可以在 ARM 上直接 `docker build`（镜像会按架构选择 ARM SDK）。但本指南以“复制项目 + 原生构建”为主。

---

## 0. 你需要拷过去的目录清单（务必完整）

把仓库根目录整体复制到 ARM 机器（例如 `/home/user/lens_dual_arm_description/`），至少包含：

- `lens_dual_arm_description/`（MuJoCo 仿真脚本与模型）
- `lens_dual_arm/`（ROS2 工作区源码：`joint_controller_node`、`robot_interface`）
- `actuator_controller_aarch64/`（ARM64 版 SDK 静态库与头文件）
- `actuator_SDK_X86/actuator_config.json`、`actuator_SDK_X86/actuator_params_config.json`（JSON 配置文件，**架构无关**）

为什么 ARM 还需要带 `actuator_SDK_X86` 的 JSON？

- 你提供的 `actuator_controller_aarch64` 目录内通常只有 `.a` 和头文件；
- 但控制器初始化 `initCanIdAndEthercatRelation(ifname, config_file, ...)` 需要 `actuator_config.json`；
- 可选参数 `loadActuatorParamsConfig(params_file)` 需要 `actuator_params_config.json`。

> 如果你已经从厂商拿到 ARM 版同名 JSON，优先用 ARM 版；否则用 x86 的 JSON 作为配置源（只要关节名、CAN ID、slave/port 映射一致即可）。

---

## 0.5 一键复现（推荐）

如果你希望对方“复制过去就能跑”，推荐直接在仓库根目录执行一条命令：

```bash
cd /path/to/lens_dual_arm_description
chmod +x scripts/arm_bootstrap.sh
./scripts/arm_bootstrap.sh
```

它会自动完成：

- 生成 `actuator_SDK_ARM/`（ARM `.a` + 头文件 + 补齐 JSON）
- 编译 `lens_dual_arm/` 工作区里的 `joint_controller_node`
- 生成可直接运行的脚本：`dist/arm_run_joint_controller.sh`

你只需要按提示修改网卡名（`ETHERCAT_IFNAME`）并执行运行脚本即可。

> `arm_bootstrap.sh` 默认 ROS 发行版为 `jazzy`。如果你的 ARM 机是 Ubuntu 22.04 + Humble，运行前先设置：`export ROS_DISTRO_NAME=humble`。

---

## 1. ARM 机器环境要求

### 1.1 系统

- Ubuntu 22.04/24.04（建议与 ROS2 发行版匹配）
- 有图形界面（若要跑 MuJoCo GUI）；纯联调可无 GUI

### 1.2 EtherCAT 运行权限（真机时）

常见要求：

- 需要 root 权限访问原始套接字/网卡（SOEM/EtherCAT 主站）
- 网卡名明确（如 `enp1s0` / `eth0`），链路 UP 且接线正确

---

## 2. 在 ARM 上复现“仿真侧”（Python + MuJoCo）

进入仿真目录：

```bash
cd /path/to/lens_dual_arm_description/lens_dual_arm_description
```

安装依赖（建议用 venv）：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip wheel
pip install -r requirements.txt
```

如果报 `No module named 'tkinter'`：

```bash
sudo apt-get update
sudo apt-get install -y python3-tk
```

基础自检：

```bash
python -c "import mujoco, numpy; print('OK', mujoco.__version__)"
```

---

## 3. 在 ARM 上安装 ROS2（用于控制器与 ROS bridge）

本项目文档以 **ROS2 Jazzy** 为例（Ubuntu 24.04），你也可以用 Humble/Iron 等，但需确保：

- C++ 控制器 `rclcpp` 等依赖可用
- Python 仿真若要发布 `JointState`，需要 `rclpy`

安装完成后确认：

```bash
source /opt/ros/<distro>/setup.bash
ros2 --help
```

> 安装 ROS2 的官方步骤变化较快，请按你设备实际发行版与官方文档执行；完成后再继续下一节编译。

---

## 4. 在 ARM 上编译 `joint_controller_node`（使用 ARM SDK 静态库）

### 4.1 准备一个“ARM 专用 SDK 目录”（推荐）

建议在 ARM 机器上整理一个目录，把 ARM `.a` + 头文件 + JSON 放在一起，形成“等价于 `actuator_SDK_X86/` 的结构”：

```bash
cd /path/to/lens_dual_arm_description

mkdir -p actuator_SDK_ARM

# 1) 拷贝 ARM 静态库与头文件
cp -a actuator_controller_aarch64/actuator_controller_aarch64/* actuator_SDK_ARM/

# 2) 补齐 JSON（从 x86 目录复制；或换成厂商给的 ARM JSON）
cp -a actuator_SDK_X86/actuator_config.json actuator_SDK_ARM/
cp -a actuator_SDK_X86/actuator_params_config.json actuator_SDK_ARM/
```

完成后，`actuator_SDK_ARM/` 目录应至少包含：

- `libactuator_controller.a`
- `libsoem.a`
- `actuator_controller.h`
- `motor_mode_enum.h`
- `actuator_config.json`
- `actuator_params_config.json`（可选但推荐）

### 4.2 编译 ROS2 工作区

```bash
cd /path/to/lens_dual_arm_description/lens_dual_arm
source /opt/ros/<distro>/setup.bash

colcon build --packages-up-to joint_controller_node \
  --cmake-args \
    -DENABLE_ACTUATOR_SDK=ON \
    -DACTUATOR_SDK_DIR=/path/to/lens_dual_arm_description/actuator_SDK_ARM
```

编译成功后应存在可执行文件：

```bash
ls -la install/joint_controller_node/lib/joint_controller_node/joint_controller_executable
```

---

## 5. ARM 上运行真机控制器（EtherCAT）

建议先确认网卡：

```bash
ip -br link
```

运行（通常需要 sudo）：

```bash
cd /path/to/lens_dual_arm_description/lens_dual_arm

sudo -E bash -lc '
  set -e
  source /opt/ros/<distro>/setup.bash
  source install/setup.bash
  export ROS_LOG_DIR=/tmp/ros_log
  mkdir -p /tmp/ros_log
  ros2 run joint_controller_node joint_controller_executable \
    --ros-args \
    --params-file /path/to/lens_dual_arm_description/lens_dual_arm/src/lens_arm_controller/src/lens_arm_controller/config/ethercat_control.yaml \
    -p ethercat_ifname:=enp12s0 \
    -p actuator_config_file:=/path/to/lens_dual_arm_description/actuator_SDK_ARM/actuator_config.json \
    -p actuator_params_file:=/path/to/lens_dual_arm_description/actuator_SDK_ARM/actuator_params_config.json \
    -p mit_kp:=20.0 -p mit_kd:=8.0
'
```

> 注意：上面的 `ethercat_control.yaml` 默认路径可能是 `eth0`、以及 JSON 指向 x86 目录；在 ARM 上建议用命令行参数覆盖成 ARM 的绝对路径（如上所示），这样**不需要修改源码/仓库文件**，复制过去也不怕路径不一致。

---

## 6. ARM 上运行仿真并发布 ROS 关节命令（联调）

终端 A（真机控制器）按 §5 启动。终端 B（仿真）：

```bash
source /opt/ros/<distro>/setup.bash
cd /path/to/lens_dual_arm_description/lens_dual_arm_description
source .venv/bin/activate

export LENS_ROS_BRIDGE=1
export LENS_ROS_TOPIC=/joint_command
export LENS_ROS_HZ=60

python dual_arm_ik_slider.py
```

联调验收点：

- `ros2 topic info /joint_command` 能看到发布者与订阅者
- 控制器端能看到收到命令的日志/计数增长（若你分支实现了外部订阅）
- 电机有响应（MIT `kp/kd` 合适、方向映射已校准）

---

## 7. 常见“复制过去跑不起来”的原因速查

| 现象 | 优先排查 |
|------|----------|
| SDK 初始化失败 | JSON 路径不对 / JSON 缺失 / 网卡名不对 / 未 root |
| `ros2` 找不到 | 没有 `source /opt/ros/<distro>/setup.bash` |
| 控制器收不到仿真话题 | `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` 不一致；或控制器未订阅外部话题 |
| MuJoCo 无窗口 | 没有桌面/X11；或缺 OpenGL 相关库（ARM 设备需按显卡驱动情况配置） |

---

## 8. 推荐的“可复刻”策略（不改仓库文件）

为了让“拷贝过去就能跑”，建议你在 ARM 机器上始终：

- 用 **绝对路径**覆盖 `actuator_config_file` / `actuator_params_file`；
- 用参数覆盖 `ethercat_ifname`；
- 不依赖宿主机中文路径；
- 需要 sudo 时用 `sudo -E bash -lc 'source ...; ros2 run ...'` 保留环境。

