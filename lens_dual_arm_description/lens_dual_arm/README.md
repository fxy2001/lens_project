# lens_dual_arm（ROS2 工作区）

本目录为 **ROS 2** 工作区，与上一级 [MuJoCo 仿真包](../lens_dual_arm_description/README.md) 配合，形成「仿真规划 → 话题/服务 → EtherCAT 真机」链路。  
**仓库总览与架构图**：见 [../README.md](../README.md)。

## 本工作区主要包与文件

| 路径 | 说明 |
|------|------|
| `src/lens_arm_controller/src/lens_arm_controller/` | `joint_controller_node` 源码、`config/ethercat_control.yaml` |
| `src/lens_arm_controller/src/robot_interface/` | 自定义 `MotionExecute` 等接口消息/服务 |
| `docker/ethercat-build/` | 隔离编译镜像与 `build_joint_controller_in_container.sh` |

## `joint_controller_node` 要点（与仿真联调前必读）

- **编译开关**：`ENABLE_ACTUATOR_SDK=ON` 时链接 `actuator_SDK` 静态库，方可真正下发 EtherCAT。  
- **参数文件**：`config/ethercat_control.yaml` 中配置 `ethercat_ifname`、`actuator_config_file`、`actuator_params_file`、`mit_kp` / `mit_kd` 等；**请按现场网口名修改**（如 `enp12s0`），JSON 路径需指向本机 `actuator_SDK_X86`。  
- **运行时库**：若链接了自定义前缀的 `fmt`/`spdlog`，运行时需将对应 `lib` 目录加入 `LD_LIBRARY_PATH`（Docker 构建说明见下文）。  
- **话题**：**订阅** `/joint_states`、**订阅** `/joint_command`（外部轨迹，参数 `accept_external_joint_command`）；**发布** `/joint_command`（内部回放监控）。外部轨迹经 `sendJointCommandToEthercat` 下发。若修改源码，需 `scripts/build_joint_controller.sh` 后重启。
- **权限**：EtherCAT 与部分环境需 `root` 或 `sudo`；`sudo` 时注意保留 `source install/setup.bash` 与 `LD_LIBRARY_PATH`（例如 `sudo -E bash -lc '...'`）。

## ROS相关库安装
1. 安装ROS2，只安装Ubuntu (Debian 包)即可，参考网址：http://fishros.org/doc/ros2/humble/Installation/Ubuntu-Install-Debians.html
2. 安装moveit：依次执行以下指令
（1） sudo apt install ros-humble-turtlesim
（2） sudo apt install ros-humble-moveit-common
（3） sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
（4） sudo apt install ros-humble-ros2-controllers
（5） sudo apt install ros-humble-ros2-control ros-humble-controller-interface ros-humble-hardware-interface
（6） sudo apt install ros-humble-controller-*
（7） sudo apt install ros-humble-topic-based-ros2-control
（8） sudo apt install ros-humble-rqt-controller-manager


## 软件使用
```bash
# 新建3个终端 分别进入到软件主目录 然后用下面指令分别进入到root用户
sudo su

# 第一个终端运行驱动程序
./dual_arm_actuator1.sh

# 第二个终端运行moveit可视化
./dual_arm_moveit2.sh

# 第三个终端运行moveit使能
./dual_arm_moveit3.sh

# 通过可视化界面选择对应的手臂进行控制，可修改目标位置、速度等参数

# 关闭程序
按 ctrl+c 即可
```

## 注意事项
1.由于有些关节没有机械限位，请勿手动旋转关节超过90°，防止绕线问题。
2.驱动程序关闭时会关闭使能，由于机械臂没有抱闸会自由下落。

## EtherCAT 隔离构建（推荐）
当 `libactuator_controller.a` 与本机 `fmt/spdlog` 版本不匹配时，建议使用容器编译：

```bash
cd /home/ylgy/桌面/lens_dual_arm_description/lens_dual_arm
./docker/ethercat-build/build_joint_controller_in_container.sh \
  /home/ylgy/桌面/lens_dual_arm_description/actuator_SDK_X86
```

说明：
- 容器基于 Ubuntu 24.04 + ROS2 Jazzy
- 容器内固定构建 `fmt 8.1.1` 与 `spdlog 1.9.2`（与 `libactuator_controller.a` 常用组合一致，以 Dockerfile 为准）
- 编译目标：`joint_controller_node`（`ENABLE_ACTUATOR_SDK=ON`）

