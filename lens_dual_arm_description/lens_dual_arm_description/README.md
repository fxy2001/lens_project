# lens_dual_arm_description

双臂 LENS 机器人的 **URDF / MJCF 描述**与 **MuJoCo 仿真工具链**：FK/IK、关节空间规划（含身体避障）、可选 **CAN/EtherCAT 协议桥接打印**，以及可选 **ROS2 `JointState` 发布**，用于与真机执行栈联调。

> **仓库总览**（多目录）：见上一级 [../README.md](../README.md)。

---

## 1. 项目在整个系统中的位置

```text
MJCF + meshes          仿真 Python 脚本              ROS2 / 真机
─────────────────      ───────────────────────      ─────────────────────────
mjcf/*.xml      ──►    fk_viewer / IK / 规划   ──►    /joint_command (可选)
meshes/*.STL           RealTimeBridge (打印)          joint_controller + SDK
urdf/*.urdf            motor_can_protocol             EtherCAT → 电机 MIT
```

- **仿真**在本地进程内用 MuJoCo 更新 `qpos`、做碰撞检测与规划。  
- **联调**时，可将**同一套关节角**通过 ROS2 话题（默认 `/joint_command`）送出；真机侧需有节点订阅并下发到驱动（见上一级 README 中「与 C++ 控制器的衔接说明」）。

---

## 2. 目录与关键文件

| 路径 | 作用 |
|------|------|
| `mjcf/` | MuJoCo 模型（推荐入口如 `mj_lens_dual_arm.xml`，另有 `mj_lens_dual_arm_feedforward.xml` 等变体） |
| `urdf/` | URDF；若直接加载需注意 mesh 路径 |
| `meshes/` | STL 网格 |
| `requirements.txt` | Python 最小依赖：`numpy`、`mujoco` |
| `mj_kinematics_env.py` | 双臂关节名列表、FK、共用 MuJoCo 加载逻辑 |
| `user_fk.py` | 用户自定义 FK 接口（与 `fk_viewer.py` 对照误差） |
| `fk_viewer.py` | FK 可视化与仿真/用户 FK 对比 |
| `left_arm_ik_slider_3d.py` | 左臂 **仅位置** IK（XYZ），滑条 GUI |
| `left_arm_ik_slider.py` | 左臂 **6D** IK（XYZ + RPY），内含 `Ros2JointCommandPublisher`，可发布左臂 7 关节 |
| `right_arm_ik_slider.py` | 右臂 6D IK + ROS 桥接（7 关节） |
| `dual_arm_ik_slider.py` | **双臂** 6D IK，两个滑条窗口 + **14 关节** ROS 发布 |
| `left_arm_plan_collision.py` | 身体/自碰避障：`mj_geomDistance`、自适应采样、失败诊断打印 |
| `left_arm_path_planner.py` | 左臂 RRT-Connect + shortcut + `RealTimeBridge` |
| `left_arm_slider_trajectory.py` | 左臂「目标 XYZ → 规划并执行」+ 虚影预览 + `RealTimeBridge` + ROS 桥接 |
| `motor_can_protocol.py` | 按《电驱通讯协议》MIT **0x0B** 打包（`util_float2Uint` 等） |
| `realtime_bridge.py` | `can`：打印标准 CAN 帧；`ethercat`：按 `actuator_SDK` 语义打印批量 MIT |

---

## 3. 外部库与可选依赖

### 3.1 `requirements.txt`（pip）

- **NumPy**：数组与数值计算。  
- **MuJoCo (≥3)**：仿真、雅可比、几何距离、viewer。

安装示例：

```bash
cd /path/to/lens_dual_arm_description
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip setuptools wheel
pip install -r requirements.txt
python -c "import mujoco, numpy; print('OK')"
```

### 3.2 系统 / 可选 Python 组件

| 组件 | 用途 |
|------|------|
| **桌面 OpenGL** | MuJoCo 原生 viewer |
| **`python3-tk`（系统包）** | `tkinter`；IK/规划 GUI。未安装会 `ModuleNotFoundError: tkinter` |
| **ROS 2 Jazzy（示例）+ `rclpy`** | 仅当设置 `LENS_ROS_BRIDGE=1` 时；需先 `source /opt/ros/<distro>/setup.bash` |

**与 ROS 共用 Python 的推荐做法**：使用带 `--system-site-packages` 的 venv，或先 source ROS 再激活 venv，以便 `rclpy` 与 `mujoco` 同时可用（具体以你机上的 ROS 安装方式为准）。

### 3.3 厂商文档与 SDK（不在本目录内时）

- 《电驱通讯协议》等：指导 CAN 帧字段与 MIT 打包（与 `motor_can_protocol.py` 对应）。  
- `actuator_SDK` / `lens_actuator_controller_guide`：EtherCAT 初始化与 `setTargetMit` 语义（与 `realtime_bridge.py` 的 `ethercat` 模式打印对应）。  
- `actuator_config.json`：关节名与 EtherCAT/CAN 映射；`RealTimeBridge` 可读取用于打印 slave/port。

---

## 4. 关节与模型约定

- **左臂 7 轴**（顺序固定，与消息 `name` 数组一致时使用同名）：  
  `Left_Shoulder_Pitch_Joint`, `Left_Shoulder_Roll_Joint`, `Left_Shoulder_Yaw_Joint`,  
  `Left_Elbow_Pitch_Joint`, `Left_Wrist_Yaw_Joint`, `Left_Wrist_Roll_Joint`, `Left_Wrist_Pitch_Joint`  
- **右臂 7 轴**：将上述 `Left_` 换成 `Right_`。  
- 具体索引与 FK 辅助函数见 `mj_kinematics_env.py`。

**仿真 vs 真机方向**：MuJoCo 中关节正方向由模型 `axis` 决定；真机电机/编码器符号可能相反。是否在驱动层对某关节取反由 **真机 ROS 控制器参数或标定**决定，不是本目录 Python 文件的硬编码前提。

---

## 5. 运行说明（仿真）

### 5.1 FK 对照

```bash
python fk_viewer.py
```

### 5.2 左臂 IK

```bash
# 仅末端位置（3D）
python left_arm_ik_slider_3d.py

# 末端位姿 6D（含 ROS 桥接环境变量见 §7）
python left_arm_ik_slider.py
```

### 5.3 右臂 / 双臂 6D IK

```bash
python right_arm_ik_slider.py
python dual_arm_ik_slider.py
```

`dual_arm_ik_slider.py` 在开启 ROS 桥接时发布 **14** 个关节名与位置。

### 5.4 左臂路径规划（RRT-Connect）

```bash
python left_arm_path_planner.py
```

- **Plan**：IK 目标关节 + RRT-Connect + shortcut + 加密轨迹。  
- **Execute**：执行离散轨迹；可配合 `RealTimeBridge` 打印 CAN/EtherCAT 语义。  
- **避障**：`left_arm_plan_collision.py` 中间距检测；失败时打印 IK/起终点/碰撞对等诊断。

### 5.5 左臂滑条轨迹（一键「规划并移动」）

```bash
python left_arm_slider_trajectory.py
```

- 目标为 **3D 位置 IK**；RRT 失败时可尝试关节空间直线插值（限位 + 避障）。  
- 拖动滑条时浅蓝色虚影为 IK 预览；执行轨迹时关闭虚影。

---

## 6. 虚拟→现实：终端桥接（不经过 ROS）

在 `left_arm_path_planner.py` 与 `left_arm_slider_trajectory.py` 中生效。

```bash
# can：打印 MIT 0x0B 标准帧；ethercat：打印与 SDK 一致的批量 MIT 说明
export LENS_BRIDGE_PROTOCOL=can

# 左臂 7 个电机 CAN ID（顺序同左臂关节名）
export LENS_LEFT_CAN_IDS=1,2,3,4,5,6,7

# 打印频率 Hz；0 关闭
export LENS_BRIDGE_PRINT_HZ=30

# MIT（示例）
export LENS_MIT_KP=20
export LENS_MIT_KD=8
export LENS_MIT_VEL=0
export LENS_MIT_TOR=0

# EtherCAT 打印模式可选
export LENS_ETHERCAT_IFNAME=enp12s0
export LENS_ACTUATOR_CONFIG=/path/to/actuator_config.json
export LENS_ACTUATOR_PARAMS=/path/to/actuator_params_config.json
export LENS_ETHERCAT_SEND_FREQUENCY=500
```

`LENS_BRIDGE_PROTOCOL=ethercat` 时，首次还会打印与文档对齐的初始化语义（加载参数、映射、disable / 设 MIT / enable 等），便于与真实 `ActuatorController` 对照。

---

## 7. ROS2 桥接（仿真 → 话题）

以下脚本内建 `Ros2JointCommandPublisher`（或从 `left_arm_ik_slider` 导入同类）：

- `left_arm_ik_slider.py`：7 关节（左臂）  
- `right_arm_ik_slider.py`：7 关节（右臂）  
- `dual_arm_ik_slider.py`：14 关节（双臂）  
- `left_arm_slider_trajectory.py`：7 关节（左臂）

环境变量：

```bash
source /opt/ros/jazzy/setup.bash   # 按实际发行版调整

export LENS_ROS_BRIDGE=1
export LENS_ROS_TOPIC=/joint_command    # 默认
export LENS_ROS_HZ=60                   # 发布上限频率

# 可选：与真机 MIT 标定对齐时可在仿真侧带（若脚本支持读取）
# export LENS_MIT_KP=20
# export LENS_MIT_KD=8
```

消息类型：`sensor_msgs/msg/JointState`，**至少填充 `name` 与 `position`**（弧度）。真机节点应**按关节名**对齐到驱动器，而不是假设数组下标恒等于电机序号。

**联调常见坑**

- **`ROS_DOMAIN_ID`**：发布端与执行端必须一致。  
- **RMW**：跨用户或 `sudo` 时，Fast DDS 共享内存可能导致 discovery 异常；可尝试统一 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（以你环境为准）。  
- **权限**：EtherCAT 与部分 ROS 传输常需一致的网络/权限策略；执行端若用 `sudo`，注意保留 `LD_LIBRARY_PATH` 与 ROS 环境（如 `sudo -E bash -lc '...'`）。

---

## 8. IK / 规划算法要点（简述）

- **3D IK**（`left_arm_ik_slider_3d.py` 等）：位置雅可比 + 阻尼最小二乘 + 多初值 + 限位惩罚。  
- **6D IK**（`left_arm_ik_slider.py` 等）：6×7 任务雅可比；姿态误差用世界系旋转对齐；含奇异与多解处理。  
- **规划**：自实现 RRT-Connect（非 OMPL 依赖）；shortcut 平滑；边碰撞用 `left_arm_plan_collision` 中带自适应步长的插值检测。

---

## 9. 从「克隆仓库」到「联调真机」推荐流程

1. 安装系统依赖：`python3-tk`，以及 ROS 2（若要用桥接）。  
2. `lens_dual_arm_description` 下创建 venv，安装 `requirements.txt`，确认 `import mujoco` 成功。  
3. 本地跑通 `fk_viewer.py` → `left_arm_ik_slider_3d.py` → `left_arm_slider_trajectory.py`，确认无 GUI/模型路径错误。  
4. 需要协议对齐时打开 `LENS_BRIDGE_PROTOCOL`，对照厂商文档看打印帧。  
5. 编译并运行 `lens_dual_arm` 中 `joint_controller_node`（见 `lens_dual_arm/README.md`），配置 `ethercat_control.yaml` 中网口与 JSON 路径。  
6. 终端 A 启动控制器；终端 B `export LENS_ROS_BRIDGE=1` 运行仿真；用 `ros2 topic echo`、`ros2 topic hz` 确认 `/joint_command` 联通。  
7. 若某关节与仿真转向相反，在**真机控制器侧**做关节名级别的符号翻转或重新标定，直至与仿真一致。

---

## 10. 常见问题

| 现象 | 处理方向 |
|------|----------|
| `No module named 'mujoco'` | venv 内 `pip install -r requirements.txt` |
| `No module named 'tkinter'` | `sudo apt install python3-tk` |
| `No module named 'rclpy'` | 先 source ROS；或 venv 加 `--system-site-packages` |
| mesh 找不到 | 改用 `mjcf/mj_lens_dual_arm.xml`；检查相对路径 |
| 真机收不到 `/joint_command` | 检查 DOMAIN_ID、RMW、话题名、QoS；是否一端未订阅外部话题（见上一级 README） |

---

## 11. 文档维护约定

功能或默认参数变更时，请同步更新：

- 本文件（运行方式、环境变量、关节列表）。  
- 上一级 [../README.md](../README.md)（架构与跨包依赖）。

---

## 12. 后续可扩展方向

- 不可达目标提示与投影。  
- YAML/JSON 统一配置 IK 与规划参数。  
- 环境障碍物（当前身体避障主要覆盖躯干/双臂自碰相关几何）。  
- 右臂专用「滑条 3D 轨迹」脚本（若需与左臂 `left_arm_slider_trajectory.py` 对称）。
