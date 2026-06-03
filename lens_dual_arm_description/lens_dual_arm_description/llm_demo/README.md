# Path IR 轨迹演示

**视觉模块对接真机控制**：见 [`视觉对接_控制API说明.md`](视觉对接_控制API说明.md)（HTTP 8000、Path IR 数据格式、联调清单）。

这个目录用于接收上游下发的 Path IR（`draw_path` JSON），并执行：

1. 协议校验与路径采样  
2. 严格缩放到固定绘制范围（继承 `dual_arm_ik_draw_circle_square.py` 的平面/范围）  
2. 在 MuJoCo 中预览轨迹  
3. 人工确认  
4. 通过 ROS2 `JointState` 下发真机

## 脚本

- `dual_arm_llm_trajectory_demo.py`

## 快速运行

在 `lens_dual_arm_description/` 或 `llm_demo/` 目录下，**直接运行即默认快速模式**（约 25 个 IK 点 + warm-start + 缓存，目标 10s 内完成）：

```bash
cd llm_demo
export LENS_ROS_BRIDGE=1   # 真机下发时必设
source /opt/ros/humble/setup.bash
python3 dual_arm_llm_trajectory_demo.py
```

同目录存在 `ir.json` 时会自动加载。无显示器（SSH）时自动跳过预览直接下发。

恢复旧版慢速高精度规划：`export LENS_SLOW_PLAN=1`

或使用一键脚本：`../scripts/run_llm_trajectory_fast.sh`

### 推荐：直接提供 Path IR JSON 文件

```bash
export LENS_PATH_IR_FILE=/abs/path/to/your_draw_path.json
python3 llm_demo/dual_arm_llm_trajectory_demo.py
```

### 或：通过环境变量直接传 JSON

```bash
export LENS_PATH_IR_JSON='{"version":"2.0","command_id":"cmd_001","command_type":"draw_path", ... }'
python3 llm_demo/dual_arm_llm_trajectory_demo.py
```

未提供 Path IR 时，脚本会使用一个内置 fallback 示例（三角形）。

## 真机下发配置

默认会先 MuJoCo 预览，并在弹窗中确认后再下发。

**跳过预览与确认，规划完成后直接下发真机**（适合无显示器 / SSH）：

```bash
/root/Project/lens_dual_arm_description_cpp/lens_dual_arm_description/scripts/run_llm_trajectory_fast.sh
```

或手动：

```bash
export LENS_MAX_TOTAL_S=10
export LENS_FAST_EXECUTE=1
export LENS_AUTO_EXECUTE=1
export LENS_PLAN_MAX_POINTS=25
export LENS_IK_MAX_ITERS=12
export LENS_IK_WARM_START=1
export LENS_ROS_BRIDGE=1
python3 llm_demo/dual_arm_llm_trajectory_demo.py
```

**关于 BPU**：本机 BPU 用于 `/root/tong/fast_api` 的 YOLO 视觉（`detect_bridge.py`），**不能加速 MuJoCo 数值 IK**。快速模式通过降采样 + warm-start IK + 轨迹缓存实现；跑机械臂时建议暂停 `detect_bridge` 释放 CPU。

`LENS_SKIP_PREVIEW=1` 与 `LENS_AUTO_EXECUTE=1` 等价。

**真机侧必须先启动并重新编译后的 `joint_controller_node`**（已支持订阅外部 `/joint_command`）：

```bash
# 终端 A
cd /path/to/lens_dual_arm_description
scripts/build_joint_controller.sh
sudo -E scripts/run_joint_controller.sh
```

联调验收：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic info /joint_command -v   # Subscription count 应 >= 1（lens_arm_controller_node）
```

只有在 `LENS_ROS_BRIDGE=1` 且控制器 `accept_external_joint_command:=true` 时才会真正驱动电机。

### 外部 HTTP API（视觉 / 其它模块）

```bash
# 终端 A: sudo -E scripts/run_joint_controller.sh
# 终端 B:
scripts/run_path_ir_api_server.sh
# 视觉精简 JSON（推荐）:
#   POST http://<IP>:8000/vision/trajectory
#   Body: llm_demo/vision_simple_example.json
# 完整 Path IR:
#   POST http://<IP>:8000/trajectory
```

详见 [`视觉对接_控制API说明.md`](视觉对接_控制API说明.md)。转换逻辑：`vision_simple_to_path_ir.py`。

## 主要环境变量

- `LENS_MAX_TOTAL_S`：总耗时预算（默认 10，配合 `LENS_FAST_EXECUTE`）
- `LENS_FAST_EXECUTE` / `LENS_PLAN_MAX_POINTS` / `LENS_IK_MAX_ITERS` / `LENS_IK_WARM_START`：快速 IK 规划
- `LENS_PLAN_CACHE` / `LENS_PATH_IR_PLAN_CACHE_DIR`：轨迹磁盘缓存（第二次同命令近瞬时）
- `LENS_AUTO_EXECUTE` / `LENS_SKIP_PREVIEW`：跳过仿真预览与确认，直接下发真机
- `LENS_PATH_IR_FILE`：Path IR 文件路径（优先）
- `LENS_PATH_IR_JSON`：Path IR JSON 字符串（次优先）
- `LENS_DUAL_ARM_MJCF`：模型路径（不设则用默认 MJCF）
- `LENS_DEMO_LEFT_X/Y/Z`、`LENS_DEMO_RIGHT_X/Y/Z`：绘制平面原点（与旧演示脚本一致）
- `LENS_DEMO_SHAPE_ROLL_DEG/PITCH_DEG/YAW_DEG`：绘制平面姿态
- `LENS_DEMO_SQUARE_HALF`：严格绘制范围半边（最终范围是 `2*half x 2*half`）
- `LENS_ROS_BRIDGE` / `LENS_ROS_TOPIC` / `LENS_ROS_HZ`：真机发布控制
