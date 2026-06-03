# 机械臂通用绘图轨迹协议控制端说明书

## 1. 文档目的

本文档面向机械臂仿真控制程序、ROS2 控制节点、MuJoCo 控制循环、真实机械臂执行层。

目标是定义一种通用的轨迹中间表示：

```text
Path IR = Path Intermediate Representation
```

它用于把任意自然语言图形描述转换成机械臂可执行的末端轨迹。

控制端只需要解析统一的 `draw_path` JSON 指令，不需要理解“梯形”“人字形”“闪电形”等自然语言语义。

---

## 2. 总体架构

```text
用户自然语言
  ↓
大模型
  ↓
Path IR JSON
  ↓
JSON Schema 校验
  ↓
Path IR 轨迹采样器
  ↓
2D 局部路径点
  ↓
3D 平面映射
  ↓
target_pos / target_quat
  ↓
IK 求解
  ↓
MuJoCo / ROS2 / 实机控制
```

控制端的职责是：

```text
1. 接收 JSON
2. 校验 JSON 合法性
3. 将 M/L/Q/C/A/Z 路径命令采样成点
4. 将局部 2D 点映射到机械臂世界坐标
5. 做工作空间、速度、加速度、安全检查
6. 调用 IK 求解器
7. 输出关节目标或底层控制量
8. 返回执行状态
```

控制端不负责：

```text
1. 理解自然语言
2. 猜测图形形状
3. 让大模型直接输出关节角
4. 让大模型直接控制电机
```

---

## 3. 顶层 JSON 格式

```json
{
  "version": "2.0",
  "command_id": "cmd_001",
  "command_type": "draw_path",

  "target": {
    "arm": "left",
    "mode": "ee_pose"
  },

  "frame": {
    "type": "local_2d_on_3d_plane",
    "unit": "meter",
    "origin": [0.48, 0.24, 0.42],
    "plane_rpy_deg": [0.0, 0.0, 0.0],
    "scale": 1.0
  },

  "path": {
    "coordinate_mode": "normalized",
    "normalize_box_m": {
      "width": 0.24,
      "height": 0.18
    },
    "commands": [
      {"cmd": "M", "p": [-0.5, -0.5]},
      {"cmd": "L", "p": [0.5, -0.5]},
      {"cmd": "L", "p": [0.0, 0.5]},
      {"cmd": "Z"}
    ]
  },

  "motion": {
    "duration_s": 5.0,
    "repeat": 1,
    "speed_mode": "constant_path_speed",
    "sample_hz": 120,
    "lift_between_subpaths": false,
    "lift_height_m": 0.03
  },

  "end_effector": {
    "rpy_deg": [0.0, -90.0, 0.0],
    "tool_offset": [0.0, 0.0, -0.08]
  },

  "safety": {
    "workspace_min": [0.20, -0.50, 0.15],
    "workspace_max": [0.80, 0.50, 0.80],
    "max_linear_speed": 0.25,
    "max_acc": 0.8,
    "allow_partial": false
  }
}
```

---

## 4. 顶层字段说明

| 字段 | 类型 | 必须 | 含义 |
|---|---|---:|---|
| `version` | string | 是 | 协议版本，当前为 `"2.0"` |
| `command_id` | string | 是 | 指令唯一 ID，用于追踪、更新、取消 |
| `command_type` | string | 是 | 当前固定为 `"draw_path"` |
| `target` | object | 是 | 控制对象 |
| `frame` | object | 是 | 局部 2D 路径到 3D 空间的映射方式 |
| `path` | object | 是 | 通用路径指令 |
| `motion` | object | 是 | 时间、采样频率、重复次数等 |
| `end_effector` | object | 是 | 末端姿态与工具偏移 |
| `safety` | object | 是 | 工作空间和速度安全限制 |

---

## 5. target 字段

```json
"target": {
  "arm": "left",
  "mode": "ee_pose"
}
```

| 字段 | 可选值 | 含义 |
|---|---|---|
| `arm` | `left` / `right` / `both` | 控制左臂、右臂或双臂 |
| `mode` | `ee_position` / `ee_pose` | 只控制末端位置，或控制末端位置与姿态 |

建议：

```text
仿真阶段：默认使用 ee_pose
调试 IK：可以先使用 ee_position
真实机械臂：优先使用 ee_pose
```

---

## 6. frame 字段

```json
"frame": {
  "type": "local_2d_on_3d_plane",
  "unit": "meter",
  "origin": [0.48, 0.24, 0.42],
  "plane_rpy_deg": [0.0, 0.0, 0.0],
  "scale": 1.0
}
```

### 6.1 含义

`frame` 定义如何把二维图形路径放到三维空间中。

二维局部点：

```text
p_local_2d = [x, y]
```

扩展为：

```text
p_local_3d = [x, y, 0]
```

再映射到世界坐标：

```text
p_world = origin + R(plane_rpy_deg) @ p_local_3d * scale
```

### 6.2 字段说明

| 字段 | 类型 | 含义 |
|---|---|---|
| `type` | string | 当前固定为 `local_2d_on_3d_plane` |
| `unit` | string | 当前固定为 `meter` |
| `origin` | array[3] | 图形中心在世界坐标系中的位置，单位 m |
| `plane_rpy_deg` | array[3] | 图形平面姿态，单位 degree |
| `scale` | number | 额外缩放系数，默认 1.0 |

---

## 7. path 字段

```json
"path": {
  "coordinate_mode": "normalized",
  "normalize_box_m": {
    "width": 0.24,
    "height": 0.18
  },
  "commands": [
    {"cmd": "M", "p": [-0.5, -0.5]},
    {"cmd": "L", "p": [0.5, -0.5]},
    {"cmd": "L", "p": [0.0, 0.5]},
    {"cmd": "Z"}
  ]
}
```

### 7.1 coordinate_mode

| 值 | 含义 |
|---|---|
| `normalized` | 坐标为归一化坐标，推荐大模型输出 |
| `meter` | 坐标已经是米制坐标 |

推荐控制端优先支持：

```text
normalized
```

因为它便于图形缩放、平移、复用。

---

### 7.2 normalized 坐标规则

当：

```json
"coordinate_mode": "normalized"
```

时，路径点一般位于：

```text
x ∈ [-0.5, 0.5]
y ∈ [-0.5, 0.5]
```

映射规则：

```text
x_meter = x_normalized * width
y_meter = y_normalized * height
```

例如：

```json
"normalize_box_m": {
  "width": 0.24,
  "height": 0.18
}
```

则：

```text
[0.5, 0.5] → [0.12m, 0.09m]
[-0.5, -0.5] → [-0.12m, -0.09m]
```

---

## 8. Path 命令定义

Path IR 支持以下命令：

```text
M: MoveTo
L: LineTo
Q: Quadratic Bezier
C: Cubic Bezier
A: Arc
Z: ClosePath
```

---

### 8.1 M：MoveTo

```json
{"cmd": "M", "p": [0.0, 0.0]}
```

作用：

```text
移动到新起点。
```

特点：

```text
M 不表示绘制线段。
M 可用于开启新的子路径。
```

---

### 8.2 L：LineTo

```json
{"cmd": "L", "p": [0.5, 0.0]}
```

作用：

```text
从当前点直线绘制到目标点。
```

---

### 8.3 Q：二阶贝塞尔曲线

```json
{
  "cmd": "Q",
  "c": [0.0, 0.5],
  "p": [0.5, 0.0]
}
```

数学形式：

```text
B(t) = (1 - t)^2 P0 + 2(1 - t)t C + t^2 P1
t ∈ [0, 1]
```

适合：

```text
简单弧线
柔和转角
笑脸嘴巴
波浪形
```

---

### 8.4 C：三阶贝塞尔曲线

```json
{
  "cmd": "C",
  "c1": [0.0, 0.5],
  "c2": [0.5, 0.5],
  "p": [0.5, 0.0]
}
```

数学形式：

```text
B(t) = (1 - t)^3 P0
     + 3(1 - t)^2 t C1
     + 3(1 - t)t^2 C2
     + t^3 P1
```

适合：

```text
复杂曲线
手写轨迹
自由曲线
```

---

### 8.5 A：圆弧

```json
{
  "cmd": "A",
  "center": [0.0, 0.0],
  "radius": 0.5,
  "start_deg": 0.0,
  "end_deg": 180.0,
  "clockwise": false
}
```

字段：

| 字段 | 含义 |
|---|---|
| `center` | 圆心 |
| `radius` | 半径 |
| `start_deg` | 起始角度，单位 degree |
| `end_deg` | 结束角度，单位 degree |
| `clockwise` | 是否顺时针 |

注意：

```text
当 coordinate_mode = normalized 时，radius 也是归一化半径。
控制端应将 radius 乘以 min(width, height)。
```

---

### 8.6 Z：ClosePath

```json
{"cmd": "Z"}
```

作用：

```text
从当前点直线回到当前子路径起点。
```

---

## 9. motion 字段

```json
"motion": {
  "duration_s": 5.0,
  "repeat": 1,
  "speed_mode": "constant_path_speed",
  "sample_hz": 120,
  "lift_between_subpaths": false,
  "lift_height_m": 0.03
}
```

| 字段 | 类型 | 含义 |
|---|---|---|
| `duration_s` | number | 一次完整轨迹执行时间 |
| `repeat` | integer | 重复次数，`-1` 表示无限循环 |
| `speed_mode` | string | 推荐固定为 `constant_path_speed` |
| `sample_hz` | number | 轨迹采样频率 |
| `lift_between_subpaths` | bool | 多笔画之间是否抬笔 |
| `lift_height_m` | number | 抬笔高度，单位 m |

### 9.1 speed_mode

推荐控制端先只实现：

```text
constant_path_speed
```

即按照路径长度等距采样，保证长边、短边的末端线速度尽量一致。

---

## 10. end_effector 字段

```json
"end_effector": {
  "rpy_deg": [0.0, -90.0, 0.0],
  "tool_offset": [0.0, 0.0, -0.08]
}
```

| 字段 | 含义 |
|---|---|
| `rpy_deg` | 末端固定姿态，单位 degree |
| `tool_offset` | 工具尖端相对于末端 body 的局部偏移 |

示例：

```text
机械臂末端直接画空中轨迹：tool_offset 可为 [0, 0, 0]
夹持笔尖：tool_offset 应设置为笔尖相对末端法兰的位置
```

---

## 11. safety 字段

```json
"safety": {
  "workspace_min": [0.20, -0.50, 0.15],
  "workspace_max": [0.80, 0.50, 0.80],
  "max_linear_speed": 0.25,
  "max_acc": 0.8,
  "allow_partial": false
}
```

控制端必须检查：

```text
1. 所有轨迹点是否在 workspace 内
2. 轨迹速度是否超过 max_linear_speed
3. 轨迹加速度是否超过 max_acc
4. IK 是否可解
5. 是否接近奇异位形
6. 是否发生潜在碰撞
```

建议：

```text
仿真阶段可以打印 warning
实机阶段必须拒绝执行危险轨迹
```

---

## 12. 控制端推荐执行流程

```python
def execute_draw_path_command(command: dict):
    # 1. 校验 command_type
    assert command["command_type"] == "draw_path"

    # 2. 校验 JSON 字段完整性
    validate_command(command)

    # 3. 将 Path IR 采样为 2D 点
    points_2d, pen_down = sample_path_2d(command["path"], command["motion"])

    # 4. 2D 点映射为 3D 世界点
    points_3d = map_2d_points_to_world(points_2d, command["frame"])

    # 5. 安全检查
    check_workspace(points_3d, command["safety"])
    check_speed(points_3d, command["motion"], command["safety"])
    check_acc(points_3d, command["motion"], command["safety"])

    # 6. 运行时按时间索引取 target_pos
    runtime.load(points_3d, pen_down, command)

    # 7. 主循环中调用 IK
    while running:
        target_pos, is_pen_down, finished = runtime.get_target(time.time())
        q_sol = solve_ik(target_pos, command["end_effector"])
        send_to_robot(q_sol)
```

---

## 13. 推荐状态反馈格式

控制端应向上层返回状态。

```json
{
  "version": "2.0",
  "command_id": "cmd_001",
  "state": "running",
  "progress": 0.42,
  "arm": "left",
  "target_pos": [0.51, 0.25, 0.42],
  "actual_pos": [0.508, 0.249, 0.421],
  "position_error": 0.003,
  "ik_status": "ok",
  "ik_condition": 12.5,
  "message": "trajectory running"
}
```

### 13.1 state 枚举

```text
pending
accepted
running
paused
finished
stopped
rejected
failed
```

### 13.2 error_type 枚举

```text
INVALID_JSON
UNSUPPORTED_VERSION
UNSUPPORTED_COMMAND
UNSUPPORTED_PATH_CMD
OUT_OF_WORKSPACE
SPEED_TOO_HIGH
ACC_TOO_HIGH
IK_FAILED
NEAR_SINGULARITY
COLLISION_RISK
COMMAND_INTERRUPTED
```

---

## 14. 和当前 MuJoCo 代码的对接建议

当前代码已经具备以下能力：

```text
1. 根据相位 phase 生成目标点
2. 支持左臂圆形轨迹
3. 支持右臂方形轨迹
4. 支持图形平面姿态 shape_rpy_deg
5. 支持末端固定姿态 ee_rpy_deg
6. 支持 IK 求解
7. 支持 MuJoCo 可视化目标轨迹和实际轨迹
8. 支持 ROS2 joint command 发布接口
```

改造重点：

```text
把原来写死的 circle/square 目标点生成逻辑替换为 Path IR runtime。
```

原始逻辑：

```python
phase = (elapsed / period_s) % 1.0
left_target_pos = left_center + Rshape @ left_offset_local
right_target_pos = right_center + Rshape @ right_offset_local
```

改造后：

```python
target_pos, pen_down, finished = trajectory_runtime.get_target(time.time())
```

然后继续沿用原来的：

```text
target_pos → IK → data.ctrl → viewer.sync()
```

---

## 15. 推荐最小实现优先级

### 第一阶段：只支持折线

```text
M
L
Z
```

可以完成：

```text
三角形
矩形
梯形
人字形
箭头
闪电
房子轮廓
字母 A
```

### 第二阶段：支持曲线

```text
Q
C
A
```

可以完成：

```text
圆
椭圆
笑脸
波浪线
心形近似
手写曲线
```

### 第三阶段：支持真实绘图设备

```text
pen_down
lift_between_subpaths
lift_height_m
接触力控制
笔尖标定
```

---

## 16. 控制端必须坚持的原则

```text
1. 大模型只输出路径，不输出关节角。
2. 所有关节角都由 IK 求解器计算。
3. 所有轨迹都必须经过安全检查。
4. 所有坐标都必须有单位。
5. 所有未知字段默认忽略，但未知 cmd 必须拒绝。
6. 实机执行前必须先仿真验证。
7. Path IR 是唯一入口，不允许自然语言直接进入控制层。
```
