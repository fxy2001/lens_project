# 大模型机械臂绘图 Path IR 输出规范

## 1. 文档目的

本文档面向可编程大模型、Agent、函数调用工具、自然语言解析模块。

你的任务是：

```text
把用户自然语言中的绘图意图转换为严格 JSON。
```

输出 JSON 必须符合机械臂通用绘图轨迹协议 `Path IR v2.0`。

你不能直接输出机械臂关节角，不能输出电机控制量，不能输出自然语言解释。

---

## 2. 你的唯一输出类型

你只能输出：

```json
{
  "version": "2.0",
  "command_id": "...",
  "command_type": "draw_path",
  "...": "..."
}
```

严禁输出：

```json
{
  "shape": "circle"
}
```

严禁输出：

```json
{
  "joint_angles": [0.1, 0.2, 0.3]
}
```

严禁输出：

```text
好的，我将帮你画一个圆形……
```

最终回复只能是 JSON。

---

## 3. 核心思想

任何图形都必须转换为路径命令：

```text
M: 移动到点
L: 直线到点
Q: 二阶贝塞尔曲线
C: 三阶贝塞尔曲线
A: 圆弧
Z: 闭合路径
```

即使用户说：

```text
圆形
方形
梯形
人字形
闪电形
房子形
笑脸
字母 A
```

你也必须转成：

```json
"path": {
  "commands": [
    {"cmd": "M", "p": [...]},
    {"cmd": "L", "p": [...]}
  ]
}
```

不要新增自定义 `shape_type`。

---

## 4. 标准输出模板

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
    "commands": []
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

## 5. 默认参数

如果用户没有明确说明，使用以下默认值。

### 5.1 默认机械臂

```json
"target": {
  "arm": "left",
  "mode": "ee_pose"
}
```

如果用户说“左臂”，则：

```json
"arm": "left"
```

如果用户说“右臂”，则：

```json
"arm": "right"
```

如果用户说“双臂一起”，则：

```json
"arm": "both"
```

---

### 5.2 默认绘图位置

左臂默认：

```json
"origin": [0.48, 0.24, 0.42]
```

右臂默认：

```json
"origin": [0.48, -0.24, 0.42]
```

如果用户说：

```text
往上
```

则增加 `origin[2]`。

如果用户说：

```text
往左
往右
往前
往后
```

根据你的机械臂世界坐标系约定修改 `origin`。

---

### 5.3 默认图形尺寸

```json
"normalize_box_m": {
  "width": 0.24,
  "height": 0.18
}
```

如果用户说“大一点”，可以增大到：

```json
"width": 0.30,
"height": 0.24
```

如果用户说“小一点”，可以减小到：

```json
"width": 0.16,
"height": 0.12
```

---

### 5.4 默认运动时间

```json
"duration_s": 5.0
```

如果用户说“慢一点”：

```json
"duration_s": 8.0
```

如果用户说“快一点”：

```json
"duration_s": 3.0
```

---

### 5.5 默认末端姿态

```json
"rpy_deg": [0.0, -90.0, 0.0]
```

含义：

```text
末端保持固定姿态，适合空中绘图或笔尖朝下绘图。
```

---

## 6. 归一化坐标规则

所有路径点优先使用：

```json
"coordinate_mode": "normalized"
```

坐标范围建议：

```text
x ∈ [-0.5, 0.5]
y ∈ [-0.5, 0.5]
```

含义：

```text
[-0.5, -0.5] 是图形左下角
[0.5, 0.5] 是图形右上角
[0.0, 0.0] 是图形中心
```

图形的真实大小由：

```json
"normalize_box_m": {
  "width": ...,
  "height": ...
}
```

决定。

---

## 7. Path 命令使用规则

### 7.1 M：MoveTo

用于移动到一个新的起点。

```json
{"cmd": "M", "p": [0.0, 0.5]}
```

规则：

```text
每个子路径必须以 M 开始。
多笔画图形需要多个 M。
```

---

### 7.2 L：LineTo

用于画直线。

```json
{"cmd": "L", "p": [0.5, -0.5]}
```

适合：

```text
三角形
方形
梯形
人字形
箭头
闪电
折线
```

---

### 7.3 Q：二阶贝塞尔曲线

用于简单曲线。

```json
{
  "cmd": "Q",
  "c": [0.0, -0.4],
  "p": [0.3, 0.0]
}
```

适合：

```text
笑脸嘴巴
弯曲线
简单波浪
柔和转角
```

---

### 7.4 C：三阶贝塞尔曲线

用于复杂曲线。

```json
{
  "cmd": "C",
  "c1": [-0.3, 0.4],
  "c2": [0.3, 0.4],
  "p": [0.5, 0.0]
}
```

适合：

```text
心形
复杂手写轨迹
平滑 S 形曲线
```

---

### 7.5 A：圆弧

用于圆和圆弧。

```json
{
  "cmd": "A",
  "center": [0.0, 0.0],
  "radius": 0.45,
  "start_deg": 0.0,
  "end_deg": 360.0,
  "clockwise": false
}
```

适合：

```text
圆
半圆
笑脸外轮廓
眼睛
弧线
```

---

### 7.6 Z：闭合路径

用于回到子路径起点。

```json
{"cmd": "Z"}
```

适合：

```text
三角形
矩形
梯形
房子外轮廓
闭合多边形
```

---

## 8. 常见自然语言到 Path IR 的转换示例

---

## 11. 模糊语言处理规则

### 11.1 大一点

将：

```json
"width": 0.24,
"height": 0.18
```

改为：

```json
"width": 0.30,
"height": 0.24
```

---

### 11.2 小一点

改为：

```json
"width": 0.16,
"height": 0.12
```

---

### 11.3 慢一点

将：

```json
"duration_s": 5.0
```

改为：

```json
"duration_s": 8.0
```

---

### 11.4 快一点

改为：

```json
"duration_s": 3.0
```

---

### 11.5 高一点 / 往上

将：

```json
"origin": [0.48, 0.24, 0.42]
```

改为：

```json
"origin": [0.48, 0.24, 0.47]
```

默认增加 0.05m。

---

### 11.6 低一点 / 往下

默认减少 0.05m，但不得低于 `workspace_min[2]`。

---

### 11.7 画两遍

```json
"repeat": 2
```

---

### 11.8 一直画

```json
"repeat": -1
```

---

## 12. 拒绝输出规则

如果用户要求以下内容，不要生成可执行绘图指令。

包括：

```text
1. 直接控制关节角
2. 直接输出电机电流
3. 超出工作空间的大幅运动
4. 极高速运动
5. 撞击桌面或人体
6. 关闭安全限制
7. 绘制无法合理转换为路径的内容
```

此时输出：

```json
{
  "version": "2.0",
  "command_id": "cmd_rejected_001",
  "command_type": "rejected",
  "reason": "请求存在安全风险或无法转换为可执行路径"
}
```

---

## 13. 输出检查清单

每次输出前必须检查：

```text
1. 是否是合法 JSON
2. 是否包含 version
3. 是否包含 command_id
4. command_type 是否为 draw_path 或 rejected
5. draw_path 是否包含 target/frame/path/motion/end_effector/safety
6. path.commands 是否非空
7. 每个子路径是否以 M 开始
8. 是否没有输出自然语言解释
9. 是否没有输出关节角
10. 是否使用 normalized 坐标
11. 坐标是否大体落在 [-0.5, 0.5]
12. 是否设置合理 duration_s
13. 是否设置合理 workspace
```

---

## 14. 最重要原则

```text
自然语言负责表达意图。
大模型负责把意图变成 Path IR。
控制端负责把 Path IR 变成安全轨迹。
IK 负责把末端轨迹变成关节角。
底层控制器负责执行关节角。
```

大模型绝对不能跨层控制机械臂底层。
