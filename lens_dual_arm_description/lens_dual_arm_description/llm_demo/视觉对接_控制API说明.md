# 视觉模块 ↔ 真机控制 对接说明

本文档面向**视觉算法同事**与**真机控制同事**，说明如何通过 HTTP 将视觉侧的抓取/运动意图下发到双臂控制系统。

| 角色 | 职责 |
|------|------|
| 视觉同事 | 相机、检测、定位；将目标位姿与动作序列转换为 **Path IR JSON** 并 POST |
| 控制同事 | 启动真机驱动与 Path IR API；开放网络端口；联调与安全确认 |

---

## 1. 系统架构

```text
┌─────────────────┐     POST JSON (Path IR)      ┌──────────────────────────┐
│  视觉程序/同事   │ ──────────────────────────► │  Path IR API (:8000)      │
│  (检测+标定)     │     http://<IP>:8000/...    │  fastapi_path_ir_api_...  │
└─────────────────┘                              └────────────┬─────────────┘
                                                                │ IK 规划
                                                                ▼
┌─────────────────┐     ROS2 /joint_command      ┌──────────────────────────┐
│  视觉检测流      │  (仅观看，非控制端口)          │  joint_controller_node    │
│  detect_bridge   │     通常 :8080               │  EtherCAT → 真机          │
└─────────────────┘                              └──────────────────────────┘
```

**端口分工（请勿混用）：**

| 端口 | 服务 | 用途 |
|------|------|------|
| **8000** | `fastapi_path_ir_api_server.py` | 视觉 → 控制：**轨迹指令**（Path IR JSON） |
| **8080**（常见） | `detect_bridge.py` | 检测画面 MJPEG、`/detect/status` 等，**不用于控臂** |

---

## 2. 控制侧准备工作（控制同事）

### 2.1 启动顺序

**终端 A — 真机驱动（需 root / EtherCAT）：**

```bash
cd /root/Project/lens_dual_arm_description_cpp/lens_dual_arm_description
sudo -E scripts/run_joint_controller.sh
```

确认日志中出现电机使能成功、`EtherCAT control enabled` 等，且配置中 `accept_external_joint_command: true`。

**终端 B — Path IR HTTP API：**

```bash
cd /root/Project/lens_dual_arm_description_cpp/lens_dual_arm_description
scripts/run_path_ir_api_server.sh
```

默认监听：`http://0.0.0.0:8000`。

### 2.2 本机自检

```bash
curl http://127.0.0.1:8000/health
curl http://127.0.0.1:8000/status
```

返回 `"ok": true` 表示 API 就绪。

### 2.3 对外开放端口

- **同一局域网**：将工控机内网 IP（如 `192.168.1.100`）告知视觉同事，访问 `http://192.168.1.100:8000`。
- **跨公网**：路由器将外网 `8000` 转发到本机；建议使用 VPN 或 IP 白名单，避免控制接口暴露在公网。
- **防火墙**（按需）：`sudo ufw allow 8000/tcp` 或 firewalld 放行 `8000/tcp`。

---

## 3. HTTP 接口说明（视觉同事）

### 3.0 推荐：精简 JSON（自动转成完整 Path IR）

视觉同事只需 POST **少量字段**，服务端自动补全为完整 `ir.json` 同结构再执行。

| 方法 | 路径 |
|------|------|
| `GET` | `/vision/schema` | 查看字段说明与示例 |
| `POST` | `/vision/trajectory` | 精简 JSON → 转换 → 规划 → 执行 |
| `POST` | `/vision/trajectory_manual` | 精简 JSON → 仅规划，再 `POST /confirm` |

**最简报文示例**（`vision_simple_example.json`）：

```json
{
  "command_id": "vision_001",
  "arm": "right",
  "target": [0.48, -0.24, 0.42],
  "duration_s": 4.0,
  "action": "grasp"
}
```

也可用 `x` / `y` / `z` 代替 `target` 数组。固定字段（`safety`、`end_effector`、协议头等）由控制侧默认配置填充，无需同事填写。

```bash
curl -X POST "http://192.168.1.243:8000/vision/trajectory" \
  -H "Content-Type: application/json" \
  -d @llm_demo/vision_simple_example.json
```

控制侧若要改默认值，可编辑 `llm_demo/vision_path_ir_defaults.json`（可选，不存在则用内置默认）。

---

### 3.1 基础信息（完整 Path IR）

| 项目 | 值 |
|------|-----|
| Base URL | `http://<控制工控机IP>:8000` |
| Content-Type | `application/json` |
| 字符编码 | UTF-8 |

### 3.2 接口列表

| 方法 | 路径 | 说明 |
|------|------|------|
| `GET` | `/health` | 健康检查 |
| `GET` | `/status` | 当前服务状态（是否在执行、上次 command_id 等） |
| `POST` | `/trajectory` | **规划 + 立即执行**（阻塞至动作结束） |
| `POST` | `/trajectory_manual` | 仅规划，等待确认 |
| `POST` | `/confirm` | 执行已暂存的轨迹 |
| `POST` | `/cancel` | 取消暂存的手动轨迹 |

**推荐联调流程：**

1. 先用 `GET /health` 确认连通。
2. 首次真机测试使用 `POST /trajectory_manual` + 控制同事现场确认 + `POST /confirm`。
3. 稳定后使用 `POST /trajectory` 一键执行。

### 3.3 请求示例（curl）

```bash
curl -X POST "http://192.168.1.100:8000/trajectory" \
  -H "Content-Type: application/json" \
  -d @grasp_path_ir.json
```

### 3.4 成功响应示例（`/trajectory`）

```json
{
  "ok": true,
  "mode": "auto_execute",
  "command_id": "vision_grasp_001",
  "traj_len": 240,
  "traj_hz": 60.0,
  "duration_s": 4.0,
  "execute_s": 4.1,
  "total_s": 5.2,
  "executed": true
}
```

### 3.5 错误响应

- HTTP `400`，body 含 `detail` 字段（如协议字段缺失、越工作空间、IK 失败等）。
- 客户端请设置足够长的超时（建议 ≥ 120 秒），`/trajectory` 会阻塞到运动结束。

### 3.6 Python 调用示例

```python
import requests

API = "http://192.168.1.100:8000"

with open("grasp_path_ir.json", "r", encoding="utf-8") as f:
    path_ir = __import__("json").load(f)

r = requests.post(f"{API}/trajectory", json=path_ir, timeout=120)
r.raise_for_status()
print(r.json())
```

---

## 4. 数据格式：Path IR v2.0

视觉同事发送的 Body **必须是**完整的 **Path IR v2.0** JSON，`command_type` 固定为 `draw_path`。

详细字段规范见同目录：

- `llm_path_ir_output_spec.md` — 路径命令与坐标规则
- `ir.json` — 可运行的最小示例

### 4.1 顶层结构（全部必填）

```json
{
  "version": "2.0",
  "command_id": "vision_grasp_001",
  "command_type": "draw_path",
  "target": { },
  "frame": { },
  "path": { },
  "motion": { },
  "end_effector": { },
  "safety": { }
}
```

| 字段 | 说明 |
|------|------|
| `version` | 固定 `"2.0"` |
| `command_id` | 每次任务唯一 ID，便于日志追踪 |
| `command_type` | 固定 `"draw_path"` |

### 4.2 `target` — 执行臂

```json
"target": {
  "arm": "right",
  "mode": "ee_pose"
}
```

| 字段 | 取值 |
|------|------|
| `arm` | `"left"` \| `"right"` \| `"both"` |
| `mode` | `"ee_pose"`（推荐）\| `"ee_position"` |

抓取场景通常使用**单臂** + `ee_pose`。

### 4.3 `frame` — 三维空间锚点（手眼标定结果写这里）

```json
"frame": {
  "type": "local_2d_on_3d_plane",
  "unit": "meter",
  "origin": [0.48, -0.24, 0.42],
  "plane_rpy_deg": [0.0, 0.0, 0.0],
  "scale": 1.0
}
```

| 字段 | 说明 |
|------|------|
| `origin` | **机器人基座坐标系**下的锚点 `[x, y, z]`（米）。视觉应将目标点变换到该坐标系后写入 |
| `plane_rpy_deg` | 局部绘图平面相对基座的姿态（度） |
| `type` | 固定 `local_2d_on_3d_plane` |
| `unit` | 固定 `meter` |

**默认参考原点（未标定时可临时使用）：**

| 臂 | `origin` |
|----|----------|
| 左臂 | `[0.48, 0.24, 0.42]` |
| 右臂 | `[0.48, -0.24, 0.42]` |

### 4.4 `path` — 局部平面上的运动路径

```json
"path": {
  "coordinate_mode": "normalized",
  "normalize_box_m": {
    "width": 0.2,
    "height": 0.2
  },
  "commands": [
    { "cmd": "M", "p": [0.0, 0.2] },
    { "cmd": "L", "p": [0.0, -0.1] },
    { "cmd": "M", "p": [0.0, 0.2] }
  ]
}
```

**`coordinate_mode`：**

| 模式 | `p` 的含义 |
|------|------------|
| `normalized` | 归一化坐标，建议范围约 `[-0.5, 0.5]`；实际尺寸由 `normalize_box_m` 的宽高（米）决定 |
| `meter` | 局部平面上的真实米制偏移 |

**`commands` 支持的命令：**

| cmd | 含义 | 主要字段 |
|-----|------|----------|
| `M` | 移动到新起点 | `p`: `[x, y]` |
| `L` | 直线到点 | `p` |
| `Q` | 二阶贝塞尔 | `c`, `p` |
| `C` | 三阶贝塞尔 | `c1`, `c2`, `p` |
| `A` | 圆弧 | `center`, `radius`, `start_deg`, `end_deg`, `clockwise` |
| `Z` | 闭合子路径 | 无 |

规则：**每个子路径必须以 `M` 开头**。

**抓取典型三段折线（示意）：**

```text
M → 预抓取高度（平面上方）
L → 下降到抓取高度
M → 抬起离开
```

### 4.5 `motion` — 时间与采样

```json
"motion": {
  "duration_s": 4.0,
  "repeat": 1,
  "speed_mode": "constant_path_speed",
  "sample_hz": 60,
  "lift_between_subpaths": false,
  "lift_height_m": 0.03
}
```

| 字段 | 说明 |
|------|------|
| `duration_s` | 整段轨迹时长（秒） |
| `sample_hz` | 采样频率（Hz） |
| `speed_mode` | 固定 `"constant_path_speed"` |
| `repeat` | 重复次数；`-1` 表示无限循环（慎用） |
| `lift_between_subpaths` | 多段子路径之间是否在 Z 方向抬高 |

### 4.6 `end_effector` — 末端姿态

```json
"end_effector": {
  "rpy_deg": [0.0, -90.0, 0.0],
  "tool_offset": [0.0, 0.0, 0.0]
}
```

联调前双方需约定抓取时的 `rpy_deg`。`tool_offset` 在当前 demo 中可能不生效，以实际控制日志为准。

### 4.7 `safety` — 工作空间限制

```json
"safety": {
  "workspace_min": [0.2, -0.5, 0.15],
  "workspace_max": [0.8, 0.5, 0.8],
  "max_linear_speed": 0.25,
  "max_acc": 0.8,
  "allow_partial": false
}
```

超出 `workspace_min` / `workspace_max` 可能导致规划被拒绝。

---

## 5. 抓取场景完整示例

将视觉标定后的目标写入 `frame.origin`，用 `path.commands` 描述接近—下降—抬起：

```json
{
  "version": "2.0",
  "command_id": "vision_grasp_001",
  "command_type": "draw_path",
  "target": {
    "arm": "right",
    "mode": "ee_pose"
  },
  "frame": {
    "type": "local_2d_on_3d_plane",
    "unit": "meter",
    "origin": [0.48, -0.24, 0.42],
    "plane_rpy_deg": [0.0, 0.0, 0.0],
    "scale": 1.0
  },
  "path": {
    "coordinate_mode": "normalized",
    "normalize_box_m": { "width": 0.2, "height": 0.2 },
    "commands": [
      { "cmd": "M", "p": [0.0, 0.2] },
      { "cmd": "L", "p": [0.0, -0.1] },
      { "cmd": "M", "p": [0.0, 0.2] }
    ]
  },
  "motion": {
    "duration_s": 4.0,
    "repeat": 1,
    "speed_mode": "constant_path_speed",
    "sample_hz": 60,
    "lift_between_subpaths": false,
    "lift_height_m": 0.03
  },
  "end_effector": {
    "rpy_deg": [0.0, -90.0, 0.0],
    "tool_offset": [0.0, 0.0, 0.0]
  },
  "safety": {
    "workspace_min": [0.2, -0.5, 0.15],
    "workspace_max": [0.8, 0.5, 0.8],
    "max_linear_speed": 0.25,
    "max_acc": 0.8,
    "allow_partial": false
  }
}
```

保存为 `grasp_path_ir.json` 后，用第 3 节 curl 或 Python 示例发送。

---

## 6. 坐标与职责划分（重要）

### 6.1 视觉同事负责

1. 相机标定 + **手眼标定**（相机坐标 → 机器人基座坐标）。
2. 将抓取目标位置写入 `frame.origin`（必要时调整 `plane_rpy_deg`）。
3. 将「接近 / 下降 / 抬起」等动作编码为 `path.commands`（`M` / `L` 等）。
4. 生成合法 Path IR JSON 并 POST 到 8000 端口。

### 6.2 控制同事负责

1. 保证 `joint_controller` 与 Path IR API 持续运行。
2. 网络与防火墙放行 8000。
3. 联调时现场确认安全范围与速度。
4. 夹爪（若有）的开关时序——**当前 Path IR 不包含夹爪字段**（见第 7 节）。

### 6.3 路径会被自动缩放

规划管线会将整条 2D 路径**按比例缩放到约 0.24 m 量级的工作区**（保持形状、改变绝对尺度）。

因此：

- ❌ 不能直接发送相机像素坐标或未经标定的图像坐标。
- ✅ 必须先变换到机器人坐标系，再通过 `frame.origin` + `path.commands` 表达相对运动。

---

## 7. 不支持的数据类型

| 数据类型 | 是否支持 | 说明 |
|----------|----------|------|
| Path IR v2.0 JSON | ✅ | 唯一正式控制输入 |
| 单个 `{x, y, z}` 点 | ❌ | 需包装为完整 Path IR |
| 检测框 `[x1,y1,x2,y2]` | ❌ | 需转换为 Path IR |
| 图片 / 深度图 / base64 | ❌ | 走视觉服务（如 8080），不走 8000 |
| 关节角数组 | ❌ | 由控制侧 IK 计算 |
| 自然语言指令 | ❌ | 需先转为 Path IR（可用 `text_to_path_ir.py` 离线转换） |
| 夹爪开/关指令 | ❌ | **当前 API 仅控制手臂轨迹**；夹爪需另行约定 |

---

## 8. 联调检查清单

### 控制同事

- [ ] `run_joint_controller.sh` 运行中，电机已使能
- [ ] `run_path_ir_api_server.sh` 运行中
- [ ] `curl http://<本机IP>:8000/health` 返回 ok
- [ ] 防火墙 / 路由器已放行 8000
- [ ] 已将正确 IP 告知视觉同事

### 视觉同事

- [ ] `curl http://<控制IP>:8000/health` 通
- [ ] POST Body 为完整 Path IR，Header 为 `application/json`
- [ ] HTTP 客户端 timeout ≥ 120 s
- [ ] 已与控制同事确认坐标系、`rpy_deg`、安全范围

### 双方

- [ ] 首次真机使用小范围、低速度轨迹
- [ ] 优先使用 `/trajectory_manual` + `/confirm` 流程
- [ ] 明确夹爪动作由谁、在何时触发

---

## 9. 相关文件与脚本

| 路径 | 说明 |
|------|------|
| `llm_demo/fastapi_path_ir_api_server.py` | HTTP API 实现 |
| `scripts/run_path_ir_api_server.sh` | 启动 API（默认端口 8000） |
| `scripts/run_joint_controller.sh` | 启动真机驱动 |
| `llm_demo/ir.json` | Path IR 示例 |
| `llm_demo/llm_path_ir_output_spec.md` | Path IR 字段完整规范 |
| `llm_demo/dual_arm_llm_trajectory_demo.py` | 规划与下发逻辑 |

---

## 10. 常见问题

**Q: 发了 JSON 但机械臂不动？**  
A: 检查 `joint_controller` 是否 EtherCAT 使能成功；`ros2 topic info /joint_command` 订阅数是否 ≥ 1；API 日志是否有 IK 或安全错误。

**Q: 返回 400 `OUT_OF_WORKSPACE`？**  
A: 调整 `frame.origin` 或 `safety.workspace_*`，确保目标在允许范围内。

**Q: 视觉只有检测框，没有 Path IR？**  
A: 需在视觉程序中增加「框/深度 → 机器人坐标 → Path IR」转换层，或与控制同事约定中间格式后由控制侧转换。

**Q: 8000 和 8080 用哪个？**  
A: **控臂用 8000**；看检测画面用 8080（`detect_bridge`）。

---

## 11. 修订记录

| 日期 | 说明 |
|------|------|
| 2026-05-29 | 初版：视觉—控制 HTTP 对接与 Path IR 数据格式 |
