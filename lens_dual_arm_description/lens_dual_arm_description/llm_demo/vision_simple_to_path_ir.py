"""
Convert a minimal vision-side JSON payload into full Path IR v2.0 (draw_path).

Colleague only sends variables (arm, target xyz, optional motion offsets).
Fixed fields match project defaults (same role as ir.json constants).
"""
from __future__ import annotations

import copy
import json
import time
from pathlib import Path
from typing import Any

# Defaults aligned with llm_demo/ir.json / 视觉对接 template (control side owns these).
_PATH_IR_DEFAULTS: dict[str, Any] = {
    "version": "2.0",
    "command_type": "draw_path",
    "target": {"arm": "right", "mode": "ee_pose"},
    "frame": {
        "type": "local_2d_on_3d_plane",
        "unit": "meter",
        "origin": [0.48, -0.24, 0.42],
        "plane_rpy_deg": [90.0, 0.0, 0.0],
        "scale": 1.0,
    },
    "path": {
        "coordinate_mode": "normalized",
        "normalize_box_m": {"width": 0.2, "height": 0.2},
        "commands": [
            {"cmd": "M", "p": [0.0, 0.15]},
            {"cmd": "L", "p": [0.0, -0.05]},
            {"cmd": "M", "p": [0.0, 0.15]},
        ],
    },
    "motion": {
        "duration_s": 4.0,
        "repeat": 1,
        "speed_mode": "constant_path_speed",
        "sample_hz": 60,
        "lift_between_subpaths": False,
        "lift_height_m": 0.03,
    },
    "end_effector": {
        "rpy_deg": [0.0, 180.0, 0.0],
        "tool_offset": [0.0, 0.0, -0.08],
        "lock_perpendicular_to_xy": True,
        "lock_perpendicular_to_xz": False,
        "lock_perpendicular_to_yz": False,
    },
    "safety": {
        "workspace_min": [0.2, -0.5, 0.15],
        "workspace_max": [0.8, 0.5, 0.8],
        "max_linear_speed": 0.25,
        "max_acc": 0.8,
        "allow_partial": False,
    },
}

# Preset path shapes in normalized local plane (center = target on plane).
_ACTION_PRESETS: dict[str, list[dict[str, Any]]] = {
    "grasp": [
        {"cmd": "M", "p": [0.0, 0.15]},
        {"cmd": "L", "p": [0.0, -0.05]},
        {"cmd": "M", "p": [0.0, 0.15]},
    ],
    "touch": [
        {"cmd": "M", "p": [0.0, 0.1]},
        {"cmd": "L", "p": [0.0, 0.0]},
    ],
    # Single waypoint at frame.origin (normalized 0,0) — move EE tip to target.
    "goto": [
        {"cmd": "M", "p": [0.0, 0.0]},
    ],
    "line": [
        {"cmd": "M", "p": [0.0, -0.5]},
        {"cmd": "L", "p": [0.0, 0.5]},
    ],
}

_THIS_DIR = Path(__file__).resolve().parent
_DEFAULTS_FILE = _THIS_DIR / "vision_path_ir_defaults.json"


def load_defaults() -> dict[str, Any]:
    """Optional override file for control-side fixed fields."""
    if _DEFAULTS_FILE.is_file():
        with _DEFAULTS_FILE.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            base = copy.deepcopy(_PATH_IR_DEFAULTS)
            _deep_merge(base, data)
            return base
    return copy.deepcopy(_PATH_IR_DEFAULTS)


def _deep_merge(dst: dict[str, Any], src: dict[str, Any]) -> None:
    for k, v in src.items():
        if k in dst and isinstance(dst[k], dict) and isinstance(v, dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = copy.deepcopy(v)


def _parse_target(body: dict[str, Any]) -> list[float]:
    if "target" in body:
        t = body["target"]
        if not isinstance(t, (list, tuple)) or len(t) != 3:
            raise ValueError("target must be [x, y, z] in meters (robot base frame)")
        return [float(t[0]), float(t[1]), float(t[2])]
    for key in ("x", "y", "z"):
        if key not in body:
            raise ValueError("need target [x,y,z] or fields x, y, z (meters)")
    return [float(body["x"]), float(body["y"]), float(body["z"])]


def _parse_commands(body: dict[str, Any]) -> list[dict[str, Any]]:
    action = str(body.get("action", "grasp")).strip().lower()
    if action in _ACTION_PRESETS:
        return copy.deepcopy(_ACTION_PRESETS[action])
    if action == "custom" and "commands" in body:
        cmds = body["commands"]
        if not isinstance(cmds, list) or not cmds:
            raise ValueError("action=custom requires non-empty commands list")
        return copy.deepcopy(cmds)
    raise ValueError(f"unknown action={action!r}; use grasp|touch|goto|line|custom")


def vision_simple_to_path_ir(body: dict[str, Any]) -> dict[str, Any]:
    """
    Minimal vision JSON -> full Path IR.

    Required:
      - arm: "left" | "right"
      - target: [x,y,z]  OR  x, y, z  (meters, robot base)

    Optional:
      - command_id
      - duration_s
      - action: grasp | touch | goto | line | custom (with commands)
      - approach, grasp_down: normalized path offsets (override preset Y)
      - rpy_deg: [roll, pitch, yaw] degrees
    """
    if not isinstance(body, dict):
        raise ValueError("body must be a JSON object")

    arm = str(body.get("arm", "")).strip().lower()
    if arm not in ("left", "right"):
        raise ValueError('arm is required: "left" or "right"')

    origin = _parse_target(body)
    commands = _parse_commands(body)

    if "approach" in body or "grasp_down" in body:
        approach = float(body.get("approach", 0.15))
        grasp_down = float(body.get("grasp_down", -0.05))
        commands = [
            {"cmd": "M", "p": [0.0, approach]},
            {"cmd": "L", "p": [0.0, grasp_down]},
            {"cmd": "M", "p": [0.0, approach]},
        ]

    out = load_defaults()
    out["command_id"] = str(body.get("command_id") or f"vision_{int(time.time() * 1000)}")
    out["target"]["arm"] = arm
    out["frame"]["origin"] = origin

    if "duration_s" in body:
        out["motion"]["duration_s"] = float(body["duration_s"])
    if "sample_hz" in body:
        out["motion"]["sample_hz"] = int(body["sample_hz"])
    if "rpy_deg" in body:
        rpy = body["rpy_deg"]
        if not isinstance(rpy, (list, tuple)) or len(rpy) != 3:
            raise ValueError("rpy_deg must be [roll, pitch, yaw]")
        out["end_effector"]["rpy_deg"] = [float(rpy[0]), float(rpy[1]), float(rpy[2])]
    if "tool_offset" in body:
        toff = body["tool_offset"]
        if not isinstance(toff, (list, tuple)) or len(toff) != 3:
            raise ValueError("tool_offset must be [x, y, z] meters in EE frame")
        out["end_effector"]["tool_offset"] = [float(toff[0]), float(toff[1]), float(toff[2])]
    if "plane_rpy_deg" in body:
        prpy = body["plane_rpy_deg"]
        if not isinstance(prpy, (list, tuple)) or len(prpy) != 3:
            raise ValueError("plane_rpy_deg must be [roll, pitch, yaw]")
        out["frame"]["plane_rpy_deg"] = [float(prpy[0]), float(prpy[1]), float(prpy[2])]
    if "box_size" in body:
        s = float(body["box_size"])
        out["path"]["normalize_box_m"] = {"width": s, "height": s}

    out["path"]["commands"] = commands
    return out


def vision_simple_schema() -> dict[str, Any]:
    """Document minimal payload for GET /vision/schema."""
    return {
        "description": "Minimal vision JSON; server expands to Path IR v2.0 automatically.",
        "endpoint_auto": "POST /vision/trajectory",
        "endpoint_manual": "POST /vision/trajectory_manual",
        "required_fields": {
            "arm": '"left" | "right"',
            "target": "[x, y, z] meters in robot base frame",
        },
        "alternative_target": {"x": 0.48, "y": -0.24, "z": 0.42},
        "optional_fields": {
            "command_id": "string",
            "duration_s": 4.0,
            "action": "grasp | touch | goto | line | custom",
            "goto": "move tool tip to target (single point); use tool_offset [0,0,0] if target is flange center",
            "approach": "normalized local-plane Y offset (default 0.15); with plane_rpy roll=90 maps to world +Z",
            "grasp_down": "normalized local-plane Y at grasp (default -0.05); maps to world -Z when roll=90",
            "plane_rpy_deg": "[90, 0, 0] default — local Y is vertical in base frame",
            "rpy_deg": "[0, 180, 0] when lock_perpendicular_to_xy (default)",
            "lock_perpendicular_to_xy": "true (default): flange vertical down, ⊥ XY",
            "tool_down": "default ON: rpy [0,180,0], body Z // world -Z for entire path",
            "commands": "required only when action=custom",
        },
        "example_minimal": {
            "command_id": "v001",
            "arm": "right",
            "target": [0.48, -0.24, 0.42],
        },
        "example_grasp": {
            "command_id": "v002",
            "arm": "right",
            "x": 0.48,
            "y": -0.24,
            "z": 0.42,
            "duration_s": 4.0,
            "action": "grasp",
        },
    }
