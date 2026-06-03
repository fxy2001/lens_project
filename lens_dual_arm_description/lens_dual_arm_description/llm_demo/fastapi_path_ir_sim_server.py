from __future__ import annotations

import os
import sys
import threading
import time
import json
import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import mujoco.viewer
import numpy as np
import tkinter as tk

# Ensure we can import sibling modules when executed from any cwd.
_THIS_DIR = Path(__file__).resolve().parent
_PKG_ROOT = _THIS_DIR.parent  # lens_dual_arm_description/
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

import dual_arm_llm_trajectory_demo as demo


try:
    from fastapi import FastAPI, HTTPException
    import uvicorn
except Exception as exc:  # noqa: BLE001
    raise RuntimeError(
        "Missing FastAPI deps. Please install: pip install fastapi uvicorn\n"
        "Then rerun this server script."
    ) from exc


@dataclass
class _SharedState:
    lock: threading.Lock
    # latest accepted command (raw json)
    last_command: dict[str, Any] | None
    last_command_id: str
    last_update_time_s: float
    last_error: str

    # planned trajectory for sim (always present after init)
    traj: demo.JointTrajectory | None
    traj_index: int

    # execution control
    pending_confirm: bool
    executing: bool
    execute_done_event: threading.Event
    execute_error: str
    execute_start_ts: float

    # ui control
    ui_confirm_requested: bool
    ui_cancel_requested: bool

    # packet logging
    packet_log_file: str
    plan_cache_dir: str

    # planning cache (single-cycle trajectory before replay expansion)
    last_plan_key: str
    last_plan_traj: demo.JointTrajectory | None


def _now() -> float:
    return time.time()


def _safe_command_id(cmd: dict[str, Any] | None) -> str:
    if not cmd:
        return ""
    v = cmd.get("command_id", "")
    return str(v) if v is not None else ""


def _stable_command_key(command: dict[str, Any]) -> str:
    s = json.dumps(command, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(s.encode("utf-8")).hexdigest()


def _prepare_fast_plan_command(command: dict[str, Any]) -> dict[str, Any]:
    """
    Cap planning sample_hz to reduce IK solve points and cut receive->execute latency.
    """
    cmd = json.loads(json.dumps(command))
    motion = cmd.get("motion")
    if not isinstance(motion, dict):
        return cmd
    src_hz = float(motion.get("sample_hz", 120.0))
    plan_hz_cap = float(os.environ.get("LENS_PLAN_SAMPLE_HZ", "50"))
    motion["sample_hz"] = max(10.0, min(src_hz, plan_hz_cap))
    # Plan exactly one cycle only; repeats are handled later by replay expansion.
    motion["repeat"] = 1
    return cmd


def _plan_from_command(cmd: dict[str, Any]) -> demo.JointTrajectory:
    # Force planning to start from home (q=0) so we go directly from home -> draw start,
    # rather than drifting to a mid-range posture first.
    os.environ.setdefault("LENS_IK_START_AT_HOME", "1")
    runtime = demo.build_runtime(_prepare_fast_plan_command(cmd))
    return demo.plan_joint_trajectory(runtime)


def _clone_traj(traj: demo.JointTrajectory) -> demo.JointTrajectory:
    return demo.JointTrajectory(
        joint_names=list(traj.joint_names),
        q_refs=[np.asarray(q, dtype=float).copy() for q in traj.q_refs],
        hz=float(traj.hz),
    )


def _cache_file_for_key(cache_dir: str, key: str) -> Path:
    return Path(cache_dir) / f"{key}.npz"


def _load_traj_from_disk_cache(cache_dir: str, key: str) -> demo.JointTrajectory | None:
    p = _cache_file_for_key(cache_dir, key)
    if not p.exists():
        return None
    try:
        z = np.load(str(p), allow_pickle=True)
        hz_arr = np.asarray(z["hz"], dtype=float).reshape(-1)
        hz = float(hz_arr[0]) if hz_arr.size > 0 else 0.0
        joint_names = [str(v) for v in z["joint_names"].tolist()]
        q = np.asarray(z["q_refs"], dtype=float)
        q_refs = [np.asarray(row, dtype=float).copy() for row in q]
        if not q_refs:
            return None
        return demo.JointTrajectory(joint_names=joint_names, q_refs=q_refs, hz=hz)
    except Exception as exc:  # noqa: BLE001
        print(f"[Cache] warning: failed to load {p}: {exc}")
        return None


def _save_traj_to_disk_cache(cache_dir: str, key: str, traj: demo.JointTrajectory) -> None:
    try:
        d = Path(cache_dir)
        d.mkdir(parents=True, exist_ok=True)
        p = _cache_file_for_key(cache_dir, key)
        q = np.asarray([np.asarray(v, dtype=float) for v in traj.q_refs], dtype=float)
        np.savez_compressed(
            str(p),
            hz=np.asarray([float(traj.hz)], dtype=float),
            joint_names=np.asarray(list(traj.joint_names), dtype=object),
            q_refs=q,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"[Cache] warning: failed to save key={key}: {exc}")


def _requested_repeat_count(command: dict[str, Any]) -> int:
    motion = command.get("motion")
    if not isinstance(motion, dict):
        return 1
    try:
        rep = int(motion.get("repeat", 1))
    except Exception:
        return 1
    if rep == -1:
        # Infinite replay is unsafe for blocking /trajectory endpoint.
        fallback = int(float(os.environ.get("LENS_REPEAT_INF_FALLBACK", "3")))
        return max(1, fallback)
    return max(1, rep)


def _expand_replay_cycles(traj: demo.JointTrajectory, repeat_count: int) -> demo.JointTrajectory:
    if repeat_count <= 1 or not traj.q_refs:
        return traj
    one = [np.asarray(q, dtype=float).copy() for q in traj.q_refs]
    q_refs: list[np.ndarray] = []
    for _ in range(int(repeat_count)):
        q_refs.extend([q.copy() for q in one])
    return demo.JointTrajectory(joint_names=list(traj.joint_names), q_refs=q_refs, hz=float(traj.hz))


def _linspace_blend(a: np.ndarray, b: np.ndarray, n: int) -> list[np.ndarray]:
    if n <= 1:
        return [b.copy()]
    out: list[np.ndarray] = []
    for t in np.linspace(0.0, 1.0, n, dtype=float):
        out.append((a * (1.0 - float(t)) + b * float(t)).astype(float, copy=False).copy())
    return out


def _augment_traj_home_blends(
    traj: demo.JointTrajectory,
    *,
    home_q: np.ndarray,
    approach_s: float = 1.0,
    hold_at_start_s: float = 0.5,
    return_s: float = 1.0,
    hold_home_s: float = 0.3,
) -> demo.JointTrajectory:
    """
    Make execution smoother and deterministic:
    - start from home (0) -> blend to first point (approach)
    - hold at the first point for a short time
    - run drawing trajectory
    - blend back to home (return)
    - hold home for a short time
    """
    if traj.hz <= 1e-6 or not traj.q_refs:
        return traj
    q0 = np.asarray(traj.q_refs[0], dtype=float).copy()
    home = np.asarray(home_q, dtype=float).copy()
    if home.shape != q0.shape:
        raise ValueError(f"home_q shape {home.shape} != traj q shape {q0.shape}")

    n_app = max(2, int(round(float(traj.hz) * max(0.0, approach_s))))
    n_ret = max(2, int(round(float(traj.hz) * max(0.0, return_s))))
    n_start_hold = max(1, int(round(float(traj.hz) * max(0.0, hold_at_start_s))))
    n_hold = max(1, int(round(float(traj.hz) * max(0.0, hold_home_s))))

    pre = _linspace_blend(home, q0, n_app)
    start_hold = [q0.copy() for _ in range(n_start_hold)]
    post = _linspace_blend(np.asarray(traj.q_refs[-1], dtype=float), home, n_ret)
    hold = [home.copy() for _ in range(n_hold)]

    q_refs = []
    q_refs.extend(pre)
    q_refs.extend(start_hold)
    q_refs.extend([np.asarray(q, dtype=float).copy() for q in traj.q_refs])
    q_refs.extend(post)
    q_refs.extend(hold)
    return demo.JointTrajectory(joint_names=list(traj.joint_names), q_refs=q_refs, hz=float(traj.hz))


def _clamp_inactive_arm_to_home(traj: demo.JointTrajectory, *, target_arm: str, home_q: np.ndarray) -> demo.JointTrajectory:
    """
    Ensure non-executing arm always stays at home (0), including during approach/return blends.
    target_arm: left/right/both
    """
    arm = str(target_arm).strip().lower()
    if arm not in ("left", "right", "both"):
        return traj
    if arm == "both":
        return traj
    if not traj.q_refs:
        return traj

    home = np.asarray(home_q, dtype=float).copy()
    n = len(traj.joint_names)
    if home.shape != (n,):
        raise ValueError(f"home_q shape {home.shape} != ({n},)")

    left_n = len(demo.LEFT_JOINT_NAMES)
    right_n = len(demo.RIGHT_JOINT_NAMES)
    if n != left_n + right_n:
        # Unknown layout; do nothing to avoid corrupting command order.
        return traj

    if arm == "left":
        inactive_slice = slice(left_n, left_n + right_n)
    else:
        inactive_slice = slice(0, left_n)

    q_refs: list[np.ndarray] = []
    for q in traj.q_refs:
        qq = np.asarray(q, dtype=float).copy()
        qq[inactive_slice] = home[inactive_slice]
        q_refs.append(qq)
    return demo.JointTrajectory(joint_names=list(traj.joint_names), q_refs=q_refs, hz=float(traj.hz))


def _inject_scope_from_dual_draw_demo_settings() -> None:
    """
    Strictly reuse the drawing plane/scope from dual_arm_ik_draw_circle_square.py:
    - left_center / right_center
    - shape_rpy_deg (drawing plane RPY)
    - square_half (range)
    This mirrors the persisted settings keys written by that script.
    """
    settings_file = Path(os.environ.get("LENS_DEMO_SETTINGS_FILE", str(Path.home() / ".lens_dual_arm_draw_demo.json")))
    if not settings_file.exists():
        print(f"[Scope] settings not found: {settings_file}, keep env/default scope.")
        return
    try:
        data = json.loads(settings_file.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        print(f"[Scope] failed to parse settings file: {settings_file}, err={exc}")
        return

    left = data.get("left_center")
    right = data.get("right_center")
    shape_rpy = data.get("shape_rpy_deg")
    square_half = data.get("square_half")

    if isinstance(left, list) and len(left) == 3:
        os.environ["LENS_DEMO_LEFT_X"] = str(float(left[0]))
        os.environ["LENS_DEMO_LEFT_Y"] = str(float(left[1]))
        os.environ["LENS_DEMO_LEFT_Z"] = str(float(left[2]))
    if isinstance(right, list) and len(right) == 3:
        os.environ["LENS_DEMO_RIGHT_X"] = str(float(right[0]))
        os.environ["LENS_DEMO_RIGHT_Y"] = str(float(right[1]))
        os.environ["LENS_DEMO_RIGHT_Z"] = str(float(right[2]))
    if isinstance(shape_rpy, list) and len(shape_rpy) == 3:
        os.environ["LENS_DEMO_SHAPE_ROLL_DEG"] = str(float(shape_rpy[0]))
        os.environ["LENS_DEMO_SHAPE_PITCH_DEG"] = str(float(shape_rpy[1]))
        os.environ["LENS_DEMO_SHAPE_YAW_DEG"] = str(float(shape_rpy[2]))
    if square_half is not None:
        os.environ["LENS_DEMO_SQUARE_HALF"] = str(float(square_half))

    print(
        "[Scope] loaded STRICT scope from dual_arm_ik_draw_circle_square.py settings: "
        f"left={left}, right={right}, shape_rpy={shape_rpy}, square_half={square_half}"
    )


def _append_packet_log(path: str, payload: dict[str, Any]) -> None:
    try:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("a", encoding="utf-8") as f:
            f.write(json.dumps(payload, ensure_ascii=False) + "\n")
    except Exception as exc:  # noqa: BLE001
        print(f"[Log] warning: failed to append packet log: {exc}")


def _wait_sim_synced_execution_done(state: _SharedState, command_id: str, timeout_s: float) -> tuple[bool, str]:
    with state.lock:
        if state.traj is None:
            return False, "no trajectory"
        if state.executing:
            return False, "already executing"
        # Hard reset playhead at execute start to avoid stale-end race.
        state.traj_index = 0
        state.executing = True
        state.pending_confirm = False
        state.last_error = ""
        state.execute_error = ""
        state.execute_start_ts = _now()
        state.execute_done_event.clear()
    print(f"[Execute] start sim-synced execution. command_id={command_id}")
    ok = state.execute_done_event.wait(timeout=max(1.0, float(timeout_s)))
    if not ok:
        with state.lock:
            state.executing = False
            state.last_error = "EXECUTE_TIMEOUT"
        return False, "EXECUTE_TIMEOUT"
    with state.lock:
        err = state.execute_error.strip()
        start_ts = state.execute_start_ts
    if err:
        return False, err
    elapsed = _now() - start_ts if start_ts > 0 else -1.0
    print(f"[Execute] finished. command_id={command_id} elapsed={elapsed:.3f}s")
    return True, ""


def _request_execute(state: _SharedState) -> tuple[bool, str]:
    with state.lock:
        if state.traj is None:
            return False, "no trajectory"
        if state.executing:
            return False, "already executing"
        command_id = state.last_command_id
        state.pending_confirm = True

    def run() -> None:
        with state.lock:
            hz = float(state.traj.hz) if state.traj is not None else 120.0
            n = len(state.traj.q_refs) if state.traj is not None else 0
        timeout_s = max(5.0, (n / max(hz, 1.0)) + 5.0)
        ok, err = _wait_sim_synced_execution_done(state, command_id, timeout_s=timeout_s)
        if not ok:
            with state.lock:
                state.last_error = f"EXECUTE_FAILED: {err}"

    threading.Thread(target=run, daemon=True).start()
    return True, command_id


def _start_confirm_ui(state: _SharedState) -> None:
    def run() -> None:
        try:
            root = tk.Tk()
            root.title("仿真确认 / 真机下发")
            root.geometry("420x220")
            root.attributes("-topmost", True)

            title = tk.Label(root, text="仿真常开：收到新轨迹会刷新预览", font=("TkDefaultFont", 10, "bold"))
            title.pack(pady=(12, 6))

            status_var = tk.StringVar(value="等待轨迹...")
            tk.Label(root, textvariable=status_var, fg="gray").pack(pady=(0, 10))

            def refresh_status() -> None:
                with state.lock:
                    cid = state.last_command_id
                    age = _now() - state.last_update_time_s if state.last_update_time_s > 0 else 0.0
                    err = state.last_error
                    executing = state.executing
                s = f"command_id={cid or '-'}  age={age:.1f}s"
                if executing:
                    s += "  [EXECUTING]"
                if err:
                    s += f"\nlast_error={err}"
                status_var.set(s)
                root.after(200, refresh_status)

            def confirm() -> None:
                with state.lock:
                    state.ui_confirm_requested = True

            def cancel() -> None:
                with state.lock:
                    state.ui_cancel_requested = True

            tk.Button(root, text="确认下发真机", command=confirm, width=18, height=2).pack(pady=4)
            tk.Button(root, text="取消执行请求", command=cancel, width=18).pack(pady=(0, 8))
            tk.Label(root, text="API：/trajectory(自动下发)  /trajectory_manual(需确认)  /confirm", fg="gray").pack()

            refresh_status()
            root.protocol("WM_DELETE_WINDOW", lambda: None)
            root.mainloop()
        except Exception as exc:  # noqa: BLE001
            print(f"[ConfirmUI] warning: failed to open confirm window: {exc}")
    threading.Thread(target=run, daemon=True).start()


def _start_api_server(state: _SharedState) -> None:
    app = FastAPI(title="Lens Dual-Arm Path IR Sim Server", version="1.0")

    @app.get("/status")
    def status() -> dict[str, Any]:
        with state.lock:
            traj_len = len(state.traj.q_refs) if state.traj is not None else 0
            hz = float(state.traj.hz) if state.traj is not None else 0.0
            return {
                "ok": True,
                "last_command_id": state.last_command_id,
                "last_update_time_s": state.last_update_time_s,
                "last_error": state.last_error,
                "traj_len": traj_len,
                "traj_hz": hz,
                "traj_index": state.traj_index,
                "pending_confirm": state.pending_confirm,
                "executing": state.executing,
                "execute_error": state.execute_error,
                "packet_log_file": state.packet_log_file,
                "plan_cache_dir": state.plan_cache_dir,
                "ros_bridge_enabled": os.environ.get("LENS_ROS_BRIDGE", "0"),
                "ros_topic": os.environ.get("LENS_ROS_TOPIC", "/joint_command"),
            }

    @app.post("/trajectory")
    def trajectory_auto_execute(command: dict[str, Any]) -> dict[str, Any]:
        """
        Auto mode: receive -> plan -> refresh sim -> execute real (blocking) -> return 200 after done.
        """
        t0 = _now()
        # Always enforce draw scope from dual-draw demo settings before planning.
        _inject_scope_from_dual_draw_demo_settings()
        demo._load_demo_draw_scope_from_env()
        _append_packet_log(state.packet_log_file, command)
        plan_key = _stable_command_key(_prepare_fast_plan_command(command))
        try:
            with state.lock:
                cached_ok = state.last_plan_key == plan_key and state.last_plan_traj is not None
                cached_traj = _clone_traj(state.last_plan_traj) if cached_ok else None
                cache_dir = state.plan_cache_dir
            disk_cached_traj = None if cached_traj is not None else _load_traj_from_disk_cache(cache_dir, plan_key)
            if cached_traj is not None:
                traj = cached_traj
                cache_source = "mem"
            elif disk_cached_traj is not None:
                traj = disk_cached_traj
                cache_source = "disk"
                with state.lock:
                    state.last_plan_key = plan_key
                    state.last_plan_traj = _clone_traj(traj)
            else:
                traj = _plan_from_command(command)
                with state.lock:
                    state.last_plan_key = plan_key
                    state.last_plan_traj = _clone_traj(traj)
                    cache_dir = state.plan_cache_dir
                _save_traj_to_disk_cache(cache_dir, plan_key, traj)
                cache_source = "miss"
        except Exception as exc:  # noqa: BLE001
            with state.lock:
                state.last_error = f"PLAN_FAILED: {exc}"
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        t1 = _now()

        with state.lock:
            state.last_command = command
            state.last_command_id = _safe_command_id(command)
            state.last_update_time_s = _now()
            state.last_error = ""
            repeat_count = _requested_repeat_count(command)
            traj = _expand_replay_cycles(traj, repeat_count)
            home_q = np.zeros(len(traj.joint_names), dtype=float)
            traj = _augment_traj_home_blends(traj, home_q=home_q)
            traj = _clamp_inactive_arm_to_home(traj, target_arm=str(command.get("target", {}).get("arm", "both")), home_q=home_q)
            state.traj = traj
            state.traj_index = 0
            state.pending_confirm = False
            state.ui_confirm_requested = False
            state.ui_cancel_requested = False

        print(
            "[API] trajectory updated: "
            f"command_id={state.last_command_id} hz={traj.hz:.1f} points={len(traj.q_refs)} repeat={repeat_count}"
        )
        print(
            f"[Perf] command_id={state.last_command_id} plan_time={(t1 - t0):.3f}s "
            f"cache={cache_source}"
        )
        timeout_s = max(5.0, (len(traj.q_refs) / max(float(traj.hz), 1.0)) + 5.0)
        ok, err = _wait_sim_synced_execution_done(state, state.last_command_id, timeout_s=timeout_s)
        if not ok:
            print(f"[Execute] auto_execute failed. command_id={state.last_command_id} err={err}")
            raise HTTPException(status_code=500, detail=err)
        t2 = _now()
        print(f"[Perf] command_id={state.last_command_id} total_api_time={(t2 - t0):.3f}s")
        return {
            "ok": True,
            "mode": "auto_execute",
            "command_id": state.last_command_id,
            "traj_len": len(traj.q_refs),
            "traj_hz": traj.hz,
            "executed": True,
        }

    @app.post("/trajectory_manual")
    def trajectory_manual_confirm(command: dict[str, Any]) -> dict[str, Any]:
        """
        Manual mode: receive -> plan -> refresh sim -> do NOT execute.
        Then call POST /confirm to execute (async) or use the UI confirm button.
        """
        t0 = _now()
        _inject_scope_from_dual_draw_demo_settings()
        demo._load_demo_draw_scope_from_env()
        _append_packet_log(state.packet_log_file, command)
        plan_key = _stable_command_key(_prepare_fast_plan_command(command))
        try:
            with state.lock:
                cached_ok = state.last_plan_key == plan_key and state.last_plan_traj is not None
                cached_traj = _clone_traj(state.last_plan_traj) if cached_ok else None
                cache_dir = state.plan_cache_dir
            disk_cached_traj = None if cached_traj is not None else _load_traj_from_disk_cache(cache_dir, plan_key)
            if cached_traj is not None:
                traj = cached_traj
                cache_source = "mem"
            elif disk_cached_traj is not None:
                traj = disk_cached_traj
                cache_source = "disk"
                with state.lock:
                    state.last_plan_key = plan_key
                    state.last_plan_traj = _clone_traj(traj)
            else:
                traj = _plan_from_command(command)
                with state.lock:
                    state.last_plan_key = plan_key
                    state.last_plan_traj = _clone_traj(traj)
                    cache_dir = state.plan_cache_dir
                _save_traj_to_disk_cache(cache_dir, plan_key, traj)
                cache_source = "miss"
        except Exception as exc:  # noqa: BLE001
            with state.lock:
                state.last_error = f"PLAN_FAILED: {exc}"
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        t1 = _now()

        with state.lock:
            state.last_command = command
            state.last_command_id = _safe_command_id(command)
            state.last_update_time_s = _now()
            state.last_error = ""
            repeat_count = _requested_repeat_count(command)
            traj = _expand_replay_cycles(traj, repeat_count)
            home_q = np.zeros(len(traj.joint_names), dtype=float)
            traj = _augment_traj_home_blends(traj, home_q=home_q)
            traj = _clamp_inactive_arm_to_home(traj, target_arm=str(command.get("target", {}).get("arm", "both")), home_q=home_q)
            state.traj = traj
            state.traj_index = 0
            state.pending_confirm = True
            state.ui_confirm_requested = False
            state.ui_cancel_requested = False

        print(
            "[API] trajectory staged (manual confirm required): "
            f"command_id={state.last_command_id} hz={traj.hz:.1f} points={len(traj.q_refs)} repeat={repeat_count}"
        )
        print(
            f"[Perf] command_id={state.last_command_id} stage_plan_time={(t1 - t0):.3f}s "
            f"cache={cache_source}"
        )
        return {
            "ok": True,
            "mode": "manual_confirm",
            "command_id": state.last_command_id,
            "traj_len": len(traj.q_refs),
            "traj_hz": traj.hz,
            "executed": False,
            "next": "POST /confirm",
        }

    @app.post("/confirm")
    def confirm() -> dict[str, Any]:
        ok, info = _request_execute(state)
        if not ok:
            raise HTTPException(status_code=400, detail=info)
        return {"ok": True, "executing": True, "command_id": info}

    @app.post("/cancel")
    def cancel() -> dict[str, Any]:
        with state.lock:
            state.pending_confirm = False
        return {"ok": True}

    host = os.environ.get("LENS_FASTAPI_HOST", "0.0.0.0")
    port = int(os.environ.get("LENS_FASTAPI_PORT", "8000"))
    log_level = os.environ.get("LENS_FASTAPI_LOG_LEVEL", "info")
    uvicorn.run(app, host=host, port=port, log_level=log_level)


def _sim_loop(state: _SharedState) -> None:
    # Strictly pin drawing plane/scope to dual draw demo settings.
    _inject_scope_from_dual_draw_demo_settings()
    demo._load_demo_draw_scope_from_env()

    # Do not preload fallback drawing. Stay idle until external trajectory arrives.
    with state.lock:
        state.last_command = None
        state.last_command_id = ""
        state.last_update_time_s = 0.0
        state.last_error = ""
        state.traj = None
        state.traj_index = 0

    model_path = os.environ.get("LENS_DUAL_ARM_MJCF", demo.DEFAULT_MODEL_PATH)
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    left_act_ids = demo._left_actuator_ids(model)
    right_act_ids = demo._right_actuator_ids(model)
    left_q_idx = demo._joint_indices(model, demo.LEFT_JOINT_NAMES)
    right_q_idx = demo._joint_indices(model, demo.RIGHT_JOINT_NAMES)
    left_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, demo.LEFT_EE_BODY)
    right_ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, demo.RIGHT_EE_BODY)
    ros_bridge = demo.Ros2JointCommandPublisher(
        demo.Ros2BridgeConfig(
            enabled=os.environ.get("LENS_ROS_BRIDGE", "0").strip().lower() in ("1", "true", "yes", "on"),
            topic=os.environ.get("LENS_ROS_TOPIC", "/joint_command"),
            hz=float(os.environ.get("LENS_ROS_HZ", "0")) or 0.0,
        )
    )
    ros_enabled = bool(getattr(ros_bridge, "cfg", None) and ros_bridge.cfg.enabled)
    ros_ok = bool(getattr(ros_bridge, "_ok", False))
    print(f"[ROS] bridge enabled={ros_enabled} ok={ros_ok} topic={getattr(ros_bridge, 'cfg', None).topic if getattr(ros_bridge, 'cfg', None) else '-'}")
    first_publish_logged = False

    # Visualization settings
    trace_len = int(float(os.environ.get("LENS_TRACE_LEN", "600")))
    trace_stride = max(1, int(float(os.environ.get("LENS_TRACE_STRIDE", "2"))))
    trace_point_size = float(os.environ.get("LENS_TRACE_POINT_SIZE", "0.0055"))
    trace_alpha = float(os.environ.get("LENS_TRACE_ALPHA", "0.85"))
    ee_marker_size = float(os.environ.get("LENS_TRACE_EE_SIZE", "0.012"))
    left_trace: list[np.ndarray] = []
    right_trace: list[np.ndarray] = []
    can_add_marker: bool | None = None

    print("[Sim] viewer running (always-on). POST /trajectory to refresh. POST /confirm to execute real.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        can_add_marker = hasattr(viewer, "add_marker")

        while viewer.is_running():
            with state.lock:
                if state.ui_cancel_requested:
                    state.ui_cancel_requested = False
                    state.ui_confirm_requested = False
                    state.pending_confirm = False
                do_confirm = state.ui_confirm_requested and state.pending_confirm
                if do_confirm:
                    state.ui_confirm_requested = False
            if do_confirm:
                ok, info = _request_execute(state)
                if not ok:
                    with state.lock:
                        state.last_error = f"CONFIRM_FAILED: {info}"

            tic = _now()
            with state.lock:
                cur_traj = state.traj
                idx = state.traj_index
                is_executing = state.executing

            if cur_traj is None or not cur_traj.q_refs:
                time.sleep(0.01)
                continue

            idx = int(np.clip(idx, 0, len(cur_traj.q_refs) - 1))
            q_ref = cur_traj.q_refs[idx]

            # Advance index ONCE (no looping). After reaching the end, keep holding the last point.
            with state.lock:
                # Only advance if we are still operating on the same trajectory object.
                # This avoids stale local idx from old trajectory overwriting new trajectory playhead.
                if state.traj is cur_traj:
                    if idx < len(cur_traj.q_refs) - 1:
                        state.traj_index = idx + 1
                    else:
                        state.traj_index = idx

            left_q = q_ref[: len(demo.LEFT_JOINT_NAMES)]
            right_q = q_ref[len(demo.LEFT_JOINT_NAMES) :]
            data.ctrl[left_act_ids] = left_q
            data.ctrl[right_act_ids] = right_q
            data.qpos[left_q_idx] = left_q
            data.qpos[right_q_idx] = right_q
            mujoco.mj_step(model, data)
            if is_executing and ros_enabled and not bool(getattr(ros_bridge, "_ok", False)):
                with state.lock:
                    state.executing = False
                    state.execute_error = "ROS_BRIDGE_NOT_READY"
                    state.last_error = "EXECUTE_FAILED: ROS_BRIDGE_NOT_READY"
                    state.execute_done_event.set()
            if is_executing and bool(getattr(ros_bridge, "_ok", False)):
                ros_bridge.maybe_publish(_now(), cur_traj.joint_names, q_ref)
                if not first_publish_logged:
                    first_publish_logged = True
                    print(f"[ROS] first command published at traj_idx={idx}")

            left_pos = data.xpos[left_ee_id].copy()
            right_pos = data.xpos[right_ee_id].copy()
            left_trace.append(left_pos)
            right_trace.append(right_pos)
            if len(left_trace) > trace_len:
                left_trace = left_trace[-trace_len:]
            if len(right_trace) > trace_len:
                right_trace = right_trace[-trace_len:]

            if can_add_marker:
                try:
                    # draw denser and brighter trace for clearer visibility
                    for p in left_trace[::trace_stride]:
                        viewer.add_marker(
                            pos=p,
                            size=np.array([trace_point_size, trace_point_size, trace_point_size], dtype=float),
                            rgba=np.array([0.2, 1.0, 0.2, trace_alpha], dtype=float),
                        )
                    for p in right_trace[::trace_stride]:
                        viewer.add_marker(
                            pos=p,
                            size=np.array([trace_point_size, trace_point_size, trace_point_size], dtype=float),
                            rgba=np.array([1.0, 0.2, 0.2, trace_alpha], dtype=float),
                        )
                    viewer.add_marker(
                        pos=left_pos,
                        size=np.array([ee_marker_size, ee_marker_size, ee_marker_size], dtype=float),
                        rgba=np.array([0.2, 1.0, 0.2, 0.95], dtype=float),
                    )
                    viewer.add_marker(
                        pos=right_pos,
                        size=np.array([ee_marker_size, ee_marker_size, ee_marker_size], dtype=float),
                        rgba=np.array([1.0, 0.2, 0.2, 0.95], dtype=float),
                    )
                except Exception:
                    can_add_marker = False

            if not can_add_marker:
                demo._draw_trace_userscn(viewer, left_trace, right_trace, left_pos, right_pos)

            viewer.sync()

            # Signal execution completion once the single-shot replay reaches the end.
            with state.lock:
                if state.executing and state.traj is not None and state.traj_index >= len(state.traj.q_refs) - 1:
                    state.executing = False
                    state.execute_error = ""
                    state.execute_done_event.set()

            # try to follow trajectory hz for smoothness
            target_dt = 1.0 / float(cur_traj.hz)
            time.sleep(max(0.0, target_dt - (_now() - tic)))


def main() -> None:
    packet_log_file = os.environ.get(
        "LENS_PATH_IR_PACKET_LOG",
        str(Path.home() / ".lens_path_ir_packets.jsonl"),
    )
    plan_cache_dir = os.environ.get(
        "LENS_PATH_IR_PLAN_CACHE_DIR",
        str(Path.home() / ".lens_path_ir_plan_cache"),
    )
    state = _SharedState(
        lock=threading.Lock(),
        last_command=None,
        last_command_id="",
        last_update_time_s=0.0,
        last_error="",
        traj=None,
        traj_index=0,
        pending_confirm=False,
        executing=False,
        execute_done_event=threading.Event(),
        execute_error="",
        execute_start_ts=0.0,
        ui_confirm_requested=False,
        ui_cancel_requested=False,
        packet_log_file=packet_log_file,
        plan_cache_dir=plan_cache_dir,
        last_plan_key="",
        last_plan_traj=None,
    )

    # Run API in background so viewer stays in main thread (more stable for OpenGL).
    threading.Thread(target=_start_api_server, args=(state,), daemon=True).start()
    _start_confirm_ui(state)
    _sim_loop(state)


if __name__ == "__main__":
    main()

