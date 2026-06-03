"""
Headless Path IR HTTP API for external projects.

Accepts draw_path JSON over HTTP, plans IK trajectory, and optionally executes on real robot.

Endpoints:
  GET  /health
  GET  /status
  POST /trajectory          plan + execute (blocking until motion done)
  POST /trajectory_manual   plan only, wait for POST /confirm
  POST /confirm             execute staged trajectory
  POST /cancel              cancel staged manual trajectory

  POST /vision/trajectory         minimal vision JSON -> Path IR -> plan + execute
  POST /vision/trajectory_manual  minimal vision JSON -> plan only
  GET  /vision/schema             minimal JSON field help + example
"""
from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

_THIS_DIR = Path(__file__).resolve().parent
_PKG_ROOT = _THIS_DIR.parent
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

import dual_arm_llm_trajectory_demo as demo
import vision_simple_to_path_ir as vision_ir

try:
    from fastapi import FastAPI, HTTPException
    from fastapi.middleware.cors import CORSMiddleware
    import uvicorn
except Exception as exc:  # noqa: BLE001
    raise RuntimeError(
        "Missing FastAPI deps. Install: pip install fastapi uvicorn\n"
        "Then rerun this server."
    ) from exc


class _ServerState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.last_command: dict[str, Any] | None = None
        self.last_command_id = ""
        self.last_error = ""
        self.last_update_time_s = 0.0
        self.traj: demo.JointTrajectory | None = None
        self.runtime: demo.CommandRuntime | None = None
        self.pending_confirm = False
        self.executing = False


def _now() -> float:
    return time.time()


def _command_id(command: dict[str, Any]) -> str:
    return str(command.get("command_id", "") or "")


def _vision_body_to_path_ir(body: dict[str, Any]) -> dict[str, Any]:
    try:
        return vision_ir.vision_simple_to_path_ir(body)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


def _plan_and_stage(state: _ServerState, command: dict[str, Any], timing: demo.RunTiming) -> demo.JointTrajectory:
    demo._apply_runtime_defaults()
    demo._load_demo_draw_scope_from_env()
    with timing.span("command.prepare"):
        prepared = demo._prepare_fast_plan_command(command)
    with timing.span("runtime.build"):
        runtime = demo.build_runtime_for_command(prepared)
    traj = demo.plan_joint_trajectory_cached(prepared, already_prepared=True, timing=timing)
    with state.lock:
        state.last_command = command
        state.last_command_id = _command_id(command)
        state.last_update_time_s = _now()
        state.last_error = ""
        state.traj = traj
        state.runtime = runtime
    return traj


def _execute_staged(state: _ServerState, timing: demo.RunTiming) -> demo.ExecuteResult:
    with state.lock:
        if state.traj is None or state.runtime is None:
            raise RuntimeError("NO_TRAJECTORY: call /trajectory or /trajectory_manual first")
        if state.executing:
            raise RuntimeError("BUSY: execution already in progress")
        traj = state.traj
        runtime = state.runtime
        state.executing = True
        state.pending_confirm = False
    try:
        return demo.execute_real(traj, duration_s=runtime.duration_s, timing=timing)
    finally:
        with state.lock:
            state.executing = False


def create_app(state: _ServerState | None = None) -> FastAPI:
    state = state or _ServerState()
    app = FastAPI(
        title="Lens Dual-Arm Path IR API",
        version="1.0",
        description="External HTTP input for Path IR draw_path commands.",
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @app.get("/health")
    def health() -> dict[str, Any]:
        return {"ok": True, "service": "lens_path_ir_api"}

    @app.get("/status")
    def status() -> dict[str, Any]:
        with state.lock:
            traj_len = len(state.traj.q_refs) if state.traj is not None else 0
            traj_hz = float(state.traj.hz) if state.traj is not None else 0.0
            duration_s = float(state.runtime.duration_s) if state.runtime is not None else 0.0
            return {
                "ok": True,
                "last_command_id": state.last_command_id,
                "last_update_time_s": state.last_update_time_s,
                "last_error": state.last_error,
                "traj_len": traj_len,
                "traj_hz": traj_hz,
                "duration_s": duration_s,
                "pending_confirm": state.pending_confirm,
                "executing": state.executing,
                "ros_bridge_enabled": demo._env_truthy("LENS_ROS_BRIDGE"),
                "ros_topic": os.environ.get("LENS_ROS_TOPIC", "/joint_command"),
                "host": os.environ.get("LENS_FASTAPI_HOST", "0.0.0.0"),
                "port": int(os.environ.get("LENS_FASTAPI_PORT", "8000")),
            }

    @app.post("/trajectory")
    def trajectory_auto_execute(command: dict[str, Any]) -> dict[str, Any]:
        """Receive Path IR JSON, plan IK, execute on real robot, return when done."""
        timing = demo.RunTiming()
        timing.meta["command_id"] = _command_id(command)
        try:
            traj = _plan_and_stage(state, command, timing)
            runtime = state.runtime
            assert runtime is not None
            exec_result = _execute_staged(state, timing)
        except Exception as exc:  # noqa: BLE001
            with state.lock:
                state.last_error = str(exc)
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        timing.print_report()
        report = timing.report_dict()
        return {
            "ok": True,
            "mode": "auto_execute",
            "command_id": state.last_command_id,
            "traj_len": len(traj.q_refs),
            "traj_hz": traj.hz,
            "duration_s": runtime.duration_s,
            "execute_s": exec_result.total_s,
            "motion_s": exec_result.motion_s,
            "hold_s": exec_result.hold_s,
            "command_to_motion_done_s": report["command_to_motion_done_s"],
            "timing": report,
            "executed": True,
        }

    @app.post("/trajectory_manual")
    def trajectory_manual(command: dict[str, Any]) -> dict[str, Any]:
        """Plan only; call POST /confirm to execute."""
        timing = demo.RunTiming()
        timing.meta["command_id"] = _command_id(command)
        try:
            traj = _plan_and_stage(state, command, timing)
            runtime = state.runtime
            assert runtime is not None
            with state.lock:
                state.pending_confirm = True
        except Exception as exc:  # noqa: BLE001
            with state.lock:
                state.last_error = str(exc)
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        timing.print_report()
        report = timing.report_dict()
        return {
            "ok": True,
            "mode": "manual_confirm",
            "command_id": state.last_command_id,
            "traj_len": len(traj.q_refs),
            "traj_hz": traj.hz,
            "duration_s": runtime.duration_s,
            "plan_s": report["wall_clock_total_s"],
            "timing": report,
            "executed": False,
            "next": "POST /confirm",
        }

    @app.post("/confirm")
    def confirm() -> dict[str, Any]:
        with state.lock:
            if not state.pending_confirm:
                raise HTTPException(status_code=400, detail="NO_PENDING_TRAJECTORY")
            if state.traj is None or state.runtime is None:
                raise HTTPException(status_code=400, detail="NO_TRAJECTORY")
        timing = demo.RunTiming()
        timing.meta["command_id"] = state.last_command_id
        try:
            exec_result = _execute_staged(state, timing)
        except Exception as exc:  # noqa: BLE001
            with state.lock:
                state.last_error = str(exc)
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        timing.print_report()
        report = timing.report_dict()
        return {
            "ok": True,
            "command_id": state.last_command_id,
            "execute_s": exec_result.total_s,
            "motion_s": exec_result.motion_s,
            "command_to_motion_done_s": report["command_to_motion_done_s"],
            "timing": report,
            "executed": True,
        }

    @app.post("/cancel")
    def cancel() -> dict[str, Any]:
        with state.lock:
            state.pending_confirm = False
        return {"ok": True}

    @app.get("/vision/schema")
    def vision_schema() -> dict[str, Any]:
        """Minimal JSON format for vision colleagues."""
        return {"ok": True, **vision_ir.vision_simple_schema()}

    @app.post("/vision/trajectory")
    def vision_trajectory_auto(body: dict[str, Any]) -> dict[str, Any]:
        """Minimal vision JSON -> expand to Path IR -> plan + execute."""
        path_ir = _vision_body_to_path_ir(body)
        timing = demo.RunTiming()
        timing.meta["command_id"] = _command_id(path_ir)
        timing.meta["vision_simple"] = True
        try:
            traj = _plan_and_stage(state, path_ir, timing)
            runtime = state.runtime
            assert runtime is not None
            exec_result = _execute_staged(state, timing)
        except Exception as exc:  # noqa: BLE001
            with state.lock:
                state.last_error = str(exc)
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        timing.print_report()
        report = timing.report_dict()
        return {
            "ok": True,
            "mode": "vision_auto_execute",
            "command_id": state.last_command_id,
            "path_ir_command_id": path_ir.get("command_id"),
            "traj_len": len(traj.q_refs),
            "traj_hz": traj.hz,
            "duration_s": runtime.duration_s,
            "execute_s": exec_result.total_s,
            "command_to_motion_done_s": report["command_to_motion_done_s"],
            "executed": True,
        }

    @app.post("/vision/trajectory_manual")
    def vision_trajectory_manual(body: dict[str, Any]) -> dict[str, Any]:
        """Minimal vision JSON -> expand to Path IR -> plan only."""
        path_ir = _vision_body_to_path_ir(body)
        timing = demo.RunTiming()
        timing.meta["command_id"] = _command_id(path_ir)
        timing.meta["vision_simple"] = True
        try:
            traj = _plan_and_stage(state, path_ir, timing)
            runtime = state.runtime
            assert runtime is not None
            with state.lock:
                state.pending_confirm = True
        except Exception as exc:  # noqa: BLE001
            with state.lock:
                state.last_error = str(exc)
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        timing.print_report()
        report = timing.report_dict()
        return {
            "ok": True,
            "mode": "vision_manual_confirm",
            "command_id": state.last_command_id,
            "traj_len": len(traj.q_refs),
            "duration_s": runtime.duration_s,
            "plan_s": report["wall_clock_total_s"],
            "executed": False,
            "next": "POST /confirm",
        }

    return app


def main() -> None:
    demo._apply_runtime_defaults()
    os.environ.setdefault("LENS_AUTO_EXECUTE", "1")
    os.environ.setdefault("LENS_ROS_BRIDGE", "1")

    host = os.environ.get("LENS_FASTAPI_HOST", "0.0.0.0")
    port = int(os.environ.get("LENS_FASTAPI_PORT", "8000"))
    log_level = os.environ.get("LENS_FASTAPI_LOG_LEVEL", "info")

    print(f"[API] Lens Path IR server listening on http://{host}:{port}")
    print("[API] POST /trajectory  (full Path IR JSON, auto plan+execute)")
    print("[API] POST /vision/trajectory  (minimal vision JSON, auto convert+execute)")
    print("[API] GET  /vision/schema  (minimal JSON help)")
    print("[API] GET  /status /health")
    print(
        f"[API] ROS bridge={'ON' if demo._env_truthy('LENS_ROS_BRIDGE') else 'OFF'} "
        f"topic={os.environ.get('LENS_ROS_TOPIC', '/joint_command')}"
    )

    app = create_app()
    uvicorn.run(app, host=host, port=port, log_level=log_level)


if __name__ == "__main__":
    main()
