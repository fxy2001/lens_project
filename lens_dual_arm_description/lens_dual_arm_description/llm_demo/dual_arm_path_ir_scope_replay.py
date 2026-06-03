from __future__ import annotations

import json
import os
from pathlib import Path
import sys

# Reuse the fully implemented pipeline:
# Path IR parse -> strict fit to draw scope -> IK plan -> sim preview -> confirm -> ROS publish.
_THIS_DIR = Path(__file__).resolve().parent
_PKG_ROOT = _THIS_DIR.parent  # lens_dual_arm_description/
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

import dual_arm_llm_trajectory_demo as demo


def _load_draw_scope_from_saved_settings() -> None:
    """
    Load draw scope from the same persisted settings used by
    dual_arm_ik_draw_circle_square.py and inject into env vars consumed by demo.
    """
    settings_file = Path(
        os.environ.get("LENS_DEMO_SETTINGS_FILE", str(Path.home() / ".lens_dual_arm_draw_demo.json"))
    )
    if not settings_file.exists():
        print(f"[Scope] settings not found: {settings_file}, fallback to env/defaults.")
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
        "[Scope] loaded from saved settings: "
        f"left={left}, right={right}, shape_rpy={shape_rpy}, square_half={square_half}"
    )


def main() -> None:
    # Priority:
    # 1) caller-exported env vars
    # 2) saved draw-demo settings
    # 3) hardcoded defaults inside dual_arm_llm_trajectory_demo.py
    _load_draw_scope_from_saved_settings()

    print("[Scope] strict scaling is ENABLED to draw scope from dual-arm draw demo.")
    print("[Flow] parse Path IR -> strict fit -> sim preview -> user confirm -> real publish")
    demo.main()


if __name__ == "__main__":
    main()

