from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path
from urllib import error, request


THIS_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT = THIS_DIR / "ir.json"
DEFAULT_SPEC_A = THIS_DIR / "llm_path_ir_output_spec.md"
DEFAULT_SPEC_B = THIS_DIR / "mechanical_arm_path_ir_control_spec.md"


def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _extract_first_json_block(text: str) -> dict:
    # Prefer fenced json block first.
    fenced = re.findall(r"```json\s*(\{[\s\S]*?\})\s*```", text, flags=re.IGNORECASE)
    candidates = fenced if fenced else []
    if not candidates:
        # Fallback: first balanced-like object (greedy between first "{" and last "}").
        s = text.find("{")
        e = text.rfind("}")
        if s >= 0 and e > s:
            candidates = [text[s : e + 1]]
    for c in candidates:
        try:
            return json.loads(c)
        except Exception:
            continue
    raise ValueError("Model response does not contain valid JSON object.")


def _validate_ir(obj: dict) -> None:
    required_top = ["version", "command_id", "command_type", "target", "frame", "path", "motion", "end_effector", "safety"]
    for k in required_top:
        if k not in obj:
            raise ValueError(f"Missing top-level field: {k}")
    if obj.get("command_type") != "draw_path":
        raise ValueError("command_type must be draw_path")
    if obj.get("version") != "2.0":
        raise ValueError("version must be 2.0")
    if not isinstance(obj["path"].get("commands"), list) or len(obj["path"]["commands"]) == 0:
        raise ValueError("path.commands must be a non-empty list")
    if obj["target"].get("arm") not in ("left", "right", "both"):
        raise ValueError("target.arm must be left/right/both")
    if obj["target"].get("mode") not in ("ee_position", "ee_pose"):
        raise ValueError("target.mode must be ee_position/ee_pose")


def _is_zhipu_endpoint(base_url: str) -> bool:
    return "bigmodel.cn" in base_url.lower()


def _validate_api_key_or_exit(api_key: str, *, zhipu_format: bool) -> None:
    key = api_key.strip()
    if not key:
        raise SystemExit("API key is empty. Set OPENAI_API_KEY or ZHIPU_API_KEY.")
    if "你的" in key or key.lower() in ("key", "sk-xxx", "your_api_key"):
        raise SystemExit("API key looks like a placeholder. Please use your real key.")
    try:
        key.encode("ascii")
    except UnicodeEncodeError as exc:
        raise SystemExit("API key contains non-ASCII characters.") from exc
    if zhipu_format and "." not in key:
        raise SystemExit("Invalid ZHIPU_API_KEY format. Expected: {API Key ID}.{secret}")
    if len(key) < 8:
        raise SystemExit("API key looks too short.")


def _build_messages(user_text: str, spec_a: str, spec_b: str) -> list[dict]:
    system = (
        "你是机械臂 Path IR 生成器。"
        "必须严格输出一个 JSON 对象，不允许任何解释文字。"
        "输出必须符合 Path IR v2.0，且 command_type 必须为 draw_path。"
        "未知字段不要输出；必须包含 target/frame/path/motion/end_effector/safety。"
    )
    user = (
        "请根据以下规范，把自然语言转为一条 Path IR v2.0 JSON 指令。\n\n"
        "==== 规范A（大模型输出规范）====\n"
        f"{spec_a}\n\n"
        "==== 规范B（控制端协议规范）====\n"
        f"{spec_b}\n\n"
        "==== 用户文本指令 ====\n"
        f"{user_text}\n\n"
        "只返回 JSON。"
    )
    return [
        {"role": "system", "content": system},
        {"role": "user", "content": user},
    ]


def _call_openai_chat(messages: list[dict], model: str, api_key: str, base_url: str) -> str:
    url = base_url.rstrip("/") + "/chat/completions"
    body = {
        "model": model,
        "temperature": 0.1,
        "messages": messages,
    }
    data = json.dumps(body, ensure_ascii=False).encode("utf-8")
    req = request.Request(url, method="POST", data=data)
    req.add_header("Authorization", f"Bearer {api_key}")
    req.add_header("Content-Type", "application/json")
    try:
        with request.urlopen(req, timeout=120) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
    except error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        if int(exc.code) == 429:
            raise RuntimeError(
                "LLM API rate-limited (HTTP 429 Too Many Requests). "
                "Please wait and retry, reduce request frequency, check account quota/billing, "
                "or switch to another reachable BASE_URL gateway."
            ) from exc
        raise RuntimeError(f"LLM API HTTP {exc.code}: {detail}") from exc
    except error.URLError as exc:
        reason = getattr(exc, "reason", exc)
        reason_s = str(reason)
        if "Too Many Requests" in reason_s or "429" in reason_s:
            raise RuntimeError(
                "LLM API appears rate-limited (429). "
                "Please retry later or use another BASE_URL gateway."
            ) from exc
        raise RuntimeError(
            "Network unreachable when calling LLM API. "
            "Please check internet/proxy, or set BASE_URL to a reachable gateway. "
            f"base_url={base_url}, reason={reason}"
        ) from exc
    payload = json.loads(raw)
    choices = payload.get("choices") or []
    if not choices:
        raise RuntimeError("OpenAI response has no choices")
    msg = choices[0].get("message") or {}
    content = msg.get("content")
    if not isinstance(content, str) or not content.strip():
        raise RuntimeError("OpenAI response content is empty")
    return content


def main() -> None:
    ap = argparse.ArgumentParser(description="Convert text instruction to Path IR JSON.")
    ap.add_argument("--text", default="", help="Natural language instruction text. If empty, prompt interactively.")
    ap.add_argument("--output", default=str(DEFAULT_OUTPUT), help="Output Path IR JSON path.")
    ap.add_argument(
        "--model",
        default=os.environ.get("ZHIPU_MODEL", os.environ.get("OPENAI_MODEL", "glm-4-plus")),
        help="Model name (default: glm-4-plus for Zhipu).",
    )
    ap.add_argument(
        "--base-url",
        default=os.environ.get("ZHIPU_BASE_URL", os.environ.get("OPENAI_BASE_URL", "https://open.bigmodel.cn/api/paas/v4")),
        help="API base URL.",
    )
    ap.add_argument("--spec-a", default=str(DEFAULT_SPEC_A), help="Path to llm output spec markdown.")
    ap.add_argument("--spec-b", default=str(DEFAULT_SPEC_B), help="Path to protocol spec markdown.")
    ap.add_argument("--print-only", action="store_true", help="Print JSON only, do not write file.")
    args = ap.parse_args()

    zhipu_key = os.environ.get("ZHIPU_API_KEY", "").strip()
    openai_key = os.environ.get("OPENAI_API_KEY", "").strip()
    api_key = zhipu_key or openai_key
    if not api_key:
        raise SystemExit("Set OPENAI_API_KEY or ZHIPU_API_KEY before running.")
    zhipu_format = bool(zhipu_key) or _is_zhipu_endpoint(args.base_url)
    _validate_api_key_or_exit(api_key, zhipu_format=zhipu_format)

    text = args.text.strip()
    if not text:
        text = input("请输入文本指令: ").strip()
    if not text:
        raise SystemExit("Empty instruction text.")

    spec_a = _read_text(Path(args.spec_a))
    spec_b = _read_text(Path(args.spec_b))
    messages = _build_messages(text, spec_a, spec_b)
    try:
        content = _call_openai_chat(messages, model=args.model, api_key=api_key, base_url=args.base_url)
    except RuntimeError as exc:
        raise SystemExit(f"[ERROR] {exc}") from exc
    obj = _extract_first_json_block(content)
    _validate_ir(obj)

    out = json.dumps(obj, ensure_ascii=False, indent=2)
    if args.print_only:
        print(out)
        return

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(out + "\n", encoding="utf-8")
    print(f"[OK] Path IR written: {out_path}")
    print(f"[OK] command_id={obj.get('command_id')} arm={(obj.get('target') or {}).get('arm')} mode={(obj.get('target') or {}).get('mode')}")


if __name__ == "__main__":
    main()

