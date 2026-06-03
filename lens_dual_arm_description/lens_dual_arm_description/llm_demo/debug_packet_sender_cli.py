from __future__ import annotations

import argparse
import json
from pathlib import Path
from urllib import request, error


def _post_json(url: str, payload: dict) -> tuple[int, str]:
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = request.Request(url, data=data, method="POST")
    req.add_header("Content-Type", "application/json")
    try:
        with request.urlopen(req, timeout=300) as resp:
            body = resp.read().decode("utf-8", errors="replace")
            return int(resp.status), body
    except error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        return int(exc.code), body


def _load_json_from_file(path_str: str) -> dict:
    p = Path(path_str).expanduser()
    return json.loads(p.read_text(encoding="utf-8"))


def main() -> None:
    ap = argparse.ArgumentParser(description="Manual packet sender for Path-IR FastAPI server.")
    ap.add_argument("--host", default="127.0.0.1", help="Server host")
    ap.add_argument("--port", type=int, default=8000, help="Server port")
    ap.add_argument(
        "--endpoint",
        default="/trajectory",
        choices=["/trajectory", "/trajectory_manual", "/confirm", "/status"],
        help="Default endpoint",
    )
    args = ap.parse_args()

    base = f"http://{args.host}:{args.port}"
    endpoint = args.endpoint
    print(f"[Sender] server={base} default_endpoint={endpoint}")
    print("Commands: :ep /trajectory|/trajectory_manual|/confirm|/status, :file <json_path>, :quit")
    print("Or paste raw JSON in one line and press Enter.")

    while True:
        raw = input("packet> ").strip()
        if not raw:
            continue
        if raw == ":quit":
            break
        if raw.startswith(":ep "):
            endpoint = raw.split(" ", 1)[1].strip()
            print(f"[Sender] endpoint -> {endpoint}")
            continue
        if raw == ":confirm":
            code, body = _post_json(base + "/confirm", {})
            print(f"[Resp] {code} {body}")
            continue
        if raw == ":status":
            with request.urlopen(base + "/status", timeout=30) as resp:
                body = resp.read().decode("utf-8", errors="replace")
                print(f"[Resp] {resp.status} {body}")
            continue

        payload: dict
        try:
            if raw.startswith(":file "):
                payload = _load_json_from_file(raw.split(" ", 1)[1].strip())
            else:
                payload = json.loads(raw)
        except Exception as exc:  # noqa: BLE001
            print(f"[Sender] invalid input: {exc}")
            continue

        code, body = _post_json(base + endpoint, payload)
        print(f"[Resp] {code} {body}")


if __name__ == "__main__":
    main()

