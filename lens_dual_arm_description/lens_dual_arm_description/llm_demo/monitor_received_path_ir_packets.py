from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path


def main() -> None:
    ap = argparse.ArgumentParser(description="Monitor received Path IR packets (jsonl).")
    ap.add_argument(
        "--file",
        default=os.environ.get("LENS_PATH_IR_PACKET_LOG", str(Path.home() / ".lens_path_ir_packets.jsonl")),
        help="jsonl file written by fastapi_path_ir_sim_server.py",
    )
    ap.add_argument("--pretty", action="store_true", help="Pretty-print JSON payload")
    args = ap.parse_args()

    p = Path(args.file)
    print(f"[Monitor] file={p}")
    if not p.exists():
        print("[Monitor] waiting for file to be created...")
        p.parent.mkdir(parents=True, exist_ok=True)
        p.touch(exist_ok=True)

    with p.open("r", encoding="utf-8") as f:
        f.seek(0, 2)
        while True:
            line = f.readline()
            if not line:
                time.sleep(0.1)
                continue
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except Exception:
                print(f"[Packet] {line}")
                continue
            cid = obj.get("command_id", "-")
            arm = (obj.get("target") or {}).get("arm", "-")
            mode = (obj.get("target") or {}).get("mode", "-")
            print(f"[Packet] command_id={cid} arm={arm} mode={mode}")
            if args.pretty:
                print(json.dumps(obj, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

