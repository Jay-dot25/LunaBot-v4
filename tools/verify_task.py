#!/usr/bin/env python3
"""Validate the project task board and create non-overwriting run evidence."""
from __future__ import annotations
import argparse
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BOARD = ROOT / "docs" / "task-board.md"
ID_RE = re.compile(r"^\| (P(?:0|1|2|3|4|5|6|7|8|9|10|11)-[0-9][0-9]) \|")
DEP_RE = re.compile(r"P(?:0|1|2|3|4|5|6|7|8|9|10|11)-[0-9][0-9]")

def validate_board(text: str) -> tuple[list[str], list[str]]:
    rows = [line for line in text.splitlines() if line.startswith("| P")]
    ids = [m.group(1) for line in rows if (m := ID_RE.match(line))]
    errors: list[str] = []
    if len(ids) != 36:
        errors.append(f"expected 36 task rows, found {len(ids)}")
    if len(set(ids)) != len(ids):
        errors.append("duplicate task IDs")
    known = set(ids)
    for line in rows:
        match = ID_RE.match(line)
        if not match:
            continue
        task_id = match.group(1)
        cells = [cell.strip() for cell in line.strip("|").split("|")]
        if len(cells) != 5:
            errors.append(f"{task_id}: expected five board columns")
            continue
        deps = [ref for ref in DEP_RE.findall(cells[2]) if ref != task_id]
        unknown = sorted(set(deps) - known)
        if unknown:
            errors.append(f"{task_id}: unknown dependencies {unknown}")
        if not cells[3] or not cells[4]:
            errors.append(f"{task_id}: affected files or acceptance missing")
    return ids, errors

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", default="P0-03")
    parser.add_argument("--run-id", default=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        _, errors = validate_board("| P0-01 | P0 | MISSING-99 | x | y |")
        if not errors:
            print("FAIL: malformed fixture was not rejected")
            return 1
        print("PASS: malformed task-board fixture rejected")
        return 0
    if not re.fullmatch(r"P(?:0|1|2|3|4|5|6|7|8|9|10|11)-[0-9][0-9]", args.task):
        parser.error("invalid task ID")
    if not re.fullmatch(r"[A-Za-z0-9._-]{1,80}", args.run_id):
        parser.error("run ID may contain only letters, digits, dot, underscore, hyphen")
    if not BOARD.is_file():
        print(f"FAIL: missing {BOARD}")
        return 1
    text = BOARD.read_text(encoding="utf-8")
    ids, errors = validate_board(text)
    if args.task not in ids:
        errors.append(f"task {args.task} is not on the board")
    if errors:
        for error in errors:
            print("FAIL:", error)
        return 1
    out = ROOT / "evidence" / "runs" / args.run_id / args.task
    out.mkdir(parents=True, exist_ok=False)
    manifest = {
        "task_id": args.task,
        "run_id": args.run_id,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "repository": str(ROOT),
        "verification": "task-board-integrity",
        "result": "PASS",
        "task_count": len(ids),
        "task_board": str(BOARD.relative_to(ROOT)),
        "checks": ["36 task rows", "unique IDs", "dependency references resolve",
                   "affected files and acceptance fields present"],
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"PASS: {len(ids)} tasks; evidence: {out.relative_to(ROOT)}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
