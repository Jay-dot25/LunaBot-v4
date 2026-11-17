#!/usr/bin/env python3
"""Run one fresh, headless DiffDrive versus synchronized Gazebo truth trial."""
import argparse
import json
import os
import shlex
import signal
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RECORDER_SOURCE = ROOT / "tools/capture_gazebo_truth.cc"


def build_recorder(output):
    flags = subprocess.run(
        ["pkg-config", "--cflags", "--libs", "ignition-transport11", "ignition-msgs8"],
        check=True, capture_output=True, text=True).stdout
    command = ["g++", "-O2", "-std=c++17", "-pthread",
               str(RECORDER_SOURCE), "-o", str(output), *shlex.split(flags)]
    subprocess.run(command, check=True, cwd=ROOT)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--model")
    parser.add_argument("--world")
    parser.add_argument("--turn-speed", type=float, default=0.6)
    parser.add_argument("--timeout-sec", type=int, default=150)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    run_dir.mkdir(parents=True, exist_ok=False)
    recorder = Path("/tmp/capture_gazebo_truth")
    if not recorder.exists():
        build_recorder(recorder)

    truth_path = run_dir / "gazebo_truth.csv"
    recorder_log = (run_dir / "truth_recorder.log").open("w", encoding="utf-8")
    process = subprocess.Popen(
        [str(recorder), str(truth_path)], cwd=ROOT, stdout=recorder_log,
        stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(1.0)
    env = dict(os.environ, HEADLESS="1", DEMO="1",
               DEMO_TURN_SPEED=str(args.turn_speed),
               EVIDENCE_DIR=str(run_dir))
    if args.model:
        env["LUNABOT_MODEL_PATH"] = str(Path(args.model).resolve())
    if args.world:
        env["LUNABOT_WORLD_PATH"] = str(Path(args.world).resolve())
    command = ["timeout", "--signal=INT", "--kill-after=15",
               str(args.timeout_sec), "bash", str(ROOT / "scripts/launch-b.sh")]
    (run_dir / "run_command.json").write_text(json.dumps({
        "command": command, "headless": True, "demo": True,
        "turn_speed_rad_s": args.turn_speed, "model": env.get("LUNABOT_MODEL_PATH"),
        "world": env.get("LUNABOT_WORLD_PATH"),
        "truth_recorder": str(recorder),
    }, indent=2) + "\n", encoding="utf-8")
    try:
        with (run_dir / "launcher.log").open("w", encoding="utf-8") as log:
            result = subprocess.run(command, cwd=ROOT, env=env, stdout=log,
                                    stderr=subprocess.STDOUT)
    finally:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)
        recorder_log.close()
    (run_dir / "launcher_exit.txt").write_text(
        f"{result.returncode}\n", encoding="utf-8")
    if not truth_path.exists() or truth_path.stat().st_size == 0:
        raise RuntimeError(f"truth recorder did not produce data: {truth_path}")
    analyze = subprocess.run(
        ["python3", str(ROOT / "tools/analyze_odom_truth.py"),
         str(run_dir), "--output", str(run_dir / "qualification.json")],
        cwd=ROOT)
    print(f"launcher_exit={result.returncode} qualification_exit={analyze.returncode}")
    return analyze.returncode


if __name__ == "__main__":
    raise SystemExit(main())
