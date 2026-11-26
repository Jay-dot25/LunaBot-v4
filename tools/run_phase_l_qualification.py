#!/usr/bin/env python3
"""Launch a fresh Phase L mission and score its physical accuracy offline."""
import argparse
from contextlib import contextmanager, ExitStack
import csv
import fcntl
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import uuid

from phase_l_accuracy import evaluate
from prepare_phase_l_world import prepare
from run_odom_qualification import ROOT, build_recorder
from verify_p8_acceptance import verify_trial


@contextmanager
def capture(command, log, env):
    with log.open("w", encoding="utf-8") as stream:
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            yield process
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


def score(run, expected):
    with (run / "gazebo_truth.csv").open(newline="", encoding="utf-8") as stream:
        truth = {float(row["sim_time"]): tuple(float(row[key]) for key in ("x", "y", "yaw"))
                 for row in csv.DictReader(stream)}
    events = [json.loads(line) for line in
              (run / "navigation.jsonl").read_text(encoding="utf-8").splitlines()]
    return evaluate(events, truth, expected)



def qualify(run, mode, timeout_sec, renderer=None, slam_config=None):
    run.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, EVIDENCE_DIR=str(run), EVIDENCE="1",
               ROS_DOMAIN_ID=os.environ.get("ROS_DOMAIN_ID", "42"),
               ROS_LOCALHOST_ONLY="1", FASTDDS_BUILTIN_TRANSPORTS="UDPv4",
               IGN_PARTITION=f"lunabot-qualification-{uuid.uuid4().hex}",
               P8_ROUNDTRIP="0", P8_MAP_QUALIFICATION="0", AUTO_GOAL="false",
               FINAL_DEMO="0", PRESENTATION_FAST="0", HEADLESS="1", DEMO="1")
    if mode == "manual":
        env.update(HEADLESS="0", DEMO="0", FINAL_DEMO="1", GAZEBO_SERVER_ONLY="1",
                   REQUIRE_MANUAL_GOAL="true", PRESENTATION_FAST="1",
                   RETAIN_RUNTIME_GATES_BETWEEN_GOALS="true")
        if "microsoft" in os.uname().release.lower():
            env.setdefault("WSL_SOFTWARE_PROFILE", "1")
    elif mode == "p8":
        env.update(P8_ROUNDTRIP="1", P8_RUN_ID=run.name + "-" + uuid.uuid4().hex[:8])
    else:
        env.update(PRESENTATION_FAST="1")
    if renderer:
        world = ROOT / "src/lunabot_gazebo/worlds/lunar_world.sdf"
        variant = run / world.name
        prepare(world, variant, renderer)
        env["WORLD_PATH_OVERRIDE"] = str(variant)

    if slam_config:
        override = run / "slam_override.yaml"
        override.write_bytes(slam_config.read_bytes())
        env["SLAM_CONFIG_OVERRIDE"] = str(override)

    command = ["timeout", "--signal=TERM", "--kill-after=60",
               str(timeout_sec), "bash", str(ROOT / "launch-l")]
    (run / "configuration.json").write_text(json.dumps(
        {"mode": mode, "renderer_override": renderer, "slam_config": str(slam_config) if slam_config else None,
         "command": command,
         "ros_domain_id": env["ROS_DOMAIN_ID"]}, indent=2) + "\n", encoding="utf-8")

    recorder = run / "capture_gazebo_truth"
    build_recorder(recorder)
    native = [str(recorder), str(run / "gazebo_truth.csv"),
              "/world/lunar_world/dynamic_pose/info"]
    ros = [sys.executable, str(ROOT / "tools/capture_phase_l_navigation.py"),
           str(run / "navigation.jsonl"), "--ros-args", "-p", "use_sim_time:=true"]
    with ExitStack() as stack:
        captures = [stack.enter_context(capture(cmd, run / log, env))
                    for cmd, log in ((native, "truth_recorder.log"), (ros, "ros_recorder.log"))]
        print(f"Phase L {mode} qualification evidence: {run}", flush=True)
        with (run / "console.log").open("w", encoding="utf-8") as log:
            result = subprocess.run(command, cwd=ROOT, env=env, stdout=log,
                                    stderr=subprocess.STDOUT, check=False)
        recorder_failed = any(process.poll() is not None for process in captures)

    try:
        report = score(run, expected=3 if mode == "p8" else 1)
    except (ValueError, OSError, KeyError) as exc:
        report = {"acceptance_pass": False, "problems": [str(exc)]}
    if mode == "p8":
        try:
            delivery = verify_trial(run, run.name)
        except (ValueError, OSError, KeyError, sqlite3.Error) as exc:
            report["problems"].append(str(exc))
        else:
            report["delivery_id"] = delivery["delivery_id"]
    report.update(mode=mode, launcher_exit=result.returncode)
    if recorder_failed:
        report["problems"].append("a required recorder exited during the mission")
    if result.returncode:
        report["problems"].append(f"launcher exited with status {result.returncode}")
    if report["problems"]:
        report["acceptance_pass"] = False
    serialized = json.dumps(report, indent=2, allow_nan=False) + "\n"
    (run / "qualification.json").write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0 if report["acceptance_pass"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--mode", choices=("headless", "manual", "p8"), default="headless")
    parser.add_argument("--timeout-sec", type=int)
    parser.add_argument("--repeat", type=int, default=1,
                        help="run this many cold trials in separate subdirectories")
    parser.add_argument("--renderer", choices=("ogre", "ogre2"))
    parser.add_argument("--slam-config", type=Path)
    args = parser.parse_args()
    if (args.timeout_sec is not None and args.timeout_sec <= 0) or args.repeat <= 0:
        parser.error("--timeout-sec and --repeat must be positive")
    if args.repeat > 1 and args.mode == "manual":
        parser.error("manual qualification requires a separate operator launch for each trial")
    timeout_sec = args.timeout_sec or (1200 if args.mode == "p8" else 360)
    build = ROOT / ".build"
    build.mkdir(exist_ok=True)
    with (build / "phase-l-qualification.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("another Phase L qualification is running in this checkout")
        run = args.run_dir.resolve()
        if args.repeat == 1:
            return qualify(run, args.mode, timeout_sec, args.renderer, args.slam_config)
        run.mkdir(parents=True, exist_ok=False)
        results = []
        for i in range(1, args.repeat + 1):
            trial = run / f"trial-{i:02d}"
            code = qualify(trial, args.mode, timeout_sec, args.renderer, args.slam_config)
            results.append({"trial": trial.name, "acceptance_pass": code == 0})
        passed = all(trial["acceptance_pass"] for trial in results)
        (run / "qualification.json").write_text(json.dumps(
            {"acceptance_pass": passed, "trials": results}, indent=2) + "\n",
            encoding="utf-8")
        return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
