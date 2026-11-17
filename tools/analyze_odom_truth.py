#!/usr/bin/env python3
"""Compare the Phase B odom trace against evaluator-only Gazebo truth."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def yaw_delta(a, b):
    return math.atan2(math.sin(b - a), math.cos(b - a))


def nearest(truth, times, stamp):
    i = bisect.bisect_left(times, stamp)
    candidates = [j for j in (i - 1, i) if 0 <= j < len(truth)]
    j = min(candidates, key=lambda n: abs(times[n] - stamp))
    return truth[j], abs(times[j] - stamp)


def pose_truth(row):
    return (float(row["x"]), float(row["y"]), float(row["yaw"]))


def pose_odom(row):
    return (float(row["x"]), float(row["y"]), float(row["yaw"]))


def qualify(run_dir):
    run = Path(run_dir)
    odom = read_csv(run / "diag_drive.csv")
    truth = read_csv(run / "gazebo_truth.csv")
    if len(odom) < 10 or len(truth) < 10:
        raise ValueError("insufficient odometry or truth samples")
    truth.sort(key=lambda row: float(row["sim_time"]))
    times = [float(row["sim_time"]) for row in truth]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("truth simulation clock is not strictly monotonic")

    # Require truth to cover every command sample; use nearest simulation stamp.
    aligned = []
    for row in odom:
        sample, dt = nearest(truth, times, float(row["t_sim"]))
        aligned.append((row, sample, dt))
    max_dt = max(item[2] for item in aligned)

    segments = {}
    for label, field in (("straight", "cmd_v"), ("turn", "cmd_wz")):
        active = [item for item in aligned if abs(float(item[0][field])) > 0.05]
        if len(active) < 10:
            raise ValueError(f"missing active {label} segment")
        first, last = active[0], active[-1]
        o0, o1 = pose_odom(first[0]), pose_odom(last[0])
        t0, t1 = pose_truth(first[1]), pose_truth(last[1])
        odom_distance = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
        truth_distance = math.hypot(t1[0] - t0[0], t1[1] - t0[1])
        odom_yaw = yaw_delta(o0[2], o1[2])
        truth_yaw = yaw_delta(t0[2], t1[2])
        yaw_error = abs(yaw_delta(truth_yaw, odom_yaw))
        segments[label] = {
            "sample_count": len(active),
            "sim_start": float(first[0]["t_sim"]),
            "sim_end": float(last[0]["t_sim"]),
            "endpoint_truth_alignment_error_sec": max(first[2], last[2]),
            "odom_distance_m": odom_distance,
            "truth_distance_m": truth_distance,
            "distance_error_m": abs(odom_distance - truth_distance),
            "distance_error_limit_m": max(0.10, 0.10 * truth_distance),
            "odom_yaw_rad": odom_yaw,
            "truth_yaw_rad": truth_yaw,
            "yaw_error_rad": yaw_error,
        }
    # Turn gate applies only to the commanded in-place turn; straight yaw is a
    # diagnostic measure and should be close to zero as well.
    reset_pose = pose_odom(odom[0])
    odom_messages = read_csv(run / "odometry_samples.csv")
    stamps = [float(row["sim_time"]) for row in odom_messages]
    result = {
        "run_dir": str(run),
        "odom_samples": len(odom_messages),
        "truth_samples": len(truth),
        "max_nearest_truth_alignment_sec": max_dt,
        "truth_stamp_alignment_pass": max_dt <= 0.02,
        "odom_start_pose": {"x_m": reset_pose[0], "y_m": reset_pose[1], "yaw_rad": reset_pose[2]},
        "fresh_run_origin_pass": math.hypot(reset_pose[0], reset_pose[1]) <= 0.02 and abs(reset_pose[2]) <= 0.02,
        "odom_stamp_monotonic_pass": all(b > a for a, b in zip(stamps, stamps[1:])),
        "segments": segments,
    }
    straight = segments["straight"]
    turn = segments["turn"]
    result["straight_gate_pass"] = straight["distance_error_m"] <= straight["distance_error_limit_m"]
    result["turn_gate_pass"] = turn["yaw_error_rad"] <= 0.10
    result["acceptance_pass"] = all((
        result["truth_stamp_alignment_pass"],
        result["fresh_run_origin_pass"],
        result["odom_stamp_monotonic_pass"],
        result["straight_gate_pass"],
        result["turn_gate_pass"],
    ))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = qualify(args.run_dir)
    serialized = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0 if result["acceptance_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
