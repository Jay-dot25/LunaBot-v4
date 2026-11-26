#!/usr/bin/env python3
"""Evaluate Phase L navigation against isolated Gazebo truth."""
import bisect
import math

# Physical goal tolerance retains the original Phase L 0.35 m envelope.
# Localization thresholds are those of the existing P3-02 qualification.
GOAL_LIMIT = 0.35
POSITION_LIMIT = 0.25
HEADING_LIMIT = 0.15
MAX_SYNC_GAP = 0.10
MAX_SLAM_GAP = 0.50


def quaternion_yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def align_pose(source, reference, pose):
    """Map a pose into a reference frame using one paired origin."""
    rotation = angle_difference(reference[2], source[2])
    c, s = math.cos(rotation), math.sin(rotation)
    dx, dy = pose[0] - source[0], pose[1] - source[1]
    return (reference[0] + c * dx - s * dy,
            reference[1] + s * dx + c * dy,
            angle_difference(pose[2] + rotation, 0))


def position_error(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def evaluate(events, truth, expected_goals):
    """Align once before goal publication and score all completed goals."""
    if len(truth) < 2:
        raise ValueError("Gazebo truth samples are missing")
    times = sorted(truth)

    def truth_at(stamp):
        i = bisect.bisect_left(times, stamp)
        candidates = times[max(0, i - 1):i + 1]
        nearest = min(candidates, key=lambda t: abs(t - stamp))
        if abs(nearest - stamp) > MAX_SYNC_GAP:
            raise ValueError(f"no synchronized Gazebo truth at {stamp:.3f} simulation seconds")
        return truth[nearest]

    goals = [event for event in events if event["kind"] == "goal"]
    completions = [e for e in events if e["kind"] == "reached"]
    if not goals:
        raise ValueError("no navigation goals recorded")

    first_goal = goals[0]["sim_time"]
    slam = [e for e in events if e["kind"] == "slam"]
    odom = [e for e in events if e["kind"] == "odom"]
    initial_odom = next((e for e in odom if e["sim_time"] > 0), None)
    anchor = next((e for e in slam if 0 < e["sim_time"] < first_goal), None)
    if anchor is None or initial_odom is None:
        raise ValueError("missing pre-goal SLAM and odometry origin")
    truth_origin = truth_at(anchor["sim_time"])
    odom_truth_origin = truth_at(initial_odom["sim_time"])
    if (position_error(truth_origin, odom_truth_origin) > 0.05 or
            abs(angle_difference(truth_origin[2], odom_truth_origin[2])) > 0.05):
        raise ValueError("rover moved before the SLAM origin could be established")
    map_origin = anchor["pose"]

    results = []
    active_goal = None
    last_completed = None
    problems = []
    for event in events:
        if event["kind"] == "goal":
            if event["frame"] != "map" or event["sim_time"] <= 0:
                problems.append("goal requires a map frame and valid simulation clock")
                continue
            # P8 republishes an unchanged active goal while waiting for discovery.
            if active_goal is None and last_completed is not None and event["position"] == last_completed["position"]:
                continue
            if active_goal is None or event["position"] != active_goal["position"]:
                if active_goal is not None:
                    problems.append("goal was superseded before reaching it")
                active_goal = event
            continue
        if event["kind"] != "reached":
            continue
        if active_goal is None:
            if last_completed is None:
                problems.append("goal completion was received before its goal")
            continue
        try:
            actual = align_pose(truth_origin, map_origin, truth_at(event["sim_time"]))
        except ValueError as exc:
            problems.append(str(exc))
        else:
            error = position_error(actual, active_goal["position"])
            results.append(dict(sim_time=event["sim_time"], goal_map=active_goal["position"],
                                truth_map=actual, physical_goal_error_m=error,
                                pass_position=error <= GOAL_LIMIT))
        last_completed = active_goal
        active_goal = None

    end = completions[-1]["sim_time"] if completions else max(e["sim_time"] for e in events)
    slam_samples = [anchor, *(e for e in slam if anchor["sim_time"] < e["sim_time"] <= end)]
    gaps = [b["sim_time"] - a["sim_time"] for a, b in zip(slam_samples, slam_samples[1:])]
    max_slam_gap = max([end - slam_samples[-1]["sim_time"], *gaps])
    if len(slam_samples) < 2 or max_slam_gap > MAX_SLAM_GAP:
        problems.append("SLAM transform coverage is too sparse")

    def compare(samples, source, reference):
        errors = []
        for sample in samples:
            try:
                truth_pose = align_pose(source, reference, truth_at(sample["sim_time"]))
            except ValueError as exc:
                problems.append(str(exc))
                continue
            estimate = sample["pose"]
            errors.append((position_error(truth_pose, estimate),
                           abs(angle_difference(truth_pose[2], estimate[2]))))
        return dict(samples=len(errors), peak_position_error_m=max((e[0] for e in errors), default=None),
                    peak_heading_error_rad=max((e[1] for e in errors), default=None))

    slam_error = compare(slam_samples, truth_origin, map_origin)
    odom_error = compare([e for e in odom if 0 < e["sim_time"] <= end],
                         odom_truth_origin, initial_odom["pose"])
    if (not slam_error["samples"] or slam_error["peak_position_error_m"] > POSITION_LIMIT
            or slam_error["peak_heading_error_rad"] > HEADING_LIMIT):
        problems.append("SLAM exceeded the position or heading drift limit")
    if len(results) != expected_goals or active_goal is not None:
        problems.append(f"expected {expected_goals} completed goals, scored {len(results)}")
    if any(not goal["pass_position"] for goal in results):
        problems.append("Gazebo ground truth exceeded the physical goal tolerance")
    scans = [event for event in events if event["kind"] == "scan"]
    scan_metrics = dict(samples=len(scans),
                        valid_beams=sum(event["valid_beams"] for event in scans),
                        near_minimum=sum(event["near_minimum"] for event in scans))
    return dict(acceptance_pass=not problems, problems=list(dict.fromkeys(problems)),
                goals=results, slam=slam_error, wheel_odometry=odom_error,
                max_slam_gap_sec=max_slam_gap, scan=scan_metrics,
                limits=dict(goal_position_m=GOAL_LIMIT, slam_position_m=POSITION_LIMIT,
                            slam_heading_rad=HEADING_LIMIT, synchronization_sec=MAX_SYNC_GAP,
                            slam_gap_sec=MAX_SLAM_GAP))
