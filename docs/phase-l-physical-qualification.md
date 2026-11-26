# Phase L physical qualification

The qualification runner launches the normal Phase L entry point in a fresh
Gazebo Transport partition and writes an acceptance report for each trial.
Stop other LunaBot simulations before running it. Phase L's legacy startup
cleanup searches for lunar-world and ROS processes by name.

## Setup

Use Ubuntu 22.04, ROS 2 Humble, and Gazebo Fortress. The commands assume the
ROS and Gazebo package repositories have been configured. Install these
packages in addition to the dependencies in the project README.

```bash
sudo apt install ignition-fortress ros-humble-ros-ign-bridge \
  ros-humble-slam-toolbox ros-humble-nav2-map-server ros-humble-rviz2 \
  g++ cmake make pkg-config libignition-transport11-dev libignition-msgs8-dev \
  python3-pytest python3-numpy python3-pil python3-venv
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .build/qualification-venv
.build/qualification-venv/bin/python -m pip install pyarrow==17.0.0
.build/qualification-venv/bin/python -m pip install \
  --index-url https://download.pytorch.org/whl/cpu torch==2.5.1
.build/qualification-venv/bin/python -m pytest -q
```

The simulation runs used Fortress 1.0.3, Ignition Transport 11.4.2, Ignition
Messages 8.7.1, slam_toolbox 2.6.10, and Nav2 map server 1.1.20. Use
`dpkg-query -W` to compare the installed package versions when reproducing a
result. The virtual environment uses the Ubuntu system Python to preserve ROS
Humble's Python extension compatibility.

## Run the qualification

From the checkout root, source ROS and use a new evidence directory on each
invocation. Run modes sequentially.

```bash
source /opt/ros/humble/setup.bash
run_id="$(date -u +%Y%m%dT%H%M%SZ)"

python3 tools/run_phase_l_qualification.py \
  --run-dir "evidence/runs/$run_id-headless" --repeat 3

python3 tools/run_phase_l_qualification.py --mode manual \
  --run-dir "evidence/runs/$run_id-manual" --timeout-sec 600

python3 tools/run_phase_l_qualification.py --mode p8 \
  --run-dir "evidence/runs/$run_id-p8" --repeat 3

python3 tests/run_safety_faults.py --evidence "evidence/runs/$run_id-safety"
```

In manual mode, wait for the occupancy grid and select a goal in connected
mapped free space using **2D Goal Pose** in RViz. P8 validates the committed
Habitat–Science–Habitat map binding and runs the standard mission and delivery
gates. The separate live safety test exercises the Phase B controller boundary,
sensor and clock loss, actuator timeout, restart inhibition, and missing
localization. Phase L follower tests cover unsafe and stale obstacle maps.

Use `--renderer ogre2` or `--slam-config /absolute/path/to/config.yaml`
for A/B experiments. These copy the override into the trial's evidence
directory for inspection.

## Read the result

A successful launcher exit is one required check. The evaluator also requires
every Gazebo goal miss to be at most **0.35 m**, peak SLAM position error to be
at most **0.25 m**, and peak SLAM heading error to be at most **0.15 rad**.
The physical limit preserves the original Phase L goal envelope. The follower
now stops within 0.20 m of the estimated goal to leave a localization margin.
The SLAM limits come from Phase C qualification. Phase L goals currently
qualify position only.

The Gazebo Transport recorder writes simulation-stamped model poses directly
to CSV. A separate ROS subscriber records map-frame transforms, wheel
odometry, goal and completion events, and scan counts. The evaluator aligns
the frames once before the first goal, checks timestamp coverage, and rejects
missing or stale samples. Gazebo truth stays outside ROS and the navigation
graph. The report includes wheel odometry errors for comparison.

Review `qualification.json`, `navigation.jsonl`, `gazebo_truth.csv`, and
`console.log` in a trial directory. A repeated run returns failure if any
trial fails and stores each trial report separately.
