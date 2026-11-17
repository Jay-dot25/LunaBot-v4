# P3-02 SLAM mapping and localization qualification

Status: **Completed with bounded but marginal absolute localization accuracy.** P3-02 proves headless mapping and saved-map localization without RViz. The measured 0.230 m position error leaves 0.020 m margin to the qualification bound; improve localization before relying on it for tighter navigation tolerances.

## Scope and truth boundary

This qualification uses the existing Gazebo world, GPU LiDAR, DiffDrive wheel odometry, and slam_toolbox. Evaluator truth is captured through a separate Gazebo transport recorder and is never published into ROS or used by the operational localizer. No second estimator was added. Existing A*, simulation assets, and launch entry points remain in place.

## Qualified runtime profile

- Headless launch uses Gazebo Fortress with OGRE2 and LIBGL_ALWAYS_SOFTWARE=1 in this WSL environment. No RViz process is required.
- Camera/depth streams use 160x120 in the qualification profile; depth measured about 12.2 Hz with maximum inter-message gap 0.143 s. This reduced diagnostic profile needs benchmarking before final dashboard/perception selection.
- slam_toolbox mapping uses scan throttle 1, maximum scan range and scan-buffer distance 15 m, minimum travel distance 0.02 m, and minimum heading 0.05 rad. These keep the map bounded and refresh map-to-odom during motion.
- The saved occupancy YAML uses free_thresh 0.19. The map_saver PGM uses gray 205 for unknown. Its default 0.25 threshold reloaded those cells as free, so launch-c now adjusts below 50/255. A fresh map_server reload confirms unknown cells remain unknown.

## Evidence

Primary run tree: evidence/runs/20261103-p302/.

- loop-mapping: controlled headless loop route; truth ended at (0.685870, 0.051806) m, yaw -0.045386 rad. DiffDrive odometry ended at (0.687814, 0.051712) m, yaw -0.068524 rad for this route.
- Live scan at simulation time 190 s: 720 beams, 403 valid returns between 1.1839 m and 55.1891 m. See localization-loop-final/scan_summary.txt.
- Saved map: 117x584 cells, 0.05 m/cell. Fresh nav2_map_server configure/activate reload yielded 38,860 known cells, including 37,617 free and 1,243 occupied; 29,468 cells remained unknown. See loop-mapping/saved_map_reload.txt.
- Saved slam_toolbox pose graph: 11,204,648-byte .posegraph and 719,124-byte .data. A fresh SLAM_MODE=localization launch loaded it with match_type=3, verified a non-empty map, and passed headless topic/TF checks. See localization-loop-final/slam.log and last_run.log.
- Cold localization at the mapped origin reported map-to-chassis (0,0,0), matching the independent Gazebo start pose.
- During a bounded 0.08 m/s straight test, truth at sim 100.602 s was (0.292844, -0.000093) m, yaw -0.000053 rad. At 100.606 s, map-to-chassis was (0.063, -0.004) m, yaw about 0.000 rad. Position error was 0.230 m; yaw error was 0.000053 rad. Both meet bounds of 0.25 m and 0.15 rad, with small position margin.
- A relocalization seed shifted to x=0.16 m (0.10 m from scan-matched x=0.063 m) was published on /initialpose; after scan processing the estimate returned to (0.063, -0.004) m. Truth remained isolated from ROS.
- Safety health reported no health or motion fault after the test. The expected watchdog stop after input expiry was observed; no movement continued after zero/timeout.

## Initial failure and demonstrated fix

The first headless run under OGRE returned all 720 LiDAR ranges at the 0.15 m minimum and produced a 6x6 grid with no known cells. A minimal wall-world reproduced the failure under OGRE. With OGRE2 plus software OpenGL, the same wall returned 92/360 valid beams at about 2.90 m. The full rover under that renderer produced the 403/720 live returns and non-empty map above.

OGRE2 under the hardware-selected D3D12 renderer aborts in GL3PlusTextureGpu::copyTo. The working WSL test profile therefore sets LIBGL_ALWAYS_SOFTWARE=1. This renderer limitation is environment-specific and does not make RViz a dependency.

## Acceptance matrix

| Gate | Result |
|---|---|
| Headless Phase C starts without RViz | PASS |
| Valid LiDAR returns and map updates during controlled drive | PASS: 403/720 valid; non-empty map |
| Occupancy YAML/PGM save and independent reload | PASS: 117x584, 0.05 m/cell; unknown/free/occupied preserved |
| slam_toolbox pose-graph save and fresh-process reload | PASS: map verified after match_type=3 load |
| map-to-odom and map-to-chassis localization TF | PASS |
| Absolute localization bound | PASS with limited margin: 0.230 m <= 0.25 m; yaw 0.000053 rad <= 0.15 rad |
| Deliberate initial-pose offset recovery | PASS: 0.10 m seed perturbation scan-matched back to x=0.063 m |
| Evaluator truth isolated from operational localization | PASS |

## Verification commands

Run the syntax check from the repository and run the SDF checks from /tmp so generated checker files stay outside the repository:

    cd /home/jay/lunabot-v4 && bash -n scripts/launch-c.sh
    cd /tmp && ign sdf -k /home/jay/lunabot-v4/src/lunabot_gazebo/worlds/lunar_world.sdf
    cd /tmp && ign sdf -k /home/jay/lunabot-v4/src/lunabot_gazebo/models/lunabot_v4/model.sdf

The mapping and localization runtime runs are already captured under evidence/runs/20261103-p302/. A*, GUI visualization, autonomous mission, wider-route accuracy, and higher-resolution sensor performance remain separate tasks.
