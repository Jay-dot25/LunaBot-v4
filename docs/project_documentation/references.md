# References

Primary/official material for context, plus repository sources. References contextualize established systems and standards; they do not imply LunaBot implements those systems. Accessed 2026-11-18.

## Planetary rover autonomy and sensing

1. NASA/JPL, “NASA’s Perseverance Now Autonomously Pinpoints Its Location on Mars,” 18 Feb 2026. [NASA](https://www.nasa.gov/missions/mars-2020-perseverance/perseverance-rover/nasas-perseverance-now-autonomously-pinpoints-its-location-on-mars/). Describes AutoNav and Mars Global Localization; use as high-level current context.
2. NASA/JPL, “NASA’s Self-Driving Perseverance Mars Rover ‘Takes the Wheel’,” 1 Jul 2021. [NASA](https://www.nasa.gov/solar-system/nasas-self-driving-perseverance-mars-rover-takes-the-wheel/). AutoNav, 3D terrain maps, obstacle routing and human rover-team role.
3. NASA/JPL Robotics, “Surface System Software and Rover Navigation” (Mars Science Laboratory). [JPL Robotics](https://www-robotics.jpl.nasa.gov/what-we-do/flight-projects/mars-science-laboratory/surface-system-software-and-rover-navigation/). Curiosity surface navigation, stereo hazard assessment, inertial/wheel fusion and visual odometry.
4. NASA Planetary Data System, “Mars Science Laboratory Rover Navigation Camera.” [PDS instrument context](https://pds.nasa.gov/ds-view/pds/viewContext.jsp?identifier=urn%3Anasa%3Apds%3Acontext%3Ainstrument%3Anavcam.msl&version=1.0). Instrument description and cited Navcam papers.
5. Maki, J.N. et al., “Mars Exploration Rover Engineering Cameras,” *Journal of Geophysical Research: Planets*, 108(E12), 8071 (2003). DOI: [10.1029/2003JE002077](https://doi.org/10.1029/2003JE002077).
6. Maki, J.N. et al., “The Mars Science Laboratory Engineering Cameras,” *Space Science Reviews* 170, 77–93 (2012). DOI: [10.1007/s11214-012-9882-4](https://doi.org/10.1007/s11214-012-9882-4).
7. NASA Science, “VIPER Rover and Instruments.” [NASA Science](https://science.nasa.gov/mission/viper/rover-and-instruments/). Official mission instrument overview; program status and plans may change, so avoid claiming a future mission outcome from this page.
8. NASA NTRS, “VIPER Rover Navigation and Perception Hardware / Navigation Overview,” technical briefing PDF. [NASA NTRS](https://ntrs.nasa.gov/api/citations/20230008814/downloads/TI-VIPER-Tech.pdf). Technical concept material on stereo cameras, lighting, short waypoint driving and lunar polar conditions.
9. NASA, “How Perseverance Drives on Mars.” [NASA Science](https://science.nasa.gov/resource/how-perseverance-drives-on-mars/). Official AutoNav summary.

## ROS, simulation and algorithms

10. ROS 2 Humble documentation, concepts and message ecosystem. [ROS documentation](https://docs.ros.org/en/humble/).
11. `slam_toolbox` ROS 2 package documentation and repository. [Steve Macenski / slam_toolbox](https://github.com/SteveMacenski/slam_toolbox). The project uses this package; precise installed version is recorded by `docs/phase-l-physical-qualification.md` as 2.6.10 for its qualification environment.
12. Nav2 documentation. [Navigation2](https://docs.nav2.org/). Comparison/future integration reference only; LunaBot current custom grid planner is not Nav2.
13. Open Robotics, Gazebo Sim documentation. [Gazebo documentation](https://gazebosim.org/docs/). Simulation platform reference.
14. S. Koenig and M. Likhachev, “D* Lite,” *Proceedings of the AAAI Conference on Artificial Intelligence* (2002), 476–483. [AAAI publication](https://aaai.org/papers/00476-aaai02-072-d-lite/). Algorithm context; LunaBot's candidate implementation is benchmark-only.
15. P.E. Hart, N.J. Nilsson and B. Raphael, “A Formal Basis for the Heuristic Determination of Minimum Cost Paths,” *IEEE Transactions on Systems Science and Cybernetics* 4(2), 100–107 (1968). DOI: [10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136). A* foundation.
16. T. Rösmann, F. Hoffmann and T. Bertram, “Integrated Online Trajectory Planning and Optimization in Distinctive Topologies,” *Robotics and Autonomous Systems* 88 (2017), 142–153. DOI: [10.1016/j.robot.2016.11.007](https://doi.org/10.1016/j.robot.2016.11.007). Background for local trajectory planners; not LunaBot's follower.

## Project primary sources

17. LunaBot repository `README.md`, `docs/architecture.md`, phase guides `docs/phase-a-launch-a.md` through `docs/phase-12-launch-l.md`, `docs/sensor-contracts.md`, `docs/slam-qualification.md`, `docs/odometry-qualification.md`, `docs/p6-calibration-plan.md`, `docs/p7-replan-benchmark.md`, `docs/phase-l-physical-qualification.md`, `docs/verification-matrix.md`.
18. LunaBot model/world: `src/lunabot_gazebo/models/lunabot_v4/model.sdf`, `src/lunabot_gazebo/worlds/lunar_world.sdf`.
19. LunaBot code: `scripts/astar_navigation.py`, `scripts/terrain_plan.py`, `scripts/terrain_costs.py`, `scripts/terrain_aware_planner.py`, `scripts/terrain_path_follower.py`, `scripts/obstacle_detector.py`, `scripts/mission_lifecycle.py`, `scripts/control_odometry.py` and the remaining scripts under `scripts/`.
20. LunaBot run evidence: paths listed in `LunaBot_Experiments_and_Results.md`; metrics follow the dated source reports and are explicitly scoped.

## Citation and comparison notes

NASA material describes operational systems at a public-summary level; use cited technical publications for specific algorithm or hardware claims. No uncited quantitative rover-to-rover performance comparison is asserted here. Confirm latest mission/program status before publication. Project code and local evidence should be cited with repository commit ID and immutable evidence hashes in a submitted paper.
