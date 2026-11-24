# Fault-Tolerant TurtleBot3 Navigation (ROS 2)

A ROS 2 Humble / Nav2 simulation workspace demonstrating fault-tolerant autonomous mobile robot navigation through sensor fault injection, EKF state estimation, adaptive speed supervision, and waypoint execution.

> **Project source:** `turtlebot_nav_ws/`  
> **Technical report:** `docs/Fault-Tolerant-Autonomous-Mobile-Robot-Technical-Report.pdf`  
> **Middleware:** ROS 2 Humble · Gazebo · Nav2 · robot_localization · RViz2

## Project overview

This workspace is organized into three ROS 2 packages:

| Package | Responsibility |
|---|---|
| `bot_description` | Robot model package and description build metadata |
| `bot_localization` | EKF configuration, AMCL/map configuration, Nav2 parameters, map and simulation launch |
| `bot_navigation` | Fault injector, fault supervisor, waypoint navigator and launch/configuration files |

The simulation pipeline is:

`Gazebo sensors → fault_injector → EKF / Nav2 → fault_supervisor → /speed_limit → Nav2 controller → /cmd_vel`

The waypoint navigator sequences mission goals through Nav2's `BasicNavigator` / waypoint-following interface.

## Key capabilities

- **Sensor fault injection:** LiDAR blackout, Gaussian LiDAR range noise, and wheel-slip odometry corruption.
- **State estimation:** EKF fuses wheel linear velocity with IMU heading/angular-rate information. The report describes Mahalanobis innovation gating for rejecting outliers.
- **Global and local navigation:** AMCL/map server and Nav2 planning/controller layers.
- **Supervisory fault management:** sensor watchdogs and kinematic residual checks select degraded operating states and publish dynamic velocity limits on `/speed_limit`.
- **Mission execution:** dynamic waypoint sequencing with lifecycle readiness checks.
- **Telemetry:** CSV logging of elapsed time, supervisor mode, speed percentage, path length, odometry–EKF drift and cumulative control effort.

## Architecture and interfaces

| Node / subsystem | Important interfaces | Role |
|---|---|---|
| Gazebo | Publishes `/scan`, `/odom`, `/imu`, `/clock`; subscribes `/cmd_vel` | Differential-drive simulation and sensor generation |
| `fault_injector` | Subscribes `/scan`, `/odom`; publishes `/scan_faulty`, `/odom_faulty` | Injects configured sensor faults |
| `ekf_filter_node` | Uses `/odom_faulty` and `/imu`; publishes `/odometry/filtered` | Fused state estimate |
| AMCL / map server | Uses `/scan`, `/initialpose`; publishes map-to-odom TF | Global localization |
| Nav2 | Uses `/scan_faulty`, `/speed_limit`; publishes `/cmd_vel` | Global planning and local trajectory control |
| `fault_supervisor` | Monitors faulty sensors, IMU, filtered odometry and command velocity; publishes `/speed_limit` | Detects faults and applies speed envelopes |
| `waypoint_navigator` | Uses clicked/dynamic waypoints and Nav2 waypoint action | Mission sequencing |

### TF authority

The report specifies the transform chain:

`map → odom → base_footprint → base_scan`

It sets `publish_tf: false` in the EKF configuration so the Gazebo differential-drive plugin remains the authority for `odom → base_footprint`, while AMCL publishes `map → odom`.

## Fault detection and supervisory states

The technical report documents these detection conditions:

- LiDAR watchdog: scan-message age greater than **0.5 s**.
- Wheel-slip residuals: `|ωodom − ωimu| > 0.35 rad/s` or `|vodom − vekf| > 0.12 m/s`.
- Mahalanobis innovation gating: configured rejection thresholds of **2.0** for odometry twist and **2.5** for IMU angular velocity.

| Supervisor state | Reported speed cap | Interpretation |
|---|---:|---|
| `NOMINAL_BALANCED` | 100% | Healthy sensors, balanced profile |
| `NOMINAL_ECO` | 65% | Healthy sensors, energy-conservation profile |
| `DEGRADED_WHEEL_SLIP` | 45% | Kinematic residual indicates wheel slip |
| `DEGRADED_LIDAR_LOSS` | 30% | LiDAR watchdog timeout |

The supervisor applies the configured speed percentage through Nav2's `/speed_limit` interface. The report's LiDAR-blackout experiment also records a 30% cap and restoration to 100% after scan recovery.

## Experimental results (as reported)

Six scenarios are described in the report:

1. **Nominal baseline:** 100% speed; odometry–EKF drift reported below 0.03 m.
2. **LiDAR blackout:** approximately 5 s scan dropout; watchdog detects the loss within 0.5 s; speed falls to 30% and returns to 100% after recovery.
3. **LiDAR noise:** Gaussian range noise with σ = 0.25 m; report states local costmap filtering preserved collision-free waypoint tracking.
4. **Wheel slip:** wheel linear velocity scaled by 0.35× and yaw rate by 1.8×; raw odometry reaches 0.51 m peak drift (0.38 m steady-state), while the EKF/supervisor response rejects corrupted measurements and caps speed at 45%.
5. **Pop-up obstacle:** an unmapped static box is spawned on the active route; Nav2 updates the local costmap and re-routes without collision.
6. **Eco optimization:** speed capped at 65%; cumulative quadratic control effort grows more slowly.

The report defines cumulative control effort as (E = \int (v^2 + \omega^2)\,dt). It is a control-effort proxy, not a direct electrical battery-energy measurement.

## Screenshots and figures

The images below are extracted from the supplied technical report. Captions identify the original report page.

### Gazebo simulation — report page 2

![Gazebo TurtleBot3 simulation with LiDAR rays](docs/images/gazebo-simulation.png)

### Global costmap and AMCL — report page 4

![Global costmap and AMCL localization](docs/images/global-costmap-amcl.png)

### Local trajectory and TF — report page 4

![Local trajectory and transform frames](docs/images/local-trajectory-tf.png)

### Multi-scenario telemetry — report page 6

![Fault-tolerant navigation telemetry comparison](docs/images/telemetry-report.png)

The workspace also includes the supplied `fault_tolerance_comparison.png` telemetry plot under `turtlebot_nav_ws/`.

## Requirements

The supplied workspace targets:

- Ubuntu with ROS 2 Humble
- Gazebo and TurtleBot3 simulation packages
- Nav2 (`nav2_bringup` and Nav2 RViz plugins)
- `robot_localization`
- `slam_toolbox` is not required by the report's AMCL-based localization path

Install the ROS packages appropriate to your ROS 2 Humble environment. At minimum, verify that `ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-turtlebot3-gazebo`, `ros-humble-robot-localization`, and `ros-humble-turtlebot3` are available.

## Build and run

Clone the repository:

```bash
git clone https://github.com/abdur-netizen/ros2-turtlebot3-fault_tolerance.git
cd ros2-turtlebot3-fault_tolerance/turtlebot_nav_ws
```

Source ROS 2 Humble, install dependencies, and build:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Launch the simulation, EKF, map/AMCL, Nav2 and RViz2:

```bash
ros2 launch bot_localization navigation_sim.launch.py
```

In a second terminal, source the same ROS and workspace setup, then launch fault injection, supervision and waypoint navigation:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2-turtlebot3-fault_tolerance/turtlebot_nav_ws/install/setup.bash
ros2 launch bot_navigation fault_tolerant_nav.launch.py
```

The included launch file starts with `fault_mode: none` and `opt_profile: balanced`. Inspect the node parameter declarations in the source before selecting other fault modes or profiles. Use a separate terminal to inspect topics and node health:

```bash
ros2 topic list
ros2 node list
ros2 topic echo /speed_limit
ros2 topic echo /odometry/filtered
```

> **Important:** The technical report documents the experiments and measured outcomes; those results are not a guarantee that every machine will reproduce identical timing or telemetry. Confirm package availability, launch success, topic names and fault parameters on the target ROS installation. The supplied archive did not include generated `build/`, `install/` or `log/` directories, so rebuild locally.

## Repository contents

- `turtlebot_nav_ws/src/` — ROS 2 package source, launch files, YAML configuration, map and tests
- `turtlebot_nav_ws/*.csv` — supplied mission/scenario telemetry
- `turtlebot_nav_ws/plot_mission_metrics.py` — telemetry plotting utility
- `turtlebot_nav_ws/fault_tolerance_comparison.png` — supplied comparison plot
- `turtlebot_nav_ws/rrrr.rviz` — supplied RViz configuration
- `docs/` — original technical report and report figures

Generated build outputs, runtime logs, Python bytecode caches and other transient artifacts are intentionally excluded.

## Future work

The report proposes a three-sample persistence/hysteresis filter to reduce brief turn-slip false positives and dynamic scaling of local costmap inflation radius based on EKF covariance trace.

## Reference

*Fault-Tolerant Sensor Fusion, Motion Planning, and Autonomous Navigation in ROS 2*, technical report for `turtlebot_nav_ws`, dated 24 November 2026, supplied with this project.
