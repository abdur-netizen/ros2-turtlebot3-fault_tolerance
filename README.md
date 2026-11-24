# Fault-Tolerant TurtleBot3 Navigation (ROS 2)

A ROS 2 Humble / Nav2 project focused on resilient autonomous mobile-robot navigation under simulated sensor faults. The repository contains the core ROS 2 node implementations, selected launch/configuration files, and a report-derived technical summary.

**Technical report summary:** [docs/TECHNICAL_REPORT_SUMMARY.md](docs/TECHNICAL_REPORT_SUMMARY.md)  
**Middleware target:** ROS 2 Humble · Gazebo · Nav2 · robot_localization · RViz2

## Project overview

The project is organized around three ROS 2 packages:

| Package | Responsibility |
|---|---|
| `bot_description` | Robot description package metadata |
| `bot_localization` | EKF configuration, AMCL configuration, map metadata and simulation launch |
| `bot_navigation` | LiDAR/odometry fault injection, adaptive fault supervision, telemetry logging and dynamic waypoint execution |

Conceptual data flow:

`Gazebo sensors → fault_injector → EKF / Nav2 → fault_supervisor → /speed_limit → Nav2 controller → /cmd_vel`

The waypoint navigator receives dynamic goals from `/clicked_point` and `/dynamic_waypoints`, then dispatches routes through Nav2's `BasicNavigator`.

## Implemented capabilities

- LiDAR blackout and Gaussian range-noise injection.
- Wheel-slip odometry corruption (linear velocity scale and yaw-rate multiplier).
- EKF sensor-fusion configuration using wheel velocity and IMU information.
- Scan-age watchdog and wheel/IMU/EKF kinematic residual checks.
- Adaptive Nav2 speed limits through `/speed_limit`.
- Dynamic waypoint updates, route pre-emption and mission feedback.
- CSV telemetry for path length, odometry–EKF drift, covariance trace and cumulative control effort.

## Fault thresholds and speed envelopes

| Mode | Trigger | Speed limit |
|---|---|---:|
| `NOMINAL_BALANCED` | Healthy sensors, balanced profile | 100% |
| `NOMINAL_ECO` | Healthy sensors, eco profile | 65% |
| `DEGRADED_WHEEL_SLIP` | `|ωodom − ωimu| > 0.35 rad/s` or `|vodom − vekf| > 0.12 m/s` | 45% |
| `DEGRADED_LIDAR_LOSS` | LiDAR message age greater than 0.5 s | 30% |

The report lists EKF Mahalanobis rejection thresholds of 2.0 for odometry twist and 2.5 for IMU angular velocity.

## Reported simulation scenarios

The supplied report documents six runs: nominal baseline, LiDAR blackout, LiDAR noise (σ = 0.25 m), wheel slip (linear velocity ×0.35 and yaw rate ×1.8), pop-up unmapped obstacle, and eco optimization.

Key report-reported outcomes include:
- Nominal odometry–EKF drift below 0.03 m.
- LiDAR blackout detection within 0.5 s, with speed reduced to 30% and restored after sensor recovery.
- Wheel-slip raw odometry peak drift of 0.51 m (0.38 m steady-state), with the supervisor applying a 45% cap.
- Eco mode applying a 65% cap and reducing the slope of cumulative control effort.

These are results stated in the supplied report, not a claim of independent re-testing. The report's cumulative control effort (E = \int(v^2 + \omega^2)dt) is a control-effort proxy, not a direct battery-energy measurement.

## Source files in this repository

- `turtlebot_nav_ws/src/bot_navigation/bot_navigation/fault_injector.py`
- `turtlebot_nav_ws/src/bot_navigation/bot_navigation/fault_supervisor.py`
- `turtlebot_nav_ws/src/bot_navigation/bot_navigation/waypoint_navigator.py`
- `turtlebot_nav_ws/src/bot_navigation/launch/fault_tolerant_nav.launch.py`
- `turtlebot_nav_ws/src/bot_navigation/config/waypoints.yaml`
- `turtlebot_nav_ws/src/bot_localization/config/ekf.yaml`
- `turtlebot_nav_ws/src/bot_localization/config/amcl.yaml`
- `turtlebot_nav_ws/src/bot_localization/launch/navigation_sim.launch.py`
- `turtlebot_nav_ws/src/bot_localization/maps/virtual_world.yaml`
- `docs/TECHNICAL_REPORT_SUMMARY.md`

> **Workspace completeness:** This commit adds the core ROS 2 nodes and selected configuration/launch files. The original archive also contains additional assets (including the Nav2 parameter file, map image, telemetry CSVs, plotting utility, RViz configuration and report screenshots). Those binary/large supporting assets are not yet present in this repository snapshot; therefore the committed files alone are not a complete, immediately launchable workspace.

## ROS 2 environment and next steps

Target environment: Ubuntu + ROS 2 Humble. The complete project also depends on Gazebo/TurtleBot3 simulation, Nav2 and `robot_localization`.

After restoring the remaining workspace assets from the supplied archive, the intended build flow is:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The intended launch sequence is:

```bash
ros2 launch bot_localization navigation_sim.launch.py
# In a second terminal:
ros2 launch bot_navigation fault_tolerant_nav.launch.py
```

## Future work from the report

1. Add a three-sample persistence/hysteresis filter to suppress brief false-positive wheel-slip detections during tight turns.
2. Dynamically scale local costmap inflation radius using the EKF covariance trace.

## Reference

*Fault-Tolerant Sensor Fusion, Motion Planning, and Autonomous Navigation in ROS 2*, supplied technical report for `turtlebot_nav_ws`, dated 24 November 2026.
