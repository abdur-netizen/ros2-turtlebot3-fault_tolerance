# Technical report summary

**Source:** *Fault-Tolerant Sensor Fusion, Motion Planning, and Autonomous Navigation in ROS 2*  
**Workspace:** `turtlebot_nav_ws` · **Middleware:** ROS 2 Humble and Nav2 · **Report date:** 24 November 2026

This document summarizes the supplied seven-page technical report. Numerical results below are report-reported experimental outcomes, not independently reproduced measurements in this repository.

## 1. System architecture

The report groups the system into three packages:

- `bot_description`: robot kinematic model/description.
- `bot_localization`: EKF state estimation, AMCL/map server configuration, Nav2 parameters and simulation launch.
- `bot_navigation`: runtime fault injection, supervisory fault management and dynamic waypoint execution.

Main nodes and interfaces:

| Component | Interfaces | Function |
|---|---|---|
| Gazebo simulator | Publishes `/scan`, `/odom`, `/imu`, `/clock`; subscribes `/cmd_vel` | Differential-drive physics and sensor simulation |
| `fault_injector` | `/scan`, `/odom` → `/scan_faulty`, `/odom_faulty` | LiDAR dropout/noise and wheel-slip corruption |
| EKF | `/odom_faulty`, `/imu` → `/odometry/filtered` | Multi-sensor state estimation |
| AMCL/map server | `/scan`, `/initialpose`; map-to-odom TF | Global localization |
| Nav2 | `/scan_faulty`, `/speed_limit` → `/cmd_vel` | Global planning and local trajectory control |
| `fault_supervisor` | Monitors sensor, odometry and command topics; publishes `/speed_limit` | Fault isolation, degraded speed envelope and CSV KPI logging |
| `waypoint_navigator` | `/clicked_point`, `/dynamic_waypoints`; Nav2 waypoint action | Dynamic multi-waypoint mission execution |

### Coordinate transforms

The documented REP-105 transform chain is:

`map → odom → base_footprint → base_scan`

The report specifies `publish_tf: false` in `bot_localization/config/ekf.yaml`. This leaves the Gazebo differential-drive plugin as the sole publisher of `odom → base_footprint`, while AMCL publishes the drift-correcting `map → odom` transform.

## 2. Sensor fusion and motion model

The EKF state is described as:

`x = [x, y, θ, vx, ωz]ᵀ`

The report says the filter operates at 50 Hz and fuses wheel linear velocity from `/odom_faulty` with IMU yaw orientation/angular-rate information. Measurements are evaluated using the squared Mahalanobis distance:

`d² = yᵀ S⁻¹ y`

The report lists rejection thresholds of 2.0 for odometry twist and 2.5 for IMU angular velocity.

For differential-drive wheel radius `r`, track width `L`, commanded linear velocity `v` and angular velocity `ω`:

`ωR = (v + (L/2)ω) / r`  
`ωL = (v − (L/2)ω) / r`

The reported multi-objective trajectory cost balances goal distance, obstacle clearance, reference-velocity tracking and command smoothness. The energy-effort proxy is:

`E = ∫(v² + ω²)dt`

This proxy is not a direct measurement of battery current, voltage or electrical energy.

## 3. Fault detection and adaptive supervision

The report documents:

- LiDAR watchdog: declares blackout when `/scan_faulty` message age exceeds **0.5 s**.
- Wheel-slip residual: `|ωodom − ωimu| > 0.35 rad/s` or `|vodom − vekf| > 0.12 m/s`.
- Dynamic speed limits are published on `/speed_limit` at 10 Hz.

| Supervisor mode | Trigger | Speed cap | Response |
|---|---|---:|---|
| `NOMINAL_BALANCED` | Healthy LiDAR and no slip residuals; balanced profile | 100% | Normal trajectory tracking |
| `NOMINAL_ECO` | Healthy sensors; eco profile | 65% | Lower speed to reduce command effort |
| `DEGRADED_WHEEL_SLIP` | Wheel/IMU or wheel/EKF residual threshold exceeded | 45% | EKF outlier rejection and reduced drive/turn speed |
| `DEGRADED_LIDAR_LOSS` | Scan age greater than 0.5 s | 30% | Reduced-speed dead-reckoning until sensor recovery |

## 4. Integration issues resolved in the report

1. **Competing TF authorities:** disabling EKF TF publication resolved duplicate `odom → base_footprint` transforms and restored costmap alignment.
2. **Duplicate launch/configuration instances:** consolidated launch ownership to avoid duplicate EKF nodes and map-coordinate mismatches.
3. **Nav2 lifecycle cascade shutdown:** the report records increasing bond timeout to 20 s, setting behavior-tree loop duration to 50 ms and transform tolerance to 2 s.
4. **RViz2 hybrid-graphics rendering:** X11 (`QT_QPA_PLATFORM=xcb`) and staggered launch timing were used to avoid OpenGL viewport deadlocks.

## 5. Experimental scenarios

| Run | Scenario | Injected condition | Reported outcome |
|---|---|---|---|
| 1 | Nominal baseline | No fault; balanced profile | 100% speed; odometry–EKF drift below 0.03 m |
| 2 | LiDAR blackout | Complete scan dropout for about 5 s | Watchdog responds within 0.5 s; speed reduced to 30%, restored to 100% after recovery |
| 3 | LiDAR noise | Gaussian range noise, σ = 0.25 m | Local costmap filters transient range speckles; report states waypoint tracking remained collision-free |
| 4 | Wheel slip | Linear velocity scaled by 0.35×; yaw rate by 1.8× | Raw odometry reached 0.51 m peak drift and 0.38 m steady-state; speed capped at 45% |
| 5 | Pop-up obstacle | Unmapped static box spawned on active path | Local costmap updated and Nav2 re-routed without collision |
| 6 | Eco optimization | No sensor fault; eco profile | 65% speed cap and visibly slower cumulative control-effort growth |

### Telemetry figure interpretation

The report's page 6 figure compares speed limit, raw odometry versus EKF drift, and cumulative control effort over approximately 182 seconds. It marks the LiDAR blackout around 48–52 s, wheel-slip interval around 112–117 s and eco mode from about 155 s. The reported wheel-slip peak is 0.51 m; the eco profile produces a gentler cumulative control-effort slope.



## 7. Conclusion and proposed future work

The report concludes that EKF sensor fusion, Nav2 planning and supervisory fault management form a resilient simulated navigation pipeline for LiDAR blackouts, wheel slip and dynamic obstacles. Proposed future work is:

1. Add a three-sample persistence/hysteresis filter to suppress brief false-positive slip detections during in-place turns.
2. Scale local costmap inflation radius dynamically using the EKF covariance trace.

---

**Source fidelity note:** This summary follows the supplied report's stated architecture, thresholds, scenarios and measurements. It does not claim independent re-testing. Where a value is described as a report result, it should be interpreted as a reported simulation outcome.
