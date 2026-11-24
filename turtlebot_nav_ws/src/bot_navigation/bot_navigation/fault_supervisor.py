import csv
import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from nav2_msgs.msg import SpeedLimit


class FaultSupervisor(Node):
    def __init__(self):
        super().__init__('fault_supervisor')
        self.declare_parameter('opt_profile', 'balanced')
        self.declare_parameter('scan_timeout_sec', 0.5)
        self.declare_parameter('slip_yaw_thresh', 0.35)
        self.declare_parameter('slip_lin_thresh', 0.12)
        self.declare_parameter('log_path', 'mission_metrics.csv')
        self.declare_parameter('reset_log', False)
        self.add_on_set_parameters_callback(self.on_param_change)
        self.last_scan_time = None
        self.odom_vx = self.odom_wz = self.ekf_vx = self.imu_wz = 0.0
        self.raw_xy = self.ekf_xy = self.initial_offset = self.prev_ekf_xy = None
        self.ekf_cov_trace = 0.0
        self.start_time = None
        self.path_length = self.control_effort = 0.0
        self.current_cmd = (0.0, 0.0)
        self.active_mode = 'INIT'
        self.create_subscription(LaserScan, '/scan_faulty', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom_faulty', self.raw_odom_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.log_path = self.get_parameter('log_path').value
        self.csv_file = self.csv_writer = None
        self._open_new_csv()
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'FaultSupervisor ready; logging KPIs to {self.log_path}')

    def _open_new_csv(self):
        if self.csv_file is not None:
            self.csv_file.close()
        self.csv_file = open(self.log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'elapsed_s', 'mode', 'speed_pct', 'path_length_m',
            'odom_ekf_drift_m', 'ekf_cov_trace', 'cum_control_effort'
        ])
        self.csv_file.flush()

    def reset_metrics(self):
        self.start_time = None
        self.path_length = self.control_effort = 0.0
        self.prev_ekf_xy = self.initial_offset = None
        self.active_mode = 'INIT'
        self._open_new_csv()
        self.get_logger().info('Metrics and mission_metrics.csv reset for new scenario run.')

    def on_param_change(self, params):
        for p in params:
            if p.name == 'reset_log' and p.value is True:
                self.reset_metrics()
        return SetParametersResult(successful=True)

    def scan_cb(self, _msg: LaserScan):
        now = self.get_clock().now()
        if now.nanoseconds > 0:
            self.last_scan_time = now

    def raw_odom_cb(self, msg: Odometry):
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_wz = msg.twist.twist.angular.z
        self.raw_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def imu_cb(self, msg: Imu):
        self.imu_wz = msg.angular_velocity.z

    def ekf_cb(self, msg: Odometry):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.ekf_vx = msg.twist.twist.linear.x
        if self.prev_ekf_xy is not None:
            self.path_length += math.hypot(x - self.prev_ekf_xy[0], y - self.prev_ekf_xy[1])
        self.prev_ekf_xy = (x, y)
        self.ekf_xy = (x, y)
        self.ekf_cov_trace = msg.pose.covariance[0] + msg.pose.covariance[7]

    def cmd_cb(self, msg: Twist):
        self.current_cmd = (msg.linear.x, msg.angular.z)

    def publish_speed_limit(self, pct: float):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = True
        msg.speed_limit = float(pct)
        self.speed_pub.publish(msg)

    def control_loop(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0 or self.last_scan_time is None:
            return
        if self.start_time is None:
            self.start_time = now
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        if elapsed < 0.0:
            return
        dt = 0.1
        v, w = self.current_cmd
        self.control_effort += (v * v + w * w) * dt
        scan_age = (now - self.last_scan_time).nanoseconds * 1e-9
        lidar_fault = scan_age > self.get_parameter('scan_timeout_sec').value
        yaw_residual = abs(self.odom_wz - self.imu_wz)
        lin_residual = abs(self.odom_vx - self.ekf_vx)
        slip_fault = (
            yaw_residual > self.get_parameter('slip_yaw_thresh').value
            or lin_residual > self.get_parameter('slip_lin_thresh').value
        )
        profile = self.get_parameter('opt_profile').value
        if lidar_fault:
            mode, speed_pct = 'DEGRADED_LIDAR_LOSS', 30.0
        elif slip_fault:
            mode, speed_pct = 'DEGRADED_WHEEL_SLIP', 45.0
        elif profile == 'eco':
            mode, speed_pct = 'NOMINAL_ECO', 65.0
        else:
            mode, speed_pct = 'NOMINAL_BALANCED', 100.0
        if mode != self.active_mode:
            self.get_logger().warn(
                f'State transition: {self.active_mode} -> {mode} (speed={speed_pct}%)'
            )
            self.active_mode = mode
        self.publish_speed_limit(speed_pct)
        drift = 0.0
        if self.raw_xy is not None and self.ekf_xy is not None:
            dx0 = self.raw_xy[0] - self.ekf_xy[0]
            dy0 = self.raw_xy[1] - self.ekf_xy[1]
            if self.initial_offset is None:
                self.initial_offset = (dx0, dy0)
            drift = math.hypot(dx0 - self.initial_offset[0], dy0 - self.initial_offset[1])
        self.csv_writer.writerow([
            f'{elapsed:.2f}', mode, speed_pct, f'{self.path_length:.3f}',
            f'{drift:.3f}', f'{self.ekf_cov_trace:.4f}', f'{self.control_effort:.3f}'
        ])
        self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaultSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
