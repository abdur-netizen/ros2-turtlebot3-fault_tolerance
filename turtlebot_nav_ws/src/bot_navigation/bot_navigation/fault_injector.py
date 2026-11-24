import random
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class FaultInjector(Node):
    def __init__(self):
        super().__init__('fault_injector')
        self.declare_parameter('fault_mode', 'none')
        self.declare_parameter('slip_scale', 0.35)
        self.declare_parameter('noise_std', 0.25)
        self.fault_mode = self.get_parameter('fault_mode').value
        self.slip_scale = self.get_parameter('slip_scale').value
        self.noise_std = self.get_parameter('noise_std').value
        self.add_on_set_parameters_callback(self.on_param_change)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_faulty', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_faulty', 10)
        self.get_logger().info(f'FaultInjector ready (mode={self.fault_mode})')

    def on_param_change(self, params):
        for p in params:
            if p.name == 'fault_mode':
                self.fault_mode = p.value
                self.get_logger().warn(f'Switched fault_mode -> {self.fault_mode}')
            elif p.name == 'slip_scale':
                self.slip_scale = float(p.value)
            elif p.name == 'noise_std':
                self.noise_std = float(p.value)
        return SetParametersResult(successful=True)

    def scan_cb(self, msg: LaserScan):
        if self.fault_mode == 'lidar_blackout':
            return
        if self.fault_mode == 'lidar_noise':
            noisy = LaserScan()
            noisy.header = msg.header
            noisy.angle_min = msg.angle_min
            noisy.angle_max = msg.angle_max
            noisy.angle_increment = msg.angle_increment
            noisy.time_increment = msg.time_increment
            noisy.scan_time = msg.scan_time
            noisy.range_min = msg.range_min
            noisy.range_max = msg.range_max
            noisy.ranges = [
                max(msg.range_min, min(msg.range_max, r + random.gauss(0.0, self.noise_std)))
                if msg.range_min < r < msg.range_max else r
                for r in msg.ranges
            ]
            self.scan_pub.publish(noisy)
            return
        self.scan_pub.publish(msg)

    def odom_cb(self, msg: Odometry):
        if self.fault_mode == 'wheel_slip':
            corrupted = Odometry()
            corrupted.header = msg.header
            corrupted.child_frame_id = msg.child_frame_id
            corrupted.pose = msg.pose
            corrupted.twist = msg.twist
            corrupted.twist.twist.linear.x *= self.slip_scale
            corrupted.twist.twist.angular.z *= 1.8
            cov = list(msg.twist.covariance)
            cov[0] = 0.5
            cov[35] = 0.8
            corrupted.twist.covariance = cov
            self.odom_pub.publish(corrupted)
            return
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
