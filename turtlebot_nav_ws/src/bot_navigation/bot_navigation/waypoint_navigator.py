#!/usr/bin/env python3
import math
import time

from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter


def create_pose(
    navigator: BasicNavigator, x: float, y: float, yaw_rad: float = 0.0, frame_id: str = 'map'
) -> PoseStamped:
    """Build a PoseStamped message from (x, y, yaw_rad)."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


def quaternion_to_yaw_deg(z: float, w: float) -> float:
    """Convert planar quaternion (z, w) into heading in degrees."""
    return math.degrees(2.0 * math.atan2(z, w))


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    navigator.get_logger().info('Waiting for Gazebo /clock tick...')
    while not (0 < navigator.get_clock().now().seconds_nanoseconds()[0] < 1_000_000_000):
        rclpy.spin_once(navigator, timeout_sec=0.1)

    initial_pose = create_pose(navigator, -2.0, -0.5, 0.0)
    navigator.get_logger().info('Setting initial pose estimate for AMCL...')
    navigator.setInitialPose(initial_pose)
    navigator.get_logger().info('Waiting for Nav2 and AMCL to become active...')
    navigator.waitUntilNav2Active()

    active_waypoints = []
    mission_active = False
    route_updated = False
    last_known_xy = (-2.0, -0.5)

    def pose_array_callback(msg: PoseArray) -> None:
        nonlocal active_waypoints, route_updated
        if not msg.poses:
            return
        frame = msg.header.frame_id if msg.header.frame_id else 'map'
        now_stamp = navigator.get_clock().now().to_msg()
        new_list = []
        for p in msg.poses:
            ps = PoseStamped()
            ps.header.frame_id = frame
            ps.header.stamp = now_stamp
            ps.pose = p
            new_list.append(ps)
        active_waypoints = new_list
        route_updated = True
        navigator.get_logger().info(
            f'Received dynamic batch of {len(active_waypoints)} waypoints on /dynamic_waypoints.'
        )

    def clicked_point_callback(msg: PointStamped) -> None:
        nonlocal active_waypoints, route_updated
        if not mission_active:
            active_waypoints = []
        ref_x, ref_y = (
            (active_waypoints[-1].pose.position.x, active_waypoints[-1].pose.position.y)
            if active_waypoints else last_known_xy
        )
        yaw_rad = math.atan2(msg.point.y - ref_y, msg.point.x - ref_x)
        frame = msg.header.frame_id if msg.header.frame_id else 'map'
        active_waypoints.append(create_pose(
            navigator, msg.point.x, msg.point.y, yaw_rad, frame_id=frame
        ))
        route_updated = True
        navigator.get_logger().info(
            f'Added dynamic waypoint #{len(active_waypoints)} -> '
            f'x: {msg.point.x:+.2f} m, y: {msg.point.y:+.2f} m'
        )

    navigator.create_subscription(PoseArray, '/dynamic_waypoints', pose_array_callback, 10)
    navigator.create_subscription(PointStamped, '/clicked_point', clicked_point_callback, 10)
    navigator.get_logger().info(
        'Ready for dynamic waypoints! Use RViz2 Publish Point or publish to /dynamic_waypoints.'
    )

    last_log_time = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(navigator, timeout_sec=0.1)
            if route_updated and active_waypoints:
                if mission_active and not navigator.isTaskComplete():
                    navigator.get_logger().info('Preempting active route with updated waypoints...')
                    navigator.cancelTask()
                    rclpy.spin_once(navigator, timeout_sec=0.1)
                navigator.get_logger().info(
                    f'Navigating through {len(active_waypoints)} dynamic waypoint(s)...'
                )
                navigator.goThroughPoses(list(active_waypoints))
                mission_active = True
                route_updated = False

            if mission_active:
                if not navigator.isTaskComplete():
                    feedback = navigator.getFeedback()
                    now = time.time()
                    if feedback and (now - last_log_time) >= 0.5:
                        pos = feedback.current_pose.pose.position
                        ori = feedback.current_pose.pose.orientation
                        last_known_xy = (pos.x, pos.y)
                        yaw_deg = quaternion_to_yaw_deg(ori.z, ori.w)
                        navigator.get_logger().info(
                            f'Localized Pose -> x: {pos.x:+.2f} m, y: {pos.y:+.2f} m, '
                            f'yaw: {yaw_deg:+.1f} deg | '
                            f'Remaining dist: {feedback.distance_remaining:.2f} m | '
                            f'Poses left: {feedback.number_of_poses_remaining}'
                        )
                        last_log_time = now
                else:
                    result = navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        navigator.get_logger().info(
                            'Dynamic route completed! Waiting for new waypoints...'
                        )
                    elif result == TaskResult.CANCELED:
                        navigator.get_logger().warn('Dynamic route canceled.')
                    elif result == TaskResult.FAILED:
                        navigator.get_logger().error(
                            'Waypoint failed (check if point is inside an obstacle). Waiting for new waypoints...'
                        )
                    active_waypoints.clear()
                    mission_active = False
    except KeyboardInterrupt:
        navigator.get_logger().info('Shutting down dynamic waypoint navigator...')
    finally:
        navigator.destroyNode()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
