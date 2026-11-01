import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class PurePursuitMission(Node):

    def __init__(self):
        super().__init__('pure_pursuit_mission')

        self.path_points = [
            (7.85, -0.138), (9.66, 0.50), (10.0, 7.14), (9.00, 8.34),
            (-12.84, 8.85), (-13.68, 8.4), (-13.36, 0.874), (-12.807, -0.15),
        ]

        self.linear_speed = 0.6
        self.look_ahead_distance = 1.0
        self.wheelbase = 0.3302
        self.max_steering_angle = 0.4

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.last_odom_time = None
        self.collision_detected = False

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.path_publisher = self.create_publisher(Path, '/pure_pursuit_path', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.collision_subscriber = self.create_subscription(
            Bool, '/ego_racecar/collision', self.collision_callback, 10)

        n = len(self.path_points)
        self.segment_lengths = [
            math.hypot(self.path_points[(i + 1) % n][0] - self.path_points[i][0],
                       self.path_points[(i + 1) % n][1] - self.path_points[i][1])
            for i in range(n)
        ]
        self.cumulative_lengths = [0.0]
        for length in self.segment_lengths:
            self.cumulative_lengths.append(self.cumulative_lengths[-1] + length)
        self.total_path_length = self.cumulative_lengths[-1]

        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.path_timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info('F1TENTH Pure Pursuit started')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.last_odom_time = time.monotonic()

    def collision_callback(self, msg):
        if msg.data and not self.collision_detected:
            self.collision_detected = True
            self.get_logger().error('Collision detected! Stopping car.')

    def closest_point_on_path(self):
        minimum_distance = float('inf')
        closest_progress = 0.0

        for i in range(len(self.path_points)):
            start_x, start_y = self.path_points[i]
            end_x, end_y = self.path_points[(i + 1) % len(self.path_points)]
            dx = end_x - start_x
            dy = end_y - start_y

            fraction = ((self.robot_x - start_x) * dx
                        + (self.robot_y - start_y) * dy) / (dx * dx + dy * dy)
            fraction = max(0.0, min(1.0, fraction))

            distance = math.hypot(
                self.robot_x - (start_x + fraction * dx),
                self.robot_y - (start_y + fraction * dy)
            )

            if distance < minimum_distance:
                minimum_distance = distance
                closest_progress = (self.cumulative_lengths[i]
                                    + fraction * self.segment_lengths[i])

        return closest_progress

    def get_lookahead_point(self, progress):
        target_progress = (progress + self.look_ahead_distance) % self.total_path_length

        for i in range(len(self.segment_lengths)):
            if target_progress <= self.cumulative_lengths[i + 1]:
                start_x, start_y = self.path_points[i]
                end_x, end_y = self.path_points[(i + 1) % len(self.path_points)]
                fraction = (target_progress - self.cumulative_lengths[i]) / self.segment_lengths[i]
                return (start_x + fraction * (end_x - start_x),
                        start_y + fraction * (end_y - start_y))

        return self.path_points[0]

    def send_drive_command(self, speed, steering):
        command = AckermannDriveStamped()
        command.drive.speed = float(speed)
        command.drive.steering_angle = float(steering)
        self.drive_publisher.publish(command)

    def stop_robot(self):
        self.send_drive_command(0.0, 0.0)

    def control_loop(self):
        if self.robot_x is None:
            return

        if self.collision_detected or time.monotonic() - self.last_odom_time > 0.5:
            self.stop_robot()
            return

        target_x, target_y = self.get_lookahead_point(self.closest_point_on_path())

        dx = target_x - self.robot_x
        dy = target_y - self.robot_y

        target_y_car = -math.sin(self.robot_yaw) * dx + math.cos(self.robot_yaw) * dy
        curvature = 2.0 * target_y_car / max(dx * dx + dy * dy, 0.000001)
        steering_angle = math.atan(self.wheelbase * curvature)
        steering_angle = max(-self.max_steering_angle,
                             min(self.max_steering_angle, steering_angle))

        self.send_drive_command(self.linear_speed, steering_angle)

    def publish_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.path_points + [self.path_points[0]]:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_publisher.publish(path)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PurePursuitMission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.control_timer.cancel()
        for _ in range(10):
            node.stop_robot()
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()