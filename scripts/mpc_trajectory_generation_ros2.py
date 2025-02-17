#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from math import sin, cos, pi, atan2, fabs
from tf_transformations import quaternion_from_euler  # Install via pip if needed


class PathNode(Node):
    def __init__(self):
        super().__init__('path_node')
        self.get_logger().info("path_node is started!!")

        # Parameters and configuration
        self.declare_parameter('trajectory_type', '')
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.frame_id = "odom"  # same as id in original

        # Initialize global variables as instance variables
        self.odom_path = Path()
        self.error_path = Path()
        self.desired_path = Path()
        self.robot_odom = Odometry()
        self.odom_count = 0
        self.sum_error = 0.0
        self.error_x = 0.0
        self.current_path_x = 0.0
        self.current_path_y = 0.0
        self.current_path_theta = 0.0

        # Set up publishers
        qos = 10  # using a queue size of 10 for simplicity
        self.desired_path_pub = self.create_publisher(Path, '/desired_path', qos)
        self.odom_path_pub = self.create_publisher(Path, '/recorded_path', qos)
        self.error_path_pub = self.create_publisher(Path, '/error_path', qos)

        # Set up subscriber to /odom
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos)
        self.create_timer(0.1, self.publish_desired_path)

    def publish_desired_path(self):
        self.generation_desired_path()
        self.desired_path_pub.publish(self.desired_path)

    def odom_cb(self, msg: Odometry):
        self.robot_odom = msg
        self.odom_count += 1

        if self.odom_count % 3 == 0:
            # Update odom path
            self.odom_path.header = msg.header
            self.odom_path.header.frame_id = self.frame_id

            pose = PoseStamped()
            pose.header = msg.header
            pose.header.frame_id = self.frame_id
            pose.pose = msg.pose.pose
            self.odom_path.poses.append(pose)

            self.odom_path_pub.publish(self.odom_path)
            self.generation_desired_path()

    def generation_error_path(self):
        self.error_path.header.frame_id = self.frame_id
        # Using the node's clock for timestamps:
        now = self.get_clock().now().to_msg()
        self.error_path.header.stamp = now
        # self.error_path.header.seq = 1

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = now
        pose.pose.position.x = self.current_path_x
        pose.pose.position.y = self.current_path_y
        pose.pose.orientation.w = 1.0
        self.error_path.poses.append(pose)

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = now
        # self.error_path.header.seq = 2
        pose.pose.position.x = self.robot_odom.pose.pose.position.x
        pose.pose.position.y = self.current_path_y
        pose.pose.orientation.w = 1.0
        self.error_path.poses.append(pose)

        self.error_path_pub.publish(self.error_path)

    def generation_desired_path(self):
        self.get_logger().info("generation_desired_path()")
        desired_path = Path()
        desired_path.header.frame_id = self.frame_id

        iter_count = 1000

        if self.trajectory_type == "circle":
            radius = 5.0
            period = 1000.0
            for t in range(iter_count):
                now = self.get_clock().now().to_msg()
                desired_path.header.stamp = now
                # desired_path.header.seq = t

                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.header.stamp = now
                # pose.header.seq = t
                pose.pose.position.x = radius * sin(2 * pi * t / period)
                pose.pose.position.y = -radius * cos(2 * pi * t / period)
                grad = atan2((-radius * cos(2 * pi * (t + 1) / period) - pose.pose.position.y),
                             (radius * sin(2 * pi * (t + 1) / period) - (pose.pose.position.x + 1e-5)))
                q = quaternion_from_euler(0, 0, grad)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                desired_path.poses.append(pose)

        elif self.trajectory_type == "epitrochoid":
            R = 5.0
            r = 1.0
            d = 3.0
            period = 1000.0
            scale_factor = 1.0
            for t in range(iter_count):
                now = self.get_clock().now().to_msg()
                desired_path.header.stamp = now
                # desired_path.header.seq = t

                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.header.stamp = now
                # pose.header.seq = t
                pose.pose.position.x = scale_factor * (
                            (R + r) * cos(2 * pi * t / period) - d * cos(((R + r) / r) * 2 * pi * t / period))
                pose.pose.position.y = scale_factor * (
                            (R + r) * sin(2 * pi * t / period) - d * sin(((R + r) / r) * 2 * pi * t / period))
                grad = atan2((5 * sin(2 * pi * (t + 1) / period) * cos(2 * pi * (t + 1) / period) / (
                            (sin(2 * pi * (t + 1) / period)) ** 2 + 1) - pose.pose.position.y),
                             (5 * cos(2 * pi * (t + 1) / period) / (
                                         (sin(2 * pi * (t + 1) / period)) ** 2 + 1) - pose.pose.position.x + 1e-5))
                q = quaternion_from_euler(0, 0, grad)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                desired_path.poses.append(pose)

        elif self.trajectory_type == "infinite":
            period = 1000.0
            scale_factor = 1.0
            for t in range(iter_count):
                now = self.get_clock().now().to_msg()
                desired_path.header.stamp = now
                # desired_path.header.seq = t

                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.header.stamp = now
                # pose.header.seq = t
                pose.pose.position.x = 10 * cos(2 * pi * t / period) / ((sin(2 * pi * t / period)) ** 2 + 1)
                pose.pose.position.y = 10 * sin(2 * pi * t / period) * cos(2 * pi * t / period) / (
                            (sin(2 * pi * t / period)) ** 2 + 1)
                grad = atan2((10 * cos(2 * pi * (t + 1) / period) / (
                            (sin(2 * pi * (t + 1) / period)) ** 2 + 1) - pose.pose.position.y),
                             (10 * sin(2 * pi * (t + 1) / period) * cos(2 * pi * (t + 1) / period) / (
                                         (sin(2 * pi * (t + 1) / period)) ** 2 + 1) - pose.pose.position.x + 1e-5))
                q = quaternion_from_euler(0, 0, grad)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                desired_path.poses.append(pose)

        elif self.trajectory_type == "square":
            period = 1000.0
            l = 10.0
            PI = 3.141592
            x = 0.0
            y = 0.0
            for t in range(iter_count):
                now = self.get_clock().now().to_msg()
                desired_path.header.stamp = now
                # desired_path.header.seq = t

                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.header.stamp = now
                # pose.header.seq = t

                if t <= period * 0.25:
                    x = 0.0
                    y += l / (period * 0.25)
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    q = quaternion_from_euler(0, 0, PI / 2)
                elif t <= period * 0.5:
                    x -= l / (period * 0.25)
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    q = quaternion_from_euler(0, 0, PI)
                elif t <= period * 0.75:
                    y -= l / (period * 0.25)
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    q = quaternion_from_euler(0, 0, -PI / 2)
                elif t <= period:
                    x += l / (period * 0.25)
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    q = quaternion_from_euler(0, 0, 0)
                else:
                    pose.pose.position.x = 0.0
                    pose.pose.position.y = 0.0
                    q = quaternion_from_euler(0, 0, 0)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                desired_path.poses.append(pose)
        else:
            self.get_logger().warn("Unknown trajectory_type parameter")

        self.desired_path_pub.publish(desired_path)

    def calculate_error(self, path_x, path_y, path_theta, robot_x, robot_y, robot_theta):
        self.error_x = fabs(path_x - robot_x)
        self.sum_error += self.error_x
        self.get_logger().info(
            f"path_x: {path_x}, path_y: {path_y}, path_theta: {path_theta}, robot_x: {robot_x}, robot_y: {robot_y}, robot_theta: {robot_theta}")

    def find_line_position(self, path_number, y):
        # NOTE: original function referenced undefined variables such as path_arr_yy and generateVel.
        # Here we assume desired_path has already been published or stored as self.desired_path.
        self.current_path_y = y
        if len(self.desired_path.poses) == path_number + 1:
            self.get_logger().info("### Passing Last path ###")
            # generateVel(0.0, 0.0)  # function not defined; add as needed
            self.get_logger().info(f"Total error sum: {self.sum_error}")
            return

        start_pt = self.desired_path.poses[path_number].pose.position
        next_pt = self.desired_path.poses[path_number + 1].pose.position

        if start_pt.x == next_pt.x:
            self.current_path_x = start_pt.x
            self.current_path_theta = 1.570796
        elif start_pt.y == next_pt.y:
            self.current_path_y = start_pt.y
        else:
            self.current_path_x = (y - start_pt.y) * (next_pt.x - start_pt.x) / (next_pt.y - start_pt.y) + start_pt.x
            self.current_path_theta = atan2(next_pt.y - start_pt.y, next_pt.x - start_pt.x)

def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
