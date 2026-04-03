#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
import math

class PointStabilizerUnicycle(Node):
    def __init__(self):
        super().__init__('point_stabilizer_unicycle')

        # 1. Controller Parameters
        self.kx = 0.5        # Gain for linear velocity (v = kx * dist_error)
        self.k_theta = 2.0   # Gain for angular velocity (w = k_theta * heading_error)
        
        # Goal Position
        self.goal_x = -0.2
        self.goal_y = 0.0
        self.dist_tolerance = 0.01 # Stop within 10cm

        # 2. Subscriber: Ground truth pose from Gazebo
        self.pose_sub = self.create_subscription(
            PoseArray,
            '/saye/pose_array',
            self.pose_callback,
            10)

        # 3. Publisher: Sending (v, w) to your Base Controller
        self.cmd_pub = self.create_publisher(Twist, '/saye/cmd_high_level', 10)

        self.get_logger().info(f"Goal Seeker Started. Targeting ({self.goal_x}, {self.goal_y})")
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def get_yaw_from_quaternion(self, q):
        """Converts quaternion to yaw (theta)"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_callback(self, msg):
        if len(msg.poses) == 0:
            return

        # Time for logging
        sim_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time

        # Extract current state
        curr_x = msg.poses[2].position.x
        curr_y = msg.poses[2].position.y
        curr_theta = self.get_yaw_from_quaternion(msg.poses[2].orientation)

        # 4. Error Calculation
        dx = self.goal_x - curr_x
        dy = self.goal_y - curr_y
        dist_error = math.sqrt(dx**2 + dy**2)

        # Angle from robot to goal
        desired_theta = math.atan2(dy, dx)
        
        # Heading error wrapped to [-pi, pi]
        heading_error = desired_theta - curr_theta
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # 5. Unicycle Control Law
        if dist_error < self.dist_tolerance:
            v = 0.0
            omega = 0.0
            self.get_logger().info("Goal Reached!")
        else:
            # Linear velocity proportional to distance
            v = self.kx * dist_error
            # Angular velocity proportional to heading error
            omega = self.k_theta * heading_error

        # 6. Physical Constraints (Clamping)
        v = min(v, 0.7)        # Max speed 0.7 m/s
        omega = max(min(omega, 1.0), -1.0) # Max rotation 1.0 rad/s

        # 7. Publish Twist (v and w)
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

        # Logging for your MTP progress
        self.get_logger().info(
            f"T: {sim_time:.2f}s | Dist: {dist_error:.2f}m | V: {v:.2f} | Omega: {omega:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PointStabilizerUnicycle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()