#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Twist
import math


class LineFollowerP(Node):
    def __init__(self):
        super().__init__('line_follower_p')

        # 1. P-Controller Parameters
        # Adjust Kp if the robot oscillates (too high) or turns too slowly (too low)
        self.kp = 2
        self.target_y = 0.0
        self.constant_v = 0.3  # Constant linear velocity in m/s

        # 2. Subscriber: Listen to Odometry
        self.odom_sub = self.create_subscription(
            PoseArray,
            '/saye/pose_array', # Topic from your gz topic list
            self.odom_callback,
            10)

        # 3. Publisher: Send commands to your Base Controller
        self.cmd_pub = self.create_publisher(Twist, '/saye/cmd_high_level', 10)

        self.get_logger().info("Line Follower P-Controller started. Target Y: 0.0")
        self.sim_time_sec_start = self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg):
        sim_time_sec = self.get_clock().now().nanoseconds / 1e9
        # Gazebo's Pose_V usually puts the model's main pose at index 0
        if len(msg.poses) == 0:
            return
        

        # Extract Ground Truth Y from the first pose in the array
        current_y = msg.poses[2].position.y
        # print(msg.poses)
        
        # Calculate Error
        error_y = self.target_y - current_y
        
        # P-Controller: steering = Kp * error
        steering_angle = self.kp * error_y

        # Clamp steering angle to prevent unrealistic turns (e.g., max 0.4 rad)
        max_steer = 0.4
        steering_angle = max(min(steering_angle, max_steer), -max_steer)

        # Create the command message
        cmd = Twist()
        cmd.linear.x = self.constant_v
        cmd.angular.z = float(steering_angle)

        # Publish the command to the Base Controller
        self.cmd_pub.publish(cmd)
        
        # Debugging log
        self.get_logger().info(
            f"Time: {sim_time_sec-self.sim_time_sec_start:>8.2f}s | Actual Y: {current_y:>6.3f} | Steer Cmd: {steering_angle:>6.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()