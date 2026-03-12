#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

class SayeBaseController(Node):
    def __init__(self):
        super().__init__('saye_base_controller')
        
        # Physical Constants from saye model.sdf
        self.L = 0.2255  # Wheelbase
        self.W = 0.2     # Wheel separation (Track width)
        self.r = 0.0365  # Wheel radius
        
        # Internal State
        self.current_v = 0.0
        self.current_delta = 0.0
        
        # Subscriber: Listens for high-level commands from MPC/RL/Teleop
        self.cmd_sub = self.create_subscription(
            Twist,
            '/saye/cmd_high_level',
            self.cmd_callback,
            10)
        
        # Publishers: Send to Gazebo bridged topics
        self.pub_steer_l = self.create_publisher(Float64, '/saye/steering_left', 10)
        self.pub_steer_r = self.create_publisher(Float64, '/saye/steering_right', 10)
        self.pub_drive_l = self.create_publisher(Float64, '/saye/drive_left', 10)
        self.pub_drive_r = self.create_publisher(Float64, '/saye/drive_right', 10)
        
        # Control Loop (50Hz) - Ensures Gazebo joints are constantly "held"
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Saye Base Controller Started. Listening on /saye/cmd_high_level")

    def cmd_callback(self, msg):
        """Update internal targets when a new message arrives"""
        self.current_v = msg.linear.x
        self.current_delta = msg.angular.z

    def control_loop(self):
        v = self.current_v
        delta = self.current_delta

        # Simple Steering (Same angle for both)
        steer_l = delta
        steer_r = delta

        # Convert Linear Velocity (m/s) to Angular Velocity (rad/s)
        # omega = v / r
        wheel_speed_rad_s = v / self.r

        # Publish commands to joints
        self.publish_float(self.pub_steer_l, steer_l)
        self.publish_float(self.pub_steer_r, steer_r)
        self.publish_float(self.pub_drive_l, wheel_speed_rad_s)
        self.publish_float(self.pub_drive_r, wheel_speed_rad_s)

    def publish_float(self, publisher, value):
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SayeBaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()