#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rc_msgs.msg import RCMessage
import math

class CmdToRCBridge(Node):
    def __init__(self):
        super().__init__('cmd_to_rc_bridge')

        # 1. Publisher: Sends RC commands to the drone/base
        self.publisher_ = self.create_publisher(RCMessage, '/drone/rc_command', 10)

        # 2. Subscriber: Listens to your high-level AMCAF controller
        self.subscription = self.create_subscription(
            Twist,
            '/saye/cmd_high_level',
            self.cmd_callback,
            10)

        # 3. Internal state (Neutral defaults)
        self.curr_v = 0.0
        self.curr_delta = 0.0
        
        # Scaling Factors (Adjust these based on your robot's max speed/turn)
        # Pitch: Maps v=0.5m/s to 1570 (as in your keyboard example)
        self.V_SCALE = 140.0  
        # Yaw: Maps delta=0.4rad to 2000 (as in your keyboard example)
        self.DELTA_SCALE = 1400.0 

        # 4. Timer: Publish at 20 Hz (Matches your keyboard script frequency)
        self.timer = self.create_timer(0.05, self.update_and_publish)
        
        self.get_logger().info("AMCAF Bridge Started. Mapping Twist -> RCMessage.")

    def cmd_callback(self, msg):
        """
        Receives Twist messages:
        linear.x  -> Velocity (v)
        angular.z -> Steering Angle (delta)
        """
        self.curr_v = msg.linear.x
        self.curr_delta = msg.angular.z

    def update_and_publish(self):
        # --- Mapping Logic ---

        # 1. Map Velocity (v) to RC Pitch
        # 1500 is neutral. Positive v increases pitch (Forward).
        rc_pitch = int(1500 + (self.curr_v * self.V_SCALE))
        if rc_pitch>=1570:
            rc_pitch=1570
        if rc_pitch<=1440:
            rc_pitch=1440

        # 2. Map Steering (delta) to RC Yaw
        # 1440 is neutral as per your example. Positive delta (Left) increases yaw.
        rc_yaw = int(1440 - (self.curr_delta * self.DELTA_SCALE))

        # --- Safety Constraints (Clamping) ---
        rc_pitch = max(min(rc_pitch, 2000), 1000)
        rc_yaw = max(min(rc_yaw, 2000), 1000)

        # --- Construct RCMessage ---
        msg = RCMessage()
        msg.rc_throttle = 990   # Minimum/Idle throttle as per example
        msg.rc_roll = 1500      # Neutral roll
        msg.rc_pitch = rc_pitch
        msg.rc_yaw = rc_yaw

        # Constants for auxiliary channels
        msg.aux1 = 1000
        msg.aux2 = 1000
        msg.aux3 = 1000
        msg.aux4 = 1000

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdToRCBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()