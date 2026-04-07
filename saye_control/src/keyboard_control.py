#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rc_msgs.msg import RCMessage

from pynput import keyboard
import threading

class KeyboardRCNode(Node):
    def __init__(self):
        super().__init__('keyboard_rc_controller')

        self.publisher_ = self.create_publisher(RCMessage, '/drone/rc_command', 10)

        # Neutral values
        self.rc_pitch = 1500
        self.rc_yaw = 1500

        # Key states
        self.keys_pressed = set()

        # Start keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

        # Publish at 20 Hz
        self.timer = self.create_timer(0.05, self.update_and_publish)

    def on_press(self, key):
        self.keys_pressed.add(key)

    def on_release(self, key):
        if key in self.keys_pressed:
            self.keys_pressed.remove(key)

    def update_and_publish(self):
        # Default neutral
        pitch = 1500
        yaw = 1440

        # PITCH control (UP/DOWN)
        if keyboard.Key.up in self.keys_pressed:
            pitch = 1570
        elif keyboard.Key.down in self.keys_pressed:
            pitch = 1440

        # YAW control (LEFT/RIGHT)
        if keyboard.Key.left in self.keys_pressed:
            yaw = 2000
        elif keyboard.Key.right in self.keys_pressed:
            yaw = 1000

        # Assign independently
        self.rc_pitch = pitch
        self.rc_yaw = yaw

        # Publish message
        msg = RCMessage()
        msg.rc_throttle = 990
        msg.rc_roll = 1500
        msg.rc_pitch = self.rc_pitch
        msg.rc_yaw = self.rc_yaw

        msg.aux1 = 1000
        msg.aux2 = 1000
        msg.aux3 = 1000
        msg.aux4 = 1000

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = KeyboardRCNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.listener.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()