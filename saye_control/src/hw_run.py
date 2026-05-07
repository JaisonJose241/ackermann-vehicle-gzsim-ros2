#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
import math

class AMCAF_LineFollower(Node):
    def __init__(self):
        super().__init__('amcaf_line_follower')

        # 1. Physical Parameters (Saye-bot)
        self.L = 0.2255  # Wheelbase in meters
        self.target_y = 0.0
        self.V_nominal = 0.2 # Top speed (m/s)

        # 2. Control Gains
        # Longitudinal (Velocity) Gains - PD (unchanged)
        self.kp_v = 1.2
        self.kd_v = 0.1
        
        # Lateral (Steering) Gains - PD (updated: added derivative gains)
        self.kp_y   = 3.0   # Proportional: pulls robot to y=0
        self.kd_y   = 0.5   # Derivative:   damps lateral error rate
        self.kp_psi = 2.2   # Proportional: aligns robot heading to 0 rad
        self.kd_psi = 0.3   # Derivative:   damps heading error rate
        
        # 3. Physical Constraints (Saturation)
        self.max_a = 0.5      # m/s^2
        self.max_delta = 0.4  # rad (~23 degrees)

        # 4. Internal State Tracking
        self.last_time = self.get_clock().now()
        self.last_v = 0.0
        self.cmd_v = 0.0
        self.last_pos_x = None
        self.last_pos_y = None

        # Previous errors for derivative terms (steering PD)
        self.last_y_error   = 0.0
        self.last_psi_error = 0.0

        # Pubs and Subs
        self.pose_sub = self.create_subscription(PoseArray, '/saye/pose_array', self.control_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/saye/cmd_high_level', 10)

        self.get_logger().info("AMCAF Dynamic Line Follower Started.")

    def get_yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_callback(self, msg):
        if len(msg.poses) < 1: return

        # --- A. State Extraction ---
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0: return

        curr_x = msg.poses[0].position.x
        curr_y = msg.poses[0].position.y
        curr_psi = self.get_yaw_from_quat(msg.poses[0].orientation)

        # Estimate current velocity (v = ds/dt)
        if self.last_pos_x is None:
            v_actual = 0.0
        else:
            dist = math.sqrt((curr_x - self.last_pos_x)**2 + (curr_y - self.last_pos_y)**2)
            v_actual = dist / dt
        
        # --- B. Velocity Scheduling (V_s Logic) ---
        v_c = self.V_nominal * math.cos(self.kp_y * (self.target_y - curr_y))
        v_c = max(v_c, 0.01)
        v_b = self.V_nominal
        v_s = min(v_c, v_b, self.V_nominal)

        # --- C. Control Law Implementation ---
        # 1. Longitudinal Control: PD (unchanged)
        v_error = v_s - v_actual
        v_dot = (v_actual - self.last_v) / dt
        acceleration = (self.kp_v * v_error) + (self.kd_v * -v_dot)
        acceleration = max(min(acceleration, self.max_a), -self.max_a)

        # 2. Lateral Control: PD (updated)
        y_error   = self.target_y - curr_y
        psi_error = 0.0 - curr_psi

        # Derivative of errors
        y_error_dot   = (y_error   - self.last_y_error)   / dt
        psi_error_dot = (psi_error - self.last_psi_error) / dt

        # PD steering law:
        # delta = Kp_y*e_y + Kd_y*de_y/dt + Kp_psi*e_psi + Kd_psi*de_psi/dt
        steering_delta = (self.kp_y   * y_error   + self.kd_y   * -y_error_dot) + \
                         (self.kp_psi * psi_error + self.kd_psi * -psi_error_dot)

        print(y_error, psi_error, y_error_dot, psi_error_dot)

        steering_delta = max(min(steering_delta, self.max_delta), -self.max_delta)

        # --- D. Integration and Publication ---
        self.cmd_v += acceleration * dt
        self.cmd_v = max(0.0, self.cmd_v)

        cmd = Twist()
        cmd.linear.x  = float(self.cmd_v)
        cmd.angular.z = float(steering_delta)
        self.cmd_pub.publish(cmd)

        # Update historical values
        self.last_time      = current_time
        self.last_v         = v_actual
        self.last_pos_x     = curr_x
        self.last_pos_y     = curr_y
        self.last_y_error   = y_error      # <-- new
        self.last_psi_error = psi_error    # <-- new

def main():
    rclpy.init()
    node = AMCAF_LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()