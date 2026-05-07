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
        self.V_nominal = 0.2  # Top speed (m/s)

        # 2. Control Gains
        # Longitudinal (Velocity) Gains - PD (unchanged)
        self.kp_v = 1.2
        self.kd_v = 0.1

        # Lateral (Steering) Gains - PD
        self.kp_y   = 3.0   # Proportional: pulls robot to target_y
        self.kd_y   = 0.5   # Derivative:   damps lateral error rate
        self.kp_psi = 2.2   # Proportional: aligns robot heading to 0 rad
        self.kd_psi = 0.3   # Derivative:   damps heading error rate

        # 3. Physical Constraints (Saturation)
        self.max_a     = 0.5   # m/s^2
        self.max_delta = 0.4   # rad (~23 degrees)

        # 4. Obstacle Definition (box obstacle centered on y=0)
        # Mimics a static neighbor vehicle as per paper's Section V-B
        self.obstacle = {
            'x_min': 1.0,   # longitudinal start (m)
            'x_max': 1.2,   # longitudinal end   (m)
            'y_min': -0.3, # lateral left edge  (m)
            'y_max':  -0.01, # lateral right edge (m)
        }
        self.d_o = 0.15      # safety offset = half bot width + clearance buffer (m)

        # Road boundaries (used as virtual edge vehicles per paper eq. 8, 9)
        self.road_y_left  =  0.5   # left road boundary  (m)
        self.road_y_right = -0.5   # right road boundary (m)

        # 5. Internal State Tracking
        self.last_time      = self.get_clock().now()
        self.last_v         = 0.0
        self.cmd_v          = 0.0
        self.last_pos_x     = None
        self.last_pos_y     = None
        self.last_y_error   = 0.0
        self.last_psi_error = 0.0

        # Pubs and Subs
        self.pose_sub = self.create_subscription(
            PoseArray, '/saye/pose_array', self.control_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/saye/cmd_high_level', 10)

        self.get_logger().info("AMCAF Dynamic Line Follower with Obstacle Avoidance Started.")

    # ------------------------------------------------------------------
    # Paper Section V-B: Reference Lane Estimation
    # Obstacle treated as a static neighbor vehicle.
    # target_y is computed as weighted average of left/right clearance
    # points, weighted by inverse distance (closer = more influence).
    # As distance -> inf, weights -> 0 and target_y -> 0 naturally.
    # ------------------------------------------------------------------
    def compute_target_y(self, curr_x, curr_y):
        obs = self.obstacle

        # Longitudinal distance to obstacle front face
        dist_to_obs = obs['x_min'] - curr_x

        # Only influence target_y when approaching or alongside obstacle
        # Once past (curr_x > obs['x_max']), distance becomes negative
        # and weight -> 0, so target_y naturally returns to 0
        if curr_x > obs['x_max']:
            # Past the obstacle: no influence, return nominal
            return 0.0

        # Distance used for weighting:
        # When approaching: longitudinal distance to front face
        # When alongside:   use a small residual so weight stays finite
        if dist_to_obs > 0:
            d_obs = dist_to_obs                  # approaching
        else:
            d_obs = 0.05                         # alongside, keep weight high

        # --- Paper Eq. 8: Left side ---
        # Left clearance point = right edge of obstacle + safety offset
        # This represents "how far left of the box is safe"
        y_L_obs  = obs['y_max'] + self.d_o      # safe left clearance point
        w_L_obs  = 1.0 / d_obs                  # inverse distance weight

        # Road left edge acts as virtual left boundary vehicle (paper eq. 8)
        d_L_edge = abs(self.road_y_left - curr_y)
        d_L_edge = max(d_L_edge, 0.01)          # avoid division by zero
        y_L_edge = self.road_y_left - self.d_o  # inset from road edge
        w_L_edge = 1.0 / d_L_edge

        y_L_avg = (w_L_obs * y_L_obs + w_L_edge * y_L_edge) / \
                  (w_L_obs + w_L_edge)

        # --- Paper Eq. 9: Right side ---
        # Right clearance point = left edge of obstacle - safety offset
        y_R_obs  = obs['y_min'] - self.d_o      # safe right clearance point
        w_R_obs  = 1.0 / d_obs

        d_R_edge = abs(curr_y - self.road_y_right)
        d_R_edge = max(d_R_edge, 0.01)
        y_R_edge = self.road_y_right + self.d_o
        w_R_edge = 1.0 / d_R_edge

        y_R_avg = (w_R_obs * y_R_obs + w_R_edge * y_R_edge) / \
                  (w_R_obs + w_R_edge)

        # --- Paper Eq. 10: Virtual lane center ---
        y_ref = (y_L_avg + y_R_avg) / 2.0

        return y_ref

    # ------------------------------------------------------------------

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

        curr_x   = msg.poses[0].position.x
        curr_y   = msg.poses[0].position.y
        curr_psi = self.get_yaw_from_quat(msg.poses[0].orientation)

        # Estimate current velocity (v = ds/dt)
        if self.last_pos_x is None:
            v_actual = 0.0
        else:
            dist     = math.sqrt((curr_x - self.last_pos_x)**2 +
                                 (curr_y - self.last_pos_y)**2)
            v_actual = dist / dt

        # --- B. Update target_y using paper's weighted average logic ---
        self.target_y = -round(self.compute_target_y(curr_x, curr_y),1)
        print(self.target_y)

        # --- C. Velocity Scheduling ---
        v_c = self.V_nominal * math.cos(self.kp_y * (self.target_y - curr_y))
        v_c = max(v_c, 0.01)
        v_b = self.V_nominal
        v_s = min(v_c, v_b, self.V_nominal)

        # --- D. Control Law ---
        # 1. Longitudinal: PD (unchanged)
        v_error      = v_s - v_actual
        v_dot        = (v_actual - self.last_v) / dt
        acceleration = (self.kp_v * v_error) + (self.kd_v * -v_dot)
        acceleration = max(min(acceleration, self.max_a), -self.max_a)

        # 2. Lateral: PD with derivative on both errors
        y_error   = self.target_y - curr_y
        psi_error = 0.0 - curr_psi

        y_error_dot   = (y_error   - self.last_y_error)   / dt
        psi_error_dot = (psi_error - self.last_psi_error) / dt

        steering_delta = (self.kp_y   * y_error   + self.kd_y   * y_error_dot) + \
                         (self.kp_psi * psi_error + self.kd_psi * psi_error_dot)

        steering_delta = max(min(steering_delta, self.max_delta), -self.max_delta)

        print(f"x:{curr_x:.2f} target_y:{self.target_y:.3f} "
              f"y_err:{y_error:.3f} psi_err:{psi_error:.3f} delta:{steering_delta:.3f}")

        # --- E. Integration and Publication ---
        self.cmd_v += acceleration * dt
        self.cmd_v  = max(0.0, self.cmd_v)

        cmd           = Twist()
        cmd.linear.x  = float(self.cmd_v)
        cmd.angular.z = float(steering_delta)
        self.cmd_pub.publish(cmd)

        # Update historical values
        self.last_time      = current_time
        self.last_v         = v_actual
        self.last_pos_x     = curr_x
        self.last_pos_y     = curr_y
        self.last_y_error   = y_error
        self.last_psi_error = psi_error


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