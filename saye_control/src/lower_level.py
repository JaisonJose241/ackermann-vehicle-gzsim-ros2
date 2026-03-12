import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class AckermannManualController(Node):
    def __init__(self):
        super().__init__('ackermann_manual_controller')
        
        # Physical Constants from saye model.sdf
        self.L = 0.2255  # Wheelbase
        self.W = 0.2     # Wheel separation (Track width)
        self.r = 0.0365  # Wheel radius
        
        # Publishers
        self.pub_steer_l = self.create_publisher(Float64, '/saye/steering_left', 10)
        self.pub_steer_r = self.create_publisher(Float64, '/saye/steering_right', 10)
        self.pub_drive_l = self.create_publisher(Float64, '/saye/drive_left', 10)
        self.pub_drive_r = self.create_publisher(Float64, '/saye/drive_right', 10)
        
        # Timer for control loop (50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        # Target variables (Change these to test or link to your MPC/RL logic)
        self.target_v = 0.0      # Linear velocity in m/s
        self.target_delta = 0.1  # Steering angle in radians

    def control_loop(self):
        v = self.target_v
        delta = self.target_delta

        # 1. Calculate Steering Angles (Ackermann Geometry)
        # delta_i = atan(L*tan(delta) / (L - W/2 * tan(delta)))
        # delta_o = atan(L*tan(delta) / (L + W/2 * tan(delta)))
        
        if delta == 0:
            steer_l = 0.0
            steer_r = 0.0
        else:
            # Inner wheel turns sharper than the outer wheel
            common_term = self.L / math.tan(abs(delta))
            if delta > 0: # Turning Left
                steer_l = math.atan(self.L / (common_term - self.W/2.0))
                steer_r = math.atan(self.L / (common_term + self.W/2.0))
            else: # Turning Right
                steer_l = -math.atan(self.L / (common_term + self.W/2.0))
                steer_r = -math.atan(self.L / (common_term - self.W/2.0))

        # 2. Calculate Wheel Angular Velocity (m/s to rad/s)
        # omega = v / r
        wheel_speed_rad_s = v / self.r

        # 3. Publish commands
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
    node = AckermannManualController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()