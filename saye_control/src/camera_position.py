import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header

import cv2
import numpy as np
import math

class ArucoPosePublisher(Node):

    def __init__(self):
        super().__init__('aruco_pose_publisher')

        self.publisher_ = self.create_publisher(PoseArray, '/saye/pose_array', 10)

        self.cap = cv2.VideoCapture(1)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.timer = self.create_timer(0.01, self.process_frame)  

    def yaw_to_quaternion(self, yaw):
        """Convert yaw (theta) to quaternion"""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        h, w, _ = frame.shape

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        # Axis
        y_axis = (h // 2) + 170
        cv2.line(frame, (0, y_axis), (w, y_axis), (255, 255, 0), 2)
        cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)

        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera_frame"

        if ids is not None:
            for i in range(len(ids)):
                corner = corners[i][0]

                # Draw marker
                cv2.polylines(frame, [corner.astype(int)], True, (0, 255, 0), 2)

                cx = int(np.mean(corner[:, 0]))
                cy = int(np.mean(corner[:, 1]))

                # Centered coordinates
                x_centered = -float(cx - w // 2) + 210
                y_centered = float(y_axis - cy)

                # Orientation
                x1, y1 = corner[0]
                x2, y2 = corner[1]

                dx = x2 - x1
                dy = y2 - y1
                theta = math.atan2(dy, dx)

                # Create Pose
                pose = Pose()
                pose.position.x = x_centered*0.0035
                pose.position.y = y_centered*0.0035
                pose.position.z = 0.0

                qx, qy, qz, qw = self.yaw_to_quaternion(theta)
                pose.orientation.x = qx
                pose.orientation.y = qy
                pose.orientation.z = qz
                pose.orientation.w = qw

                pose_array.poses.append(pose)

                # Visualization
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                arrow_length = 50
                end_x = int(cx - arrow_length * math.cos(theta))
                end_y = int(cy - arrow_length * math.sin(theta))
                cv2.arrowedLine(frame, (cx, cy), (end_x, end_y), (0, 0, 255), 2)

                text = f"X:{x_centered:.1f} Y:{y_centered:.1f} Th:{theta:.2f}"
                cv2.putText(frame, text, (cx - 100, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # Publish
        self.publisher_.publish(pose_array)

        cv2.imshow("Aruco Detection (ROS2)", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()