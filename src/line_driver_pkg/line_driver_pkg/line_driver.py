#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw',
                                                  self.callback, 1)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('search_speed', 0.4)
        self.declare_parameter('threshold', 100)
        self.declare_parameter('bottom_frac', 0.10)
 
        self.last_error = 0.0

    def find_line_center(self, frame):
        """!
        @brief find the center of the line. 
 
        I used the same approach I used in lab2. It finds the largest connected area(the line) finds its center. 
 
        @param frame BGR image.
        @return (center_x or None, mask) where mask is the thresholded image.
        """
        threshold = int(self.get_parameter('threshold').value)
        bottom_frac = self.get_parameter('bottom_frac').value
 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray, 0, threshold - 1)
 
        h, w = mask.shape
        bottom_start = int((1 - bottom_frac) * h)
 
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
 
        # Components that appear in the bottom strip (label 0 is background).
        bottom_labels = np.unique(labels[bottom_start:h, :])
        bottom_labels = bottom_labels[bottom_labels != 0]
 
        if len(bottom_labels) == 0:
            return None, mask
 
        # Largest blob wins, which rejects small noise specks.
        chosen = max(bottom_labels, key=lambda lbl: stats[lbl, cv2.CC_STAT_AREA])
 
        _, xs = np.where(labels[bottom_start:h, :] == chosen)
        if len(xs) == 0:
            return None, mask
 
        return int((xs.min() + xs.max()) / 2), mask
    def callback(self, data):
        """!
        @brief Process one camera frame and publish a velocity command.
 
        @param data sensor_msgs/Image from /camera/image_raw.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return
 
        h, w = cv_image.shape[:2]
        center_x, mask = self.find_line_center(cv_image)
        move = Twist()
 
        if center_x is not None:
            error = center_x - w / 2.0
            self.last_error = error
            move.linear.x = float(self.get_parameter('linear_speed').value)
            move.angular.z = float(-self.get_parameter('kp').value * error)
            cv2.circle(cv_image, (center_x, int(0.95 * h)), 7, (0, 0, 255), -1)
        else:
            # Line lost: spin toward where it was last seen.
            direction = -1.0 if self.last_error > 0 else 1.0
            move.angular.z = direction * float(self.get_parameter('search_speed').value)
            cv2.putText(cv_image, "LINE LOST", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
 
        self.cmd_vel_pub.publish(move)
 
        cv2.imshow('camera', cv_image)
        cv2.imshow('mask', mask)
        cv2.waitKey(1)





def main(args=None):
    print("Starting...")
    rclpy.init(args=args)
    lf = LineFollower()
    try:
        rclpy.spin(lf)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
        lf.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
