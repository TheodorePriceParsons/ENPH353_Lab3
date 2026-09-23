#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, 'image_topic',
                                                  self.callback, 1)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

    def callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        #TODO fill in the details for processing an image to determine the position
        # of the line in the image and publishing a command to move the robot forward
        # while the line is in the center.


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
