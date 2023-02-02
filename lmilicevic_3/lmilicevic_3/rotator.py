from typing import Set

from numpy.core.defchararray import upper
import rclpy
from rclpy.node import Node
from rclpy.utilities import get_available_rmw_implementations
from geometry_msgs.msg import Twist, Point
from ament_index_python.packages import get_package_share_directory

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from zad32.blob_detector import *
import time

COL_COUNT = 60
ROW_COUNT = 60
WIN_MIN = 0.0
WIN_MAX = 1.0
BLUR = 5

ROTATE_DIST = 1.5
ROTATE_MIN = -0.5
ROTATE_MAX = 0.5
LINEAR_VEL = 0.0

COLOUR_MIN = 0
BLUE_MIN = 60
COLOUR_MAX = 255


def find_mid(value, minv, maxv):
    vlist = [value, minv, maxv]
    value_list = vlist.sort()
    return value_list[1]


class Rotator(Node):
    def __init__(self, min_threshold, max_threshold, blur=15, blob_params=None, detection_window=None):
        super().__init__('rotator')

        # fields
        self.max_threshold = max_threshold
        self._min_threshold = min_threshold
        self._blur = blur
        self._blob_params = blob_params
        self.detection_window = detection_window
        self.blob = Point()

        # publishers
        self.command_publisher = self.create_publisher(
            Twist,
            '/robot/cmd_vel',
            10
        )

        # subscriptions
        self.camera_subscriber = self.create_subscription(
            Image,
            '/robot/camera/image_raw',
            self.camera_callback,
            10
        )

    def camera_callback(self, msg):
        global COL_COUNT, ROW_COUNT, ROTATE_DIST, ROTATE_MIN, ROTATE_MAX, LINEAR_VEL

        img = CvBridge().imgmsg_to_cv2(msg)
        frame = cv2.inRange(img,
                            (COLOUR_MIN, COLOUR_MIN, BLUE_MIN),
                            (COLOUR_MAX, COLOUR_MAX, COLOUR_MAX)
                            )

        (rows, cols, _) = img.shape
        if cols > COL_COUNT and rows > ROW_COUNT:
            keypoints, frame = blob_detect(frame,
                                           self.min_threshold,
                                           self.max_threshold,
                                           0,
                                           blob_params=self._blob_params,
                                           search_window=self.detection_window,
                                           imshow=False)
            frame = draw_window(frame, self.detection_window)
            frame = draw_keypoints(frame, keypoints)
            cv2.imshow("FRAME", frame)
            cv2.waitKey(10)

            x, y = get_blob_relative_position(frame, keypoints[0])
            self.blob.x = x
            self.blob.y = y

        # get needed angular vel value
        rotate = -1.0 * ROTATE_DIST * self.blob.x
        rotate = find_mid(rotate, ROTATE_MIN, ROTATE_MAX)

        # send twist command
        message = Twist()
        message.linear.x = message.linear.y = LINEAR_VEL
        message.angular.z = rotate
        self.command_publisher.publish(message)


def main(args=None):
    min_thr = (COLOUR_MIN, COLOUR_MIN, BLUE_MIN)
    max_thr = (COLOUR_MAX, COLOUR_MAX, COLOUR_MAX)

    params = cv2.SimpleBlobDetector_Params()

    detection_window = [WIN_MIN, WIN_MIN, WIN_MAX, WIN_MAX]

    rclpy.init(args=args)
    rotator = Rotator(min_thr, max_thr, BLUR, params, detection_window)
    rclpy.spin(rotator)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
