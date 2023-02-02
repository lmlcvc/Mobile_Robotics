# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import math
from math import atan2, pi
import numpy as np

import rclpy
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import Twist, Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnModel, GetWorldProperties
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty

MIN_DISTANCE = np.Infinity
YAW = None
BEAM_THETA = None
CURR_X = None
CURR_Y = None
GOAL_X = None
GOAL_Y = None

STOP_DISTANCE = 0.3
LIN_X = 0.5


def yaw_from_quaternion(q: Quaternion):
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return atan2(t3, t4)


class GoToClosest(Node):

    def __init__(self):
        super().__init__('goto_closest')

        # for odom info
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.lin_x = LIN_X
        self.move = False

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            1
        )
        self.command_publisher  # prevent unused variable warning

        self.marker_publisher = self.create_publisher(
            Marker,
            'visualization_marker',
            1
        )
        self.marker_publisher

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_listener_callback,
            10
        )
        self.odom_subscription

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_listener_callback,
            10
        )
        self.scan_subscription

        self.srv = self.create_service(
            Empty,
            'goto_closest',
            self.closest_service_callback
        )
        self.srv

    def odom_listener_callback(self, msg):
        self.get_logger().info('Start position: "%s"' % msg)
        global GOAL_X, GOAL_Y, YAW, CURR_X, CURR_Y, LIN_X

        self.get_logger().info(f'self.move = {self.move}')
        if (GOAL_X is None) or (GOAL_Y is None) or (not self.move):
            return

        # velocities calc and publish
        instruction = Twist()

        # calculate diffs to goal
        CURR_X = msg.pose.pose.position.x
        CURR_Y = msg.pose.pose.position.y

        x_diff = GOAL_X - CURR_X
        y_diff = GOAL_Y - CURR_Y
        total_diff = math.sqrt(x_diff * x_diff + y_diff * y_diff)
        goal_theta = atan2(y_diff, x_diff)

        # current angles
        orientation_q = msg.pose.pose.orientation
        YAW = yaw_from_quaternion(orientation_q)

        angle_diff = goal_theta - YAW
        if angle_diff > 2 * pi:
            angle_diff = angle_diff - 2 * pi

        elif angle_diff < -2 * pi:
            angle_diff = angle_diff + 2 * pi

        if abs(angle_diff) > pi:
            angle_diff = pi - angle_diff

        self.get_logger().info(f'Angle diff: {angle_diff * (180.0 / pi)}')
        self.get_logger().info(f'Yaw: {YAW * (180.0 / pi)}')
        self.get_logger().info(f'Goal theta: {goal_theta * (180.0 / pi)}')

        if abs(angle_diff) > 0.1:  # if not approx good angle
            if abs(x_diff) <= 0.05 and abs(y_diff) <= 0.05:  # close to goal
                instruction.linear.x = 0.0
                instruction.angular.z = 0.0
            else:  # fix rotation
                instruction.linear.x = 0.0
                instruction.angular.z = 0.5 * (goal_theta - YAW)
        else:  # good rotation, move linear
            if total_diff <= 1.2 * STOP_DISTANCE:  # start decelerating close to goal
                instruction.linear.x = self.lin_x
                self.lin_x *= 0.5  # deceleration
                instruction.angular.z = 0.0

                if total_diff <= STOP_DISTANCE:  # stopping condition when goal met
                    instruction.linear.x = 0.0
                    instruction.angular.z = 0.0
                    self.move = False
            else:
                self.lin_x = LIN_X
                instruction.linear.x = self.lin_x
                instruction.angular.z = 0.5 * (goal_theta - YAW)

        self.command_publisher.publish(instruction)

        self.get_logger().info('New position: "%s"' % msg)

    def scan_listener_callback(self, msg):
        if not msg.ranges:
            return

        global GOAL_X, GOAL_Y, CURR_X, CURR_Y, YAW, BEAM_THETA, MIN_DISTANCE

        try:  # request odometry info
            trans = self.tfBuffer \
                .lookup_transform('odom',
                                  'base_scan',
                                  rclpy.time.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        CURR_X = trans.transform.translation.x
        CURR_Y = trans.transform.translation.y
        YAW = yaw_from_quaternion(trans.transform.rotation)

        # get pillar coords
        MIN_DISTANCE = min(msg.ranges)
        if MIN_DISTANCE >= np.Infinity:
            return

        BEAM_THETA = msg.ranges.index(MIN_DISTANCE) * msg.angle_increment
        GOAL_X = CURR_X + MIN_DISTANCE * math.cos(BEAM_THETA + YAW)
        GOAL_Y = CURR_Y + MIN_DISTANCE * math.sin(BEAM_THETA + YAW)

        self.get_logger().info(f"Goal coords: {GOAL_X}, {GOAL_X}")

        # mark goal
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.type = 3  # cylinder
        marker.id = 0
        marker.action = Marker.ADD

        self.get_logger().info(f"Marker: {Marker}")

        # marker coords
        marker.pose.position.x = GOAL_X
        marker.pose.position.y = GOAL_Y
        marker.pose.position.z = 0.0

        # marker orientations
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker visuals
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.7

        self.marker_publisher.publish(marker)

        self.get_logger().info(f'Goal position: {GOAL_X}, {GOAL_Y}')

    def closest_service_callback(self, request, response):
        self.move = True
        self.get_logger().info(f'Setting service: self.move = {self.move}')
        return response


def main(args=None):
    rclpy.init(args=args)

    goto_closest = GoToClosest()

    rclpy.spin(goto_closest)

    goto_closest.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
