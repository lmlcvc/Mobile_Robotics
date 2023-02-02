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

import rclpy
from rclpy.node import Node

import numpy as np
from math import atan2, pi

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion

# from tf_transformations import euler_from_quaternion


GOAL_X = None
GOAL_Y = None


def yaw_from_quaternion(q: Quaternion):
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return atan2(t3, t4)


class NavSubscriber(Node):

    def __init__(self):
        super().__init__('nav_subscriber')
        
        self.lin_x = 0.5

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            1
        )
        self.command_publisher  # prevent unused variable warning

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_listener_callback,
            10
        )
        self.odom_subscription

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_listener_callback,
            10
        )
        self.goal_subscription

    def odom_listener_callback(self, msg):
        self.get_logger().info('Start position: "%s"' % msg)

        global GOAL_X, GOAL_Y
        if (GOAL_X is None) or (GOAL_Y is None):
            return

        # velocities calc and publish
        instruction = Twist()

        # calculate diffs to goal
        current = Point()
        current.x = msg.pose.pose.position.x
        current.y = msg.pose.pose.position.y

        x_diff = GOAL_X - current.x
        y_diff = GOAL_Y - current.y
        goal_theta = atan2(y_diff, x_diff)

        # current angles
        orientation_q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(orientation_q)

        angle_diff = goal_theta - yaw
        if angle_diff > 2*pi:
            angle_diff = angle_diff - 2*pi    
            
        elif angle_diff < -2*pi:
            angle_diff = angle_diff + 2*pi  
            
        if abs(angle_diff) > pi:
            angle_diff = pi - angle_diff   	
            
        self.get_logger().info(f'Angle diff: {angle_diff * (180.0 / pi)}')
        self.get_logger().info(f'Yaw: {yaw * (180.0 / pi)}')
        self.get_logger().info(f'Goal theta: {goal_theta * (180.0 / pi)}')

        if abs(angle_diff) > 0.1:  # if not approx good angle
            if abs(x_diff) <= 0.05 and abs(y_diff) <= 0.05:  # close to goal
                instruction.linear.x = 0.0
                instruction.angular.z = 0.0
            else:  # fix rotation
                instruction.linear.x = 0.0
                instruction.angular.z = 0.5 * (goal_theta - yaw)
        else:  # good rotation, move linear
            if abs(x_diff) <= 0.1 and abs(y_diff) <= 0.1:  # close to goal
                instruction.linear.x = self.lin_x
                self.lin_x *= 0.75  # deceleration
                instruction.angular.z = 0.0
            else:
            	instruction.linear.x = self.lin_x
            	instruction.angular.z = 0.5 * (goal_theta - yaw)

        self.command_publisher.publish(instruction)

        self.get_logger().info('New position: "%s"' % msg)


    def goal_listener_callback(self, msg):
        # save msg coordinates to goal variables
        global GOAL_X, GOAL_Y
        GOAL_X = msg.pose.position.x
        GOAL_Y = msg.pose.position.y

        self.get_logger().info(f'Goal position: {GOAL_X}, {GOAL_Y}')


def main(args=None):
    rclpy.init(args=args)

    nav_subscriber = NavSubscriber()

    rclpy.spin(nav_subscriber)

    nav_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
