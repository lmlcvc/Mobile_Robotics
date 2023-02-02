import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np


PACKAGE_NAME = 'trackme'


class TrackMe(Node):
    def __init__(self):
        super().__init__(PACKAGE_NAME)

        self.move_center = 3
        self.move_radius_static = 0.4
        self.move_radius = 0.4
        self.move_radius_step = 0.05

        self.vx = 0.0
        self.vy = 0.0
        self.vmax = 1.0

        self.rx = 0
        self.ry = 0

        self.twist_publisher = self.create_publisher(Twist, '/bb/cmd_vel', 1)
        self.timer = self.create_timer(1, self.timer_callback)
        self.create_subscription(Odometry, '/bb/odom', self.odometry_callback, 1)

    def timer_callback(self):
        self.move_radius += self.move_radius_step

    def odometry_callback(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y

        # Distance and angle to origin
        distance = np.sqrt((self.rx - self.move_center) ** 2 * self.move_radius ** 2 / self.move_radius_static ** 2 + self.ry ** 2)
        angle = np.arctan2(-self.ry, (self.move_center - self.rx) * self.move_radius / self.move_radius_static)

        # Force towards origin
        back_force = (distance - self.move_radius) * 0.2
        if back_force < 0:
            back_force = 0

        self.vx += np.random.randn() * 0.1 + back_force * np.cos(angle)
        self.vy += np.random.randn() * 0.1 + back_force * np.sin(angle)

        if self.vx >  self.vmax:
            self.vx =  self.vmax
        elif self.vx < -self.vmax:
            self.vx = -self.vmax

        if self.vy >  self.vmax:
            self.vy =  self.vmax
        elif self.vy < -self.vmax:
            self.vy = -self.vmax

        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.twist_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    trackme = TrackMe()
    rclpy.spin(trackme)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
