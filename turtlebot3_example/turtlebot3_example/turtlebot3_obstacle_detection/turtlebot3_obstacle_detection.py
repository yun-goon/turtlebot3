#!/usr/bin/env python3
#
# Copyright 2023 ROBOTIS CO., LTD.
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
#
# Authors: Jeonggeun Lim, Gilbert


import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print("TurtleBot3 Obstacle Detection")
        print("----------------------------------------------")
        print("stop angle: -90 ~ 90 deg")
        print("stop distance: 0.5 m")
        print("----------------------------------------------")

        self.scan_ranges = []
        self.has_scan_received = False

        self.stop_distance = 0.5  # m

        self.tele_twist = Twist()
        
        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos_profile=qos_profile_sensor_data)

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def timer_callback(self):
        if self.has_scan_received is True:
            self.detect_obstacle()

    def detect_obstacle(self):
        twist = Twist()
        left_range = int(len(self.scan_ranges) / 4)
        right_range = int(len(self.scan_ranges) * 3 / 4)

        obstacle_distance = \
            min(
                min(self.scan_ranges[0:left_range]),
                min(self.scan_ranges[right_range:360])
            )

        if obstacle_distance < self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = self.tele_twist.angular.z
            self.get_logger().info( \
                "Turtlebot3 has stopped due to obstacles", throttle_duration_sec=3)
        else:
            twist = self.tele_twist

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
