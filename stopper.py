#!/usr/bin/python
#
# stopper.py
#
#  Created on: Nov 13, 2017
#      Author: Mika Barkan
#

PI = 3.1415926535897

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Stopper(object):
    def __init__(self, forward_speed, rotation_speed, min_angle, max_angle, min_dist_from_obstacle):
        self.forward_speed = forward_speed
        self.rotation_speed = rotation_speed

        # self.min_scan_angle = min_angle / 180 * math.pi
        # self.max_scan_angle = max_angle / 180 * math.pi

        self.min_scan_angle = math.radians(min_angle)
        self.max_scan_angle = math.radians(max_angle)
        self.min_dist_from_obstacle = 0.65
        self.direction = 1

        self.keep_moving = True
        self.command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)

        self.round = 25
        self.angular_speed = rotation_speed * PI / 180
        self.relative_angle = self.round * PI / 180

    def start_moving(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            if not self.keep_moving:
                self.rotate()
            else:
                self.move_forward()
            rate.sleep()

    def move_forward(self):
        move_msg = Twist()
        move_msg.linear.x = self.forward_speed
        self.command_pub.publish(move_msg)

    def scan_callback(self, scan_msg):
        scan_msg.angle_min = self.min_scan_angle
        scan_msg.angle_max = self.max_scan_angle

        # initialize flags for discovering which direction is more free
        left = -1
        right = -1

        for i in range(320):
            if scan_msg.ranges[i] > self.min_dist_from_obstacle or math.isnan(scan_msg.ranges[i]):
                left = i
            elif scan_msg.ranges[639 - i] > self.min_dist_from_obstacle or math.isnan(scan_msg.ranges[639 - i]):
                right = 639 - i

        for dist in scan_msg.ranges:
            if dist < self.min_dist_from_obstacle:
                self.keep_moving = False
                if right != -1 and left != -1:
                    if abs(320 - right) > abs(320 - left):
                        self.direction = 1
                    else:
                        self.direction = -1
                elif right != -1:
                    self.direction = 1
                else:
                    self.direction = -1
                break
            else:
                self.keep_moving = True

    def rotate(self):
        # rot = Twist()
        # rot.angular.z = self.direction * math.radians(25)
        # self.command_pub.publish(rot)

        # initializing values
        rot = Twist()
        rot.angular.z = abs(self.angular_speed)
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        # rotate until reached wanted degree
        while (current_angle < self.relative_angle):
            self.command_pub.publish(rot)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.angular_speed * (t1 - t0)
