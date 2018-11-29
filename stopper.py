#!/usr/bin/python

PI = 3.1415926535897

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Stopper(object):
    # constructor
    def __init__(self, forward_speed, rotation_speed, min_angle, max_angle, min_dist_from_obstacle):
        # extract info
        self.forward_speed = forward_speed
        self.rotation_speed = rotation_speed
        self.min_scan_angle = math.radians(min_angle)
        self.max_scan_angle = math.radians(max_angle)
        self.min_dist_from_obstacle = min_dist_from_obstacle
        # default values
        self.direction = 1
        self.keep_moving = True

        self.command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)

        self.round = 1
        self.angular_speed = rotation_speed * PI / 180
        self.relative_angle = self.round * PI / 180

    def findDirection(self, ranges):
        max_distance = 0
        index = 0
        # loop over ranges
        for i in ranges:
            # find the max distance
            if ranges[i] > max_distance:
                max_distance = ranges[i]
                index = i
        # return the direction to turn
        if index < (len(ranges) / 2):
            return 1
        return -1

    def start_moving(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.keep_moving:
                self.move_forward()
            else:
                self.rotate()
            rate.sleep()

    # handling the forward movement
    def move_forward(self):
        move_msg = Twist()
        move_msg.linear.x = self.forward_speed
        self.command_pub.publish(move_msg)

    def scan_callback(self, scan_msg):
        scan_msg.angle_min = self.min_scan_angle
        scan_msg.angle_max = self.max_scan_angle

        # go over the ranges
        for dist in scan_msg.ranges:
            # obstacle found
            if dist < self.min_dist_from_obstacle:
                # stop moving
                self.keep_moving = False
                # find the direction
                self.direction = self.findDirection(scan_msg.ranges)

    # handling rotation
    def rotate(self):
        # initializing values
        rot = Twist()
        rot.angular.z = self.direction * self.angular_speed
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        # rotate until reached wanted degree
        while (current_angle < self.relative_angle):
            self.command_pub.publish(rot)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.angular_speed * (t1 - t0)
        # keep moving
        self.keep_moving = True
