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
        self.obstacle = False

        self.command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)

        self.round = 5
        self.angular_speed = rotation_speed * PI / 180
        self.relative_angle = self.round * PI / 180

    def find_direction(self, ranges):
        # nan_left = 0
        # nan_right = 0
        max_distance = 0
        i = 0
        index = 0
        size = len(ranges)
        # loop on ranges
        while i < size:
            # # if nan update nan counter
            # if math.isnan(ranges[i]):
            #     # if left side, update left counter
            #     if i < size / 2:
            #         nan_left += 1
            #     # right side, update right counter
            #     else:
            #         nan_right += 1

            # if clear
            if math.isnan(ranges[i]):
                if i < size / 2:
                    self.direction = 1
                else:
                    self.direction = -1
                break
            # if not nan, check for max distance
            if ranges[i] > max_distance:
                max_distance = ranges[i]
                index = i
            i += 1

        # # if left is clear
        # if nan_left > nan_right * nan_right:
        #     self.direction = 1
        # # if right is clear
        # elif nan_right > nan_left * nan_left:
        #     self.direction = -1
        # set the direction to turn based on max value
        # else:
        if index < size / 2:
            self.direction = 1
        else:
            self.direction = -1

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
        # is there an obstacle
        self.obstacle = False
        # go over the ranges
        for dist in scan_msg.ranges:
            # obstacle found
            if dist < self.min_dist_from_obstacle:
                # stop moving
                self.keep_moving = False
                self.obstacle = True
                # find the direction
                self.find_direction(scan_msg.ranges)
                break
        # if no obstacle
        if not self.obstacle:
            # keep moving
            self.keep_moving = True

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
