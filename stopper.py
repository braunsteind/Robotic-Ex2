#!/usr/bin/python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Stopper(object):
    def __init__(self, forward_speed, rotation_speed, min_angle, max_angle, min_dist_from_obstacle):

        self.forward_speed = forward_speed
        self.rotation_speed = rotation_speed
        self.min_scan_angle = math.radians(min_angle)
        self.max_scan_angle = math.radians(max_angle)
        self.min_dist_from_obstacle = min_dist_from_obstacle

        # member for the rotation direction
        self.direction = 1
        self.turn_around = False
        self.full_rotation = False

        self.keep_moving = True
        self.command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)
        self.round = 25

    def start_moving(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            if not self.keep_moving:
                self.rotate()
            else:
                self.move_forward()
            rate.sleep()

    # function for moving the robot forward
    def move_forward(self):
        move_msg = Twist()
        move_msg.linear.x = self.forward_speed
        self.command_pub.publish(move_msg)


    def scan_callback(self, scan_msg):
        # setting the given angles
        scan_msg.angle_min = self.min_scan_angle
        scan_msg.angle_max = self.max_scan_angle

        # initialize flags for discovering which direction is more free
        left = -1
        right = -1

        # scan for most free angles on left and right sides
        for i in range(320):
            if scan_msg.ranges[i] > self.min_dist_from_obstacle or math.isnan(scan_msg.ranges[i]):
                left = i
            elif scan_msg.ranges[639 - i] > self.min_dist_from_obstacle or math.isnan(scan_msg.ranges[639 - i]):
                right = 639 - i

        # look for obstacles
        for dist in scan_msg.ranges:
            if dist < self.min_dist_from_obstacle:
                # obstacle found, stop moving forward
                self.keep_moving = False
                # obstacles in both sides, turn in the more free direction
                if right != -1 and left != -1:
                    if abs(320 - right) > abs(320 - left):
                        self.direction = 1
                    else:
                        self.direction = -1
                # obstacle in one side, turn to the other
                elif right != -1:
                    self.direction = 1
                # no free way available, turn 90 degrees and look for a way again
                elif right == -1 and left == -1:
                    self.round = 90
                # obstacle in one side, turn to the other
                else:
                    self.direction = -1
                break
            # no obstacle found, still going forward
            else:
                self.keep_moving = True


    # function for rotation
    def rotate(self):
            rot = Twist()
            rot.angular.z = self.direction * math.radians(self.round)
            self.command_pub.publish(rot)
            # self.round = 25
