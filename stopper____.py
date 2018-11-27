#!/usr/bin/python
#
# stopper.py
#
#  Created on: Nov 13, 2017
#      Author: Mika Barkan
#

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Stopper(object):
    def __init__(self, forward_speed, rotation_speed, min_angle, max_angle):
        # getting values from launch file
        self.forward_speed = forward_speed
        self.rotation_speed = rotation_speed
        # getting angles
        self.min_scan_angle = min_angle / 180 * math.pi
        self.max_scan_angle = max_angle / 180 * math.pi

        self.min_dist_from_obstacle = forward_speed + 0.25
        self.keep_moving = True

        self.command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)
        self.turn_angle_value = 0



    def start_moving(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.keep_moving:
                self.move_forward()
            else:
                self.rotate()
            rate.sleep()


    # this functions handles with rotating the robot
    def rotate(self):
        if self.turn_angle_value != 0:
            vel_msg = Twist()
            angular_speed = self.forward_speed
            #self.turn_angle_value = self.turn_angle_value*2*PI/360
            vel_msg.angular.z = angular_speed
            t0 = rospy.Time.now().to_sec()
            current_angle = 0
            while current_angle < abs(self.turn_angle_value):
                self.command_pub.publish(vel_msg)
                t1 = rospy.Time.now().to_sec()
                current_angle = angular_speed * (t1 - t0)

            self.turn_angle_value = 0
            self.keep_moving = True





    def move_forward(self):

        move_msg = Twist()
        move_msg.linear.x = self.forward_speed
        if self.keep_moving:
            self.command_pub.publish(move_msg)





    def scan_callback(self, scan_msg):

        scan_msg.angle_min = self.min_scan_angle
        scan_msg.angle_max = self.max_scan_angle
        obstacle_found = False

        if self.keep_moving:

            # check if an obstalce is nearby
            for dist in scan_msg.ranges:
                if dist < self.min_dist_from_obstacle:
                    self.keep_moving = False
                    obstacle_found = True
                    break

            # obstacle found, search for clear path
            if obstacle_found:
                left_clear_index = 0
                right_clear_index = 0

                # check in the left field of view
                for i in range(319, -1, -1):
                    if math.isnan(scan_msg.ranges[i]):
                        left_clear_index = i
                        break

                if left_clear_index == 0:
                    for i in range(319, -1, -1):
                        if scan_msg.ranges[i] > self.min_dist_from_obstacle:
                            left_clear_index = i
                            break


                # check in the right field of view
                for j in range(320, 640, 1):
                    if math.isnan(scan_msg.ranges[j]):
                        right_clear_index = j
                        break

                if right_clear_index == 0:
                    for j in range(320, 640, 1):
                        if scan_msg.ranges[j] > self.min_dist_from_obstacle:
                            right_clear_index = j
                            break

                print "left is " + str(left_clear_index)
                print "right is " + str(right_clear_index)
                # check to which direction should rotate
                if abs(left_clear_index - 320) > abs(right_clear_index - 320):
                    direction = -1
                else:
                    direction = 1


                if direction == 1:
                    self.turn_angle_value = self.min_scan_angle + (left_clear_index * (self.max_scan_angle - self.min_scan_angle) / len(scan_msg.ranges))
                else:
                    self.turn_angle_value = -1 * self.min_scan_angle + (right_clear_index * (self.max_scan_angle - self.min_scan_angle) / len(scan_msg.ranges))

                #i = 0
                #for dist in scan_msg.ranges:
                    #if dist > self.min_dist_from_obstacle or math.isnan(dist):
                     #   self.turn_angle_value = direction * self.min_scan_angle + (i * (self.max_scan_angle - self.min_scan_angle) / len(scan_msg.ranges))
                     #   break
                   # i += 1
