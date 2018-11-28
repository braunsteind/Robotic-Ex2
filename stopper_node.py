#!/usr/bin/python
#
# stopper_node.py
#
#  Created on: Nov 13, 2017
#      Author: Mika Barkan
#
import rospy, sys
from stopper import Stopper
if __name__ == "__main__":
    rospy.init_node("stopper_node", argv=sys.argv)
    forward_speed = 0.5
    rotation_speed = 0.5
    min_angle = -30
    max_angle = 30
    min_dist_from_obstacle = 0.65

    if rospy.has_param('~forward_speed'):
        forward_speed = rospy.get_param('~forward_speed')
    if rospy.has_param('~rotation_speed'):
        rotation_speed = rospy.get_param('~rotation_speed')
    if rospy.has_param('~min_angle'):
        min_angle = rospy.get_param('~min_angle')
    if rospy.has_param('~max_angle'):
        max_angle = rospy.get_param('~max_angle')
    if rospy.has_param('~min_dist_from_obstacle'):
        min_dist_from_obstacle = rospy.get_param('~min_dist_from_obstacle')
    my_stopper = Stopper(forward_speed, rotation_speed, min_angle, max_angle, min_dist_from_obstacle)
    my_stopper.start_moving()
