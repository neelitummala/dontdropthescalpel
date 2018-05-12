#!/usr/bin/env python
# Some code is from takktile_example_code.py

from math import pi, cos
from reflex_msgs.msg import PoseCommand

import rospy
from std_srvs.srv import Empty

def open_hand():
    disable_tactile_stops = rospy.ServiceProxy('/reflex_takktile/disable_tactile_stops', Empty)
    pos_pub = rospy.Publisher('/reflex_takktile/command_position', PoseCommand, queue_size=1)
    disable_tactile_stops()
    pos_pub.publish(PoseCommand(preshape=0.0))

def calibrate_tactile():
    calibrate_tactile = rospy.ServiceProxy('/reflex_takktile/calibrate_tactile', Empty) # calibrate tactile sensors
    calibrate_tactile()
    print("Done calibrating tactile sensors.")

def calibrate_fingers():
    calibrate_fingers = rospy.ServiceProxy('/reflex_takktile/calibrate_fingers_manual', Empty) # cannot calibrate fingers automatically
    calibrate_fingers()