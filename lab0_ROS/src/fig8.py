#!/usr/bin/env python

import rospy
import threading
import signal
import numpy as np

import rosbag
from std_msgs.msg import String, Float64
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('bag_follower', anonymous=True, disable_signals=True)
pub_vs = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=1)

r = rospy.Rate(20)
MAGIC_NUMER = 1.35

v = 2.0
s = 0.34

# Unit in inch
L = 11.5
# Transfer to m
L = L*2.54/100

R = L/np.tan(s)
w = v/R
steps=int(2 * np.pi/w * 20 * MAGIC_NUMER)
vs = AckermannDriveStamped()

for i in range(steps):
    if not rospy.is_shutdown():
        vs.drive.speed = v
        vs.drive.steering_angle = s
        pub_vs.publish(vs)
        r.sleep()

s = -s
for i in range(steps):
    if not rospy.is_shutdown():
        vs.drive.speed = v
        vs.drive.steering_angle = s
        pub_vs.publish(vs)
        r.sleep()


rospy.spin()