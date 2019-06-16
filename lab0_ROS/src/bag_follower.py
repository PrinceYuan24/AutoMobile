#!/usr/bin/env python

import rospy
import threading
import signal

import rosbag
from std_msgs.msg import String, Float64
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('bag_follower', anonymous=True, disable_signals=True)

REVERSE = rospy.get_param("~reverse")

bagpath = rospy.get_param("~plan_file")
bag = rosbag.Bag(bagpath)

pub_vs = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=1)
r = rospy.Rate(20)

temp=[]

#if rospy.get_param('~direc')== "F": 
for topic, msg, t in bag.read_messages():
    if not rospy.is_shutdown():
        if REVERSE:
            msg.drive.speed=-msg.drive.speed
        pub_vs.publish(msg)
        r.sleep()

rospy.spin()
bag.close()
