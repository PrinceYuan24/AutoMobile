#!/usr/bin/env python

import rospy
import threading
import signal
import numpy as np

import rosbag
from std_msgs.msg import String, Float64
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

THRESHOLD = 0.5

class scan_safety:
	def __init__(self):
		rospy.init_node('scan_safety', anonymous=True, disable_signals=True)
		self.mindis = 0
		self.scan = rospy.Subscriber('/scan', LaserScan, self.cb_scan)
		self.pub_vs = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=1)
		self.vs = AckermannDriveStamped()
		self.vs.header.frame_id = 'safe'
			
	def cb_scan(self, data):
		temp = []
		ranges = data.ranges
		intensities = data.intensities
		for i in range(0, len(intensities)):
			if abs(intensities[i] - 10.0) < 1e-3:
				temp.append(ranges[i])
		self.mindis = min(temp)
		#print(self.mindis)

if __name__ == '__main__':
	ss = scan_safety()
	r = rospy.Rate(20)
	while not rospy.is_shutdown():
		print(ss.mindis)
		if (ss.mindis < THRESHOLD):
			ss.vs.drive.speed = 0.0
			ss.pub_vs.publish(ss.vs)
		r.sleep()

	rospy.spin()