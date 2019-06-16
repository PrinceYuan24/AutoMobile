#!/usr/bin/env python

import rospy
import threading
import signal
import utils

from std_msgs.msg import String, Float64
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty



class Subscriber():
    def __init__(self):
        rospy.init_node('Subscriber', anonymous=True, disable_signals=True)
        
        self.pos = PoseWithCovarianceStamped()
        self.vs = AckermannDriveStamped()

        rospy.Service(rospy.get_param('~s_reset'), Empty, self.reset)

        rospy.Subscriber('/lab0/initpose', String, self.cb_pose)
        rospy.Subscriber('/lab0/velocity', Float64, self.cb_vel)
        rospy.Subscriber('/lab0/heading', Float64, self.cb_angle)

    def reset(self, empty):
        self.vs = AckermannDriveStamped()
        return []


    def cb_pose(self, data):
        temp = data.data
        temp = temp.split(', ')
        self.pos.pose.pose.position.x = float(temp[0])
        self.pos.pose.pose.position.y = float(temp[1])
        self.pos.pose.pose.orientation = utils.angle_to_quaternion(float(temp[2]))

        rospy.loginfo(rospy.get_caller_id() + "~t_init " + data.data)

    def cb_vel(self, data):
        self.vs.drive.speed = data.data

        rospy.loginfo(rospy.get_caller_id() + "~t_velocity %s", data.data)

    def cb_angle(self, data):
        self.vs.drive.steering_angle = data.data
        rospy.loginfo(rospy.get_caller_id() + "~t_angle %s", data.data)

if __name__ == '__main__':
    sub = Subscriber()

    r = rospy.Rate(20)
    pub_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    pub_vs = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=1)

    pub_pose.publish(sub.pos)
    while not rospy.is_shutdown():
        pub_vs.publish(sub.vs)
        r.sleep()

    rospy.spin()