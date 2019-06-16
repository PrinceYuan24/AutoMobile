#!/usr/bin/env python

import rospy
import threading
import signal

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class PosMarkers():
    def __init__(self):
        self.FILEPATH = 'goals.txt'
        rospy.init_node('pos_markers', anonymous=True, disable_signals=True)

        self.pos = Marker()
        self.data = self.generateData(self.FILEPATH)
        self.poses = []
        self.callback(self.data)
        #rospy.Subscriber('/sim_car_pose/pose', PoseStamped, self.callback)

    def callback(self, data):
        self.pos.header.frame_id = "map"
        self.pos.id+=1
        self.pos.header.stamp = rospy.Time();
        self.pos.action = Marker.ADD;
        self.pos.scale.x = 1
        self.pos.scale.y = 0.05
        self.pos.scale.z = 0.05
        self.pos.color.b = 1.0
        self.pos.color.a = 1.0
        self.pos.lifetime.secs = 100

        for d in data:
            self.pos.pose.position.x = d[0]
            self.pos.pose.position.y = d[1]
            self.poses.append(self.pos)
            #self.pos.pose.
            #self.pos.pose.orientation = data.pose.orientation

    def str2float(self, goal):
        newgoal = []
        newgoal.append(float(goal[0]))
        newgoal.append(float(goal[1]))
        goal[2] = goal[2].replace('/n', '')
        newgoal.append(float(goal[2]))
        return newgoal

    def generateData(self, filepath):
        f = open(filepath, "r")
        data = []
        for x in f:
            goal = x.split(',')
            data.append(self.str2float(goal))
        return data

if __name__ == '__main__':
    sub = PosMarkers()

    r = rospy.Rate(2)
    pub_pose = rospy.Publisher('/pose_markers/markers', Marker, queue_size=100)

    while not rospy.is_shutdown():
        for pos in sub.poses:
            print(pos)
            pub_pose.publish(pos)
            print(2)
        r.sleep()

    # while not rospy.is_shutdown():

    #     pub_pose.publish(sub.pos)
    #     r.sleep()

    rospy.spin()
