#!/usr/bin/env python

from nav_msgs.srv import GetPlan
from nav_msgs.srv import GetMap

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from nav_msgs.msg import Odometry

from lab2.msg import XYHV, XYHVPath
from lab2.srv import FollowPath

import numpy as np
from scipy import signal
import rospy
import os
from matplotlib import pyplot as plt
import networkx as nx

from DubinsMapEnvironment import DubinsMapEnvironment
from MapEnvironment import MapEnvironment
from DubinsSampler import DubinsSampler
import graph_maker
import util
import lazy_astar
import astar
import time

from visualization_msgs.msg import Marker
from lazy_astar import astar_path

class dataread:
    def __init__(self, Filepath):
        self.FILEPATH = Filepath
        self.data = self.generateData(self.FILEPATH)
        self.poses = np.array(self.data)
        rospy.Subscriber(rospy.get_param("~pose_topic", "/sim_car_pose/pose"),PoseStamped, self.get_current_pose)
        # rospy.Subscriber(rospy.get_param("~pose_topic", "/pf/viz/inferred_pose"),
#     PoseStamped, self.get_current_pose)

    def str2float(self, goal):
        newgoal = []
        print(goal)
        newgoal.append(float(goal[0]))
        newgoal.append(float(goal[1]))
        goal[2] = goal[2].replace('\n', '')
        newgoal.append(float(goal[2]))
        return newgoal

    def generateData(self, filepath):
        f = open(filepath, "r")
        data = []
        for x in f:
            goal = x.split(',')
            data.append(self.str2float(goal))
        return data

    def get_current_pose(self, msg):
        self.current_pose = msg.pose

class Pose:
    def __init__(self):
        self.current_pose = 0

        rospy.Subscriber(rospy.get_param("~pose_topic", "/sim_car_pose/pose"),PoseStamped, self.get_current_pose)
        while self.current_pose == 0:
            print("Recieve data")
            rospy.sleep(0.5)
            #self.current_pose = 0
        # rospy.Subscriber(rospy.get_param("~pose_topic", "/pf/viz/inferred_pose"),
#     PoseStamped, self.get_current_pose)

    def get_current_pose(self, msg):
        self.current_pose = msg.pose


if __name__ == '__main__':
    rospy.init_node("Goals")
    pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    pub_point = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    rospy.sleep(1)
    dataRead = dataread('goals.txt')
    goal = PoseStamped()

    Marker_id = 0

    for g in dataRead.poses:
        points = Marker()
        points.text = str(Marker_id)
        points.header.frame_id = "map"
        points.type = points.TEXT_VIEW_FACING
        points.id = Marker_id
        Marker_id += 1
        points.header.stamp = rospy.Time()
        points.action = points.ADD
        points.scale.x = 1.0/2
        points.scale.y = 1.0/2
        points.scale.z = 1.0/2
        points.color.b = 1.0
        points.color.a = 1.0
        points.pose.orientation.w = 1.0
        points.lifetime = rospy.Duration(0)

        points.pose.position.x = g[0]
        points.pose.position.y = g[1]

        # point = Point()
        # point.x = g[0]
        # point.y = g[1]

        # points.points.append(point)
        pub_point.publish(points)

    for g in dataRead.poses:
        # goal.pose.position.x = g[0]
        # goal.pose.position.y = g[1]
        # point = Point()
        # point.x = g[0]
        # point.y = g[1]
        # points.points.append(point)
        #print(goal)
        the_input = raw_input('Continue:>> ')
        index = int(the_input)
        pos = Pose()
        tan_y = goal.pose.position.y - pos.current_pose.position.y
        tan_x = goal.pose.position.x - pos.current_pose.position.x
        theta = np.arctan2(tan_y, tan_x)
        qua = util.angle_to_quaternion(theta)
        print(qua)
        while the_input != None:
            goal.pose.position.x = dataRead.poses[index][0]
            goal.pose.position.y = dataRead.poses[index][1]
            # goal.pose.orientation.x = qua.x
            # goal.pose.orientation.y = qua.y
            # goal.pose.orientation.z = qua.z
            # goal.pose.orientation.w = qua.w
            pub_goal.publish(goal)
            break

