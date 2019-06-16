#!/usr/bin/env python

import rospy
import threading
import signal

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class PosMarkers():
    def __init__(self):
        rospy.init_node('pos_markers', anonymous=True, disable_signals=True)
        
        self.pos = Marker()
        rospy.Subscriber('/sim_car_pose/pose', PoseStamped, self.callback)

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

        self.pos.pose.position = data.pose.position
        self.pos.pose.orientation = data.pose.orientation

if __name__ == '__main__':
    sub = PosMarkers()

    r = rospy.Rate(2)
    pub_pose = rospy.Publisher('/pose_markers/markers', Marker, queue_size=1)
    
    while not rospy.is_shutdown():

        pub_pose.publish(sub.pos)
        r.sleep()

    rospy.spin()