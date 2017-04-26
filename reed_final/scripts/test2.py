#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 26 10:25:44 2017

@author: team3
"""

import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

class Test():
    def __init__(self):
        rospy.init_node('TEST')
        
        # loop rate
        self.r = rospy.Rate(50)
        
        self.listener = tf.TransformListener() 
        
        self.listener.waitForTransform('/odom', 'base_link', rospy.Time(0), rospy.Duration(1))
        while(not rospy.is_shutdown()):
            (self.trans,self.rot) = self.listener.lookupTransform('/odom', '/map', rospy.Time(0))
            print "odom to map: ", self.trans, self.rot
            
            (self.trans,self.rot) = self.listener.lookupTransform('/map', 'base_link', rospy.Time(0))
            print "map to odom: ", self.trans, self.rot
            self.r.sleep()
        return
        
t = Test()
#p = [[1,2,2],[2,2,2],[2,2,2],[2,2,2]]
#print len(p)