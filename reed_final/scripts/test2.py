#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 26 10:25:44 2017

@author: team3
"""
#
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
        self.grid_size = .05
        self.x, self.y =  self.xy2grid(0.00355412702023, 0.00771332298722)
        # loop rate
        self.r = rospy.Rate(50)
        
        self.listener = tf.TransformListener() 
        
#        self.listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(1))
        self.listener.waitForTransform('map', 'camera_depth_frame', rospy.Time(0), rospy.Duration(15))
        while(not rospy.is_shutdown()):
#            (self.trans,self.rot) = self.listener.lookupTransform('base_link', '/map', rospy.Time(0))
#            print "map to Depth: ", self.trans, self.rot
            
            (self.trans,self.rot) = self.listener.lookupTransform('/map', 'base_link', rospy.Time(0))
            print "map to base_link: ", self.trans, self.rot
            self.r.sleep()
            
    def xy2grid(self, x,y):
        """ Converts the local X,Y coordinates to the grid coordinate system."""
        gridX = int(round((-x-self.grid_size/2.0)/self.grid_size))
        gridY = int(round((-y-self.grid_size/2.0)/self.grid_size))
        return gridX,gridY
        
        
t = Test()
print t.x, t.y
#p = [[1,2,2],[2,2,2],[2,2,2],[2,2,2]]
#print len(p)