#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 29 15:04:02 2017

@author: Sam Reed
"""

import rospy
from nav_msgs.msg import OccupancyGrid,MapMetaData
from geometry_msgs.msg import PoseStamped

class Frontier_Based_Exploration():
    def costmap_callback(self, map_msg):
        """ Callback to handle Map messages. """
        print "Got new MSG!"
        self.current_map.data = map_msg.data 
        
        self.meta.resolution= map_msg.info.resolution
        self.meta.height = map_msg.info.height
        self.meta.width = map_msg.info.width
        self.meta.origin = map_msg.info.origin
        pass
    
    def run(self):
        """ Runs the frontier based exploration. """
        # IN THIS LOOP ROS SENDS AND RECEIVES  
        while not rospy.is_shutdown():
            # When your trigger is activated, execute trajectory
#            self.newPose(WPT.upper())
            self.cmd_pose.publish(self.new_pose)
            self.r.sleep()
            
    def calc_centroid(self, points):
        """ This function takes in a set of points and finds its centroid. """
        x_c = 0
        y_c = 0
        count = 0
        for index in len(points):
            x = points[0]
            y = points[1]
            x_c=x_c+ x
            y_c=y_c+ y
            count+=1
        x_c=x_c/count
        y_c=y_c/count
    
                
    def __init__(self):
        """ Initialize """
        rospy.init_node('lab5')
        #rospy.on_shutdown(self.shutdown())
        
        # loop rate
        self.r = rospy.Rate(50)
        
        self.current_map = OccupancyGrid()
        self.meta = MapMetaData()
        self.new_pose = PoseStamped()
        
        # publisher for twist messages
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.costmap_callback)
        self.cmd_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)
        
        self.run()
        
        return

l = Frontier_Based_Exploration()

