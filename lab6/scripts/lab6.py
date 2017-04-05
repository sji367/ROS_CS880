#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  5 14:50:16 2017

@author: team3
"""

import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from ar_track_alv_msgs import AlvarMarker


    
class Lab6Solution():
        
    def feedback_callback(self, feedback_msg):
        """ Callback to handle feedback messages so that we know the current 
            position of the turtlebot."""        
        # Store the position and orientation
        self.current_pos.pose.position.x = feedback_msg.pose.position.x
        self.current_pos.pose.position.y = feedback_msg.pose.position.y
        self.current_pos.pose.orientation.z = feedback_msg.pose.orientation.z
        self.current_pos.pose.orientation.w = feedback_msg.pose.orientation.w    
    
    def AR_marker_callback(self, feedback_msg):
        """ """
        if len(feedback_msg)>0:
            self.marker.id = feedback_msg.id
            self.marke.pose.pose.position.z = feedback_msg.pose.pose.position.z
            self.marker.pose.pose.position.x = feedback_msg.pose.pose.position.x
            self.marker.pose.pose.orientation.z,self.marker.pose.pose.orientation.w = self.rotate(self.feedback_msg.pose.pose.orientation)
            
            if self.first_tag:
                if self.marker.id == 0:
                    self.origin.pose.pose.position.x= 1.707 - self.marker.pose.pose.position.x
                    self.origin.pose.pose.position.y= 6.395 - self.marker.pose.pose.position.z
                    self.origin.pose.pose.position.z = 0.631
                    self.origin.pose.pose.position.w = 0.775
                    
                elif self.marker.id == 1:
                    self.origin.pose.pose.position.x= -0.202 + self.marker.pose.pose.position.z
                    self.origin.pose.pose.position.y= 2.116 - self.marker.pose.pose.position.x
                    self.origin.pose.pose.position.z = 0.993
                    self.origin.pose.pose.position.w = 0.117
                    
                elif self.marker.id == 2:
                    self.origin.pose.pose.position.x= -0.269 + self.marker.pose.pose.position.x
                    self.origin.pose.pose.position.y= -1.602 + self.marker.pose.pose.position.z
                    self.origin.pose.pose.position.z = -0.824
                    self.origin.pose.pose.position.w = 0.566
                    
                elif self.marker.id == 3:
                    self.origin.pose.pose.position.x= 2.074 - self.marker.pose.pose.position.z
                    self.origin.pose.pose.position.y= 3.542 + self.marker.pose.pose.position.x
                    self.origin.pose.pose.position.z = -0.034
                    self.origin.pose.pose.position.w = 0.999
                    
                self.first_tag=False
#                self.origin.theta=self.marker.theta
                
    def newPose(self, WPT='A'):
        """Takes in the new waypoint and sets the new goal position"""
        new = PoseStamped()
        new.header.seq=1
        new.header.stamp.secs = rospy.get_rostime().to_sec()
        #new.header.stamp.nsecs = rospy.get_rostime().to_nsec()
        new.header.frame_id= 'map'
        
        new.pose.position.z = 0.0
        new.pose.orientation.x = 0.0
        new.pose.orientation.y = 0.0
        # Positions in the map
        if WPT == 'A': # By the window
            new.pose.position.x = -5.95
            new.pose.position.y = 4.15
            new.pose.orientation.z = -0.686
            new.pose.orientation.w = 0.727
            
        elif WPT == 'B': # In the middle of the hallway by the stairs
            new.pose.position.x = -10.53
            new.pose.position.y = -5.52
            new.pose.orientation.z = -0.999
            new.pose.orientation.w = 0.005
            
        elif WPT == 'C': # By the bathroom
            new.pose.position.x = -4.16
            new.pose.position.y = -5.51
            new.pose.orientation.z = 0.005
            new.pose.orientation.w = 0.999
            
        elif WPT == 'D': # In the hallway after the bathroom
            new.pose.position.x = -5.23
            new.pose.position.y = -7.59
            new.pose.orientation.z = -0.738
            new.pose.orientation.w = 0.674
            
        elif WPT == 'S': # Stop at current position
            new.pose.position.x = self.hold_pos.pose.position.x
            new.pose.position.y = self.hold_pos.pose.position.y
            new.pose.orientation.z = self.hold_pos.pose.orientation.z
            new.pose.orientation.w = self.hold_pos.pose.orientation.w
            
        # Write a new Pose message
        self.Pose = new
        
    def new_rotation(self, orientation):
        euler = tf.transformations(orientation.x, orientation.y, orientation.z, orientation.w)
        yaw = euler[1]
        quaternion = tf.transformations.quaternion_from_euler(0,0,yaw)
        return quaternion[2], quaternion[3]
        
    def __init__(self):
        """ STUFF YOU MAY WANT TO USE"""
        rospy.init_node('lab6')
        #rospy.on_shutdown(self.shutdown())
        self.make_map()
        # loop rate
        r = rospy.Rate(50)
        self.first_tag =True
        self.origin = PoseStamped()
        self.current_pos = PoseStamped()
        self.Pose = PoseStamped()
        self.marker = AlvarMarker()
        
        # publisher for twist messages
        self.cmd_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", PoseStamped, self.feedback_callback)
        self.feedback_sub = rospy.Subscriber("/ar_pose_marker/markers", AlvarMarker, self.AR_marker_callback)        
        cntr =0
        # IN THIS LOOP ROS SENDS AND RECEIVES  
        while not rospy.is_shutdown():
            self.cmd_pose.publish(self.Pose)
            r.sleep()
            cntr +=1
            # Make sure that the goal is published
#            if cntr>50:
        return


l = Lab6Solution()
