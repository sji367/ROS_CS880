#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
#from actionlib_msgs.msg import GoalID 

class Lab5Solution():
    def feedback_callback(self, feedback_msg):
        """ Callback to handle feedback messages so that we can stop the 
                turtlebot at its current position."""        
        # Store the position and orientation
        self.hold_pos.pose.position.x = feedback_msg.pose.position.x
        self.hold_pos.pose.position.y = feedback_msg.pose.position.y
        self.hold_pos.pose.orientation.z = feedback_msg.pose.orientation.z
        self.hold_pos.pose.orientation.w = feedback_msg.pose.orientation.w    
    
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
        
    def __init__(self, WPT):
        """ STUFF YOU MAY WANT TO USE"""
        rospy.init_node('lab5')
        #rospy.on_shutdown(self.shutdown())
        
        # loop rate
        r = rospy.Rate(50)
        
        self.hold_pos = PoseStamped()
        self.Pose = PoseStamped()
        
        # publisher for twist messages
        self.cmd_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", PoseStamped, self.feedback_callback)
        cntr =0
        # IN THIS LOOP ROS SENDS AND RECEIVES  
        while not rospy.is_shutdown():
            # When your trigger is activated, execute trajectory
            self.newPose(WPT.upper())
            self.cmd_pose.publish(self.Pose)
            r.sleep()
            cntr +=1
            # Make sure that the goal is published
            if cntr>50:
                WPT = raw_input('Pick a waypoint (A,B,C,D) or press S to stop in place or E to exit: ')
                if WPT == 'E':
                    break
        return

# Chose first waypoint
WPT = raw_input('Pick yor waypoint (A,B,C,D): ')
l = Lab5Solution(WPT)
