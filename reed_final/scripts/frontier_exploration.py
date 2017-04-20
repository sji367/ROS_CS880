#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 29 15:04:02 2017

@author: Sam Reed
"""

import rospy
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

class Position:
    """ Helper class that describes the positon of the turtlebot:
            x,y and theta 
    """
    x = 0.0
    y = 0.0
    theta = 0.0
    
def normalize_angle(angle):
    """REDUCE ANGLES TO -pi pi"""
    angle %= np.pi*2
    if angle > np.pi:
        angle -= np.pi*2
    return angle


class Frontier_Based_Exploration():
    def rotation_distance(self, q1, q2):
        """ Calculate the difference in yaw between two quaternions. For the 
            turtlebot, the z and w are the only two quaternions.

            Inputs:
                q1 - self.odom.pose.pose.orientation.z
                q2 - self.odom.pose.pose.orientation.w
            
            Ouput:
                yaw - yaw of the turtlebot
        """
        # This should convert the quaternions to roll/pitch/yaw. We only have 
        #   quaternions z and w so the output is only yaw.
        rot = tf.transformations.euler_from_quaternion([0.0, 0.0, q1, q2])
        yaw = rot[2]
        return yaw
        
    def costmap_callback(self, map_msg):
        """ Callback to handle Map messages. """
        print "Got new MSG!"
        self.current_map.data = map_msg.data 
        
        self.meta.resolution= map_msg.info.resolution
        self.meta.height = map_msg.info.height
        self.meta.width = map_msg.info.width
        self.meta.origin = map_msg.info.origin
        pass
    
    
    def feedback_callback(self, feedback_msg):
        """ Callback to handle feedback messages so that we can stop the 
                turtlebot at its current position."""        
        # Store the position and orientation
        self.pos.pose.position.x = feedback_msg.pose.position.x
        self.pos.pose.position.y = feedback_msg.pose.position.y
        self.pos.pose.orientation.z = feedback_msg.pose.orientation.z
        self.pos.pose.orientation.w = feedback_msg.pose.orientation.w      
    
    def odom_callback(self, odom_msg):
        """ callback to handle odometry messages"""
        # We only care about the x,y position and the z and w orientation
        self.odom.pose.pose.position.x = odom_msg.pose.pose.position.x
        self.odom.pose.pose.position.y = odom_msg.pose.pose.position.y
        self.odom.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
        self.odom.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w
        
        # convert those into x/y/theta positions
        self.position.x = self.odom.pose.pose.position.x
        self.position.y = self.odom.pose.pose.position.y
        self.position.theta = normalize_angle(self.rotation_distance(self.odom.pose.pose.orientation.z,
                                                                     self.odom.pose.pose.orientation.w))
    
    def xy2grid(self, x,y):
        """ Converts the local X,Y coordinates to the grid coordinate system."""
        gridX = (x-self.origin.x-self.grid_size/2.0)/self.grid_size
        gridY = (y-self.origin.y-self.grid_size/2.0)/self.grid_size
        return gridX,gridY
        
    def grid2xy(self, gridX, gridY):
        """ Converts the grid coordinates to the local X,Y. """
        x = gridX*self.grid_size+self.origin.x+self.grid_size/2.0
        y = gridY*self.grid_size+self.origin.y+self.grid_size/2.0 
        return x,y
        
    def mapIndex(self, x,y):
        """ Converts the x,y coordinates into a row-major index for the map"""
        return y*self.ogrid_sizeY + x
        
    
    def updateNeighbors(self, scanX, scanY):
        """ Update the free occupancy grid cells between the robot and the   
            obstacle found using the laser scan.
        """
        dx = scanX - self.position.x
        dy = scanY - self.position.y
        # Need to check that the path does not pass an object
        JumpCells=2*max(abs(dx),abs(dy))-1
        for K in range(1,JumpCells):
            # intermediate positions
            YPOS=int(round(K*1.0*dy/JumpCells));
            XPOS=int(round(K*1.0*dx/JumpCells));
            # update the map
            row_major_index = self.mapIndex(self.position.x+XPOS, self.position.y+YPOS)
            self.map[row_major_index] = 0
    
    def updateMap(self):
        """ This function updates the occupancy grid cells from the """
        laser_data=self.scan.ranges
        cur_angle = -self.scan.angle_min
        for i in range(len(laser_data)):
            if not np.isnan(laser_data[i]):
                # Update the current angle to reflect the angle of the laser 
                #  scan.
                cur_angle += self.scan.angle_increment*i
                
                # Determine the location of the obstacle found by the laser scan
                scanX = laser_data[i]*np.cos(cur_angle)
                scanY = laser_data[i]*np.sin(cur_angle)
                
                scanX_grid, scanY_grid = self.xy2grid(scanX, scanY)
                
                # Update the map between the obstacle and the robot
                self.updateNeighbors(scanX_grid, scanY_grid)
                self.map[self.mapIndex(scanX_grid, scanY_grid)] = 1
        
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
    
    def run(self):
        """ Runs the frontier based exploration. """
        # IN THIS LOOP ROS SENDS AND RECEIVES  
        while not rospy.is_shutdown():
            self.cmd_pose.publish(self.new_pose)
            self.r.sleep()
                
    def __init__(self, init_sizeX = 100, init_sizeY = 100, grid_size=0.05):
        """ Initialize """
        rospy.init_node('Frontier_exploration')
        #rospy.on_shutdown(self.shutdown())
        self.ogrid_sizeX = init_sizeX
        self.ogrid_sizeY = init_sizeY
        
        self.origin = Position()
        self.position = Position()
        self.pos = PoseStamped()
        self.odom = Odometry()
        
        self.map = [-1]*init_sizeX*init_sizeY # row-major order
        self.grid_size = grid_size # in meters/cell
        
        # loop rate
        self.r = rospy.Rate(50)
        
        self.current_map = OccupancyGrid()
        self.meta = MapMetaData()
        self.scan = LaserScan()
        self.new_pose = PoseStamped()
        
        # publisher for twist messages
        #self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.costmap_callback)
        self.pub_map = rospy.Publisher('/map', OccupancyGrid, queue_size=100)
        self.cmd_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)
        # subscribe to odometry messages
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", PoseStamped, self.feedback_callback)
        
        self.run()
        
        return

l = Frontier_Based_Exploration()

