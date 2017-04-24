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
        
    def distanceFomula(self, x1,y1, x2,y2):
         return np.sqrt(np.square(x2 - x1) + np.square(y2-y1))
         
    def calcUtility(self, centroidX, centroidY, centroidLength):
        """ Calculate the utility of the frontier's centroid using a combo of
            the distance to the centroid and the length of the centroid.
        """
        dist_weight = 1
        length_weight = 1
        dist = self.distanceFomula(centroidX, centroidY, self.position.x, self.position.y)
        util = (length_weight*centroidLength)/(dist_weight*dist)
        return util
        
    def calcUtil_dist(self, centroidX, centroidY):
        """ Calculate the utility of the frontier's centroid using just the
            distance to the centroid. 
        """
        dist = self.distanceFomula(centroidX, centroidY, self.position.x, self.position.y)
        return dist
        
    def setOccupancy(self, index, prior, Free):
        """ """
        # If there is no previous data, then use a prior of 0.5 (Beta distribution)
        if prior == -1:
            prior = 0.5
        
        # Calculate eta
        eta = self.p_measurement_given_occupied*prior + self.p_measurement_given_notOccupied*(1-prior)

        if Free:
            prob_occupancy = eta*self.p_measurement_given_occupied*prior
#            p = self.p_measurement_given_occupied
        else:
            prob_occupancy = eta*self.p_measurement_given_notOccupied*(1-prior)
#            p = self.p_measurement_given_notOccupied
        self.map[index] = prob_occupancy#prior+np.log()
    
    def updateNeighbors(self, scanX, scanY):
        """ Update the free occupancy grid cells between the robot and the   
            obstacle found using the laser scan.
        """
        dx = scanX - self.position.x
        dy = scanY - self.position.y
        prevX = 0
        prevY = 0
        # Need to check that the path does not pass an object
        JumpCells=2*max(abs(dx),abs(dy))-1
        for K in range(1,JumpCells):
            # intermediate positions
            YPOS=int(round(K*1.0*dy/JumpCells))
            XPOS=int(round(K*1.0*dx/JumpCells))
            if (prevX != XPOS) or (prevY != YPOS):
                # update the map
                row_major_index = self.mapIndex(self.position.x+XPOS, self.position.y+YPOS)
                self.setOccupancy(row_major_index, self.map[row_major_index], True)
    
    def updateMap(self):
        """ This function updates the occupancy grid cells from the """
        laser_data=self.scan.ranges
        cur_angle = normalize_angle(self.scan.angle_min+self.position.theta)
        for i in range(len(laser_data)):
            if not np.isnan(laser_data[i]):
                # Update the current angle to reflect the angle of the laser 
                #  scan.
                cur_angle = normalize_angle(self.scan.angle_increment*i + cur_angle)
                
                # Determine the location of the obstacle found by the laser scan
                scanX = laser_data[i]*np.cos(cur_angle)
                scanY = laser_data[i]*np.sin(cur_angle)
                
                scanX_grid, scanY_grid = self.xy2grid(scanX, scanY)
                
                # Update the map between the obstacle and the robot
                self.updateNeighbors(scanX_grid, scanY_grid)
                
                # Update the map to include the obstacle
                row_major_index = self.mapIndex(scanX_grid, scanY_grid)
                self.setOccupancy(row_major_index, self.map[row_major_index], False)
                
    def findFrontier(self):
        """ This function cycles through each cell in the map and finds all 
            cells that are part of the frontier. The frontier is defined as 
            regions on the border of empty space and unknown space.
        """
        # Cycle through all points in the map. When a unknown
        pass
        
    def calc_centroid(self, points):
        """ This function takes in a set of points and finds its centroid.
            
            Input:
                points - 2D array where the each row contains the X,Y location 
                    of the frontier cell
        """
        if len(points) == 0:
            return
        num_Rows = len(points[0])
        x_c, y_c = np.sum(points, axis=0)
        x_c /= int(np.round(1.0*num_Rows))
        y_c /= int(np.round(1.0*num_Rows))
        return x_c, y_c, self.calcUtil_dist(x_c, y_c)
#        x_c = 0
#        y_c = 0
#        length = 0
#        count = 0
#        prev_x = 0
#        prev_y = 0
#        for index in range(len(points)):
#            x = points[index][0]
#            y = points[index][1]
#            # This length can be incorrect if the previous point in the array 
#            #  "points" is not the closest point to the current point
#            if index >0:
#                length += self.distanceFomula(prev_x,prev_y, x,y)
#            x_c += x
#            y_c += y
#            # Store the previous x and y locations of the frontier
#            prev_x = x
#            prev_y = y
#            # Increment the counter
#            count+=1
#        if count == 0:
#            return
#        elif count == 1:
#            length = 1
#        x_c=x_c/count
#        y_c=y_c/count
#        util = self.calcUtility(x_c,y_c, length)
#        return x_c, y_c, util
        
    def pickBestCentroid(self, frontiers):
        """ Takes in all frontiers (as a 3D array) and """
        centroidX = []
        centroidY = []
        utility = []
        for i in range(len(frontiers)):
            x_c, y_c, distance_c = self.calc_centroid(frontiers[i])
            centroidX.append(x_c)
            centroidY.append(y_c)
            utility.append(distance_c)
        # Determine which centroid is the closest 
        index = np.argmax(utility)
        return centroidX[index], centroidY[index]
        
        
    def run(self):
        """ Runs the frontier based exploration. """
        # IN THIS LOOP ROS SENDS AND RECEIVES  
        while not rospy.is_shutdown():
            # Transform the odometry message to the map reference frame
            #   trans (Translation) - [x,y,z]
            #   rot (Rotation) - quentieron [x,y,z,w]
            (self.trans,self.rot) = self.listener.lookupTransform('/odom', '/map', rospy.Time(0))
            if (cur_pos):
                pass
            #self.cmd_pose.publish(self.new_pose)
            if find_new_centroid:
                frontiers = self.findFrontiers()
                centroidX, centroidY = self.pickBestCentroid(frontiers)
                self.move2Centroid(centroidX, centroidY)
            else:
                while (len(self.new_scans)>0):
                    self.updateMap()
            self.r.sleep()
                
    def __init__(self):
        """ Initialize """
        rospy.init_node('Frontier_exploration')
        
        self.listener = tf.TransformListener()        
        
        # Get the parameters for the grid
        self.ogrid_sizeX = rospy.get_param('x_size', 200)
        self.ogrid_sizeY = rospy.get_param('y_size', 200)
        self.grid_size = rospy.get_param('grid_size', 0.05) # in meters/cell (5cm)
        
        # Sensor Meta data
        self.max_range = rospy.get_param('max_range',5)
        self.min_range = rospy.get_param('min_range',0.6)
        # reliability
        self.p_measurement_given_occupied = rospy.get_param('p_z|occ',0.9)
        self.p_measurement_given_notOccupied = rospy.get_param('p_z|notOcc',0.3)
        
        # Initialize some varables to store the objects to be published/subscribed
        self.origin = Position()
        self.position = Position()
        self.pos = PoseStamped()
        self.odom = Odometry()
        
        self.current_map = OccupancyGrid()
        self.meta = MapMetaData()
        self.scan = LaserScan()
        self.new_pose = PoseStamped()
        
        # Initialize the map as unknown (-1)
        self.map = [-1]*self.ogrid_sizeX*self.ogrid_sizeY # row-major order
        
        # loop rate
        self.r = rospy.Rate(50)
        
        # publishers for OccupancyGrid and move base messages
        self.pub_map = rospy.Publisher('/map', OccupancyGrid, queue_size=100)
        self.cmd_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)
        
        # subscribers for odometry and position (/move_base/feedback) messages
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", PoseStamped, self.feedback_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        #self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.costmap_callback)
        
        self.run()
        
        return

if  __name__=="__main__":
    l = Frontier_Based_Exploration()

