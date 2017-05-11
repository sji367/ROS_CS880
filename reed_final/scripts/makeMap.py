#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue May  2 17:07:03 2017

@author: team3
"""
import rospy
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Position:
    """ Helper class that describes the positon of the turtlebot:
            x,y and theta 
    """
    x = 0.0
    y = 0.0
    theta = 0.0
    time = 0.0
    
    
def normalize_angle(angle):
    """REDUCE ANGLES TO -pi pi"""
    angle %= np.pi*2
    if angle > np.pi:
        angle -= np.pi*2
    return angle
    
class MakeMap():
    def vel_callback(self, vel_msg):
        """ """
        self.velocity.linear.x = vel_msg.linear.x
        self.velocity.linear.y = vel_msg.linear.y
        self.velocity.angular.z = vel_msg.angular.z
        
    def scan_callback(self, scan_msg):
        """ Callback for the /scan messages."""
        self.previous_scan.ranges = self.scan.ranges
        self.scan.ranges = scan_msg.ranges
        self.scan.angle_increment = scan_msg.angle_increment
        self.scan.angle_max = scan_msg.angle_max
        self.scan.angle_min = scan_msg.angle_min
    
    def odom_callback(self, odom_msg):
        """ callback to handle odometry messages"""        
        # convert those into x/y/theta positions
        self.cur_odom.x = odom_msg.pose.pose.position.x
        self.cur_odom.y = odom_msg.pose.pose.position.y
        self.cur_odom.theta = normalize_angle(self.rotation_distance(odom_msg.pose.pose.orientation.z,
                                                                     odom_msg.pose.pose.orientation.w))
        
    def xy2grid(self, x,y):
        """ Converts the local X,Y coordinates to the grid coordinate system."""
        gridX = int(round((x-self.origin.x)/self.grid_size))
        gridY = int(round((y-self.origin.y)/self.grid_size))
        return gridX,gridY
        
    def grid2xy(self, gridX, gridY):
        """ Converts the grid coordinates to the local X,Y. """
        x = gridX*self.grid_size+self.origin.x
        y = gridY*self.grid_size+self.origin.y
        return x,y
        
    def xy2mapIndex(self, x,y):
        """ Converts the x,y grid coordinates into a row-major index for the 
            map. """
        if x>self.ogrid_sizeX or y>self.ogrid_sizeY:
            print 'MAP IS TOO SMALL!!!'
            return self.ogrid_sizeX * self.ogrid_sizeY -1
        else:
            return int(y*self.ogrid_sizeY + x)
        
        
    def mapIndex2xy(self, index):
        """ Converts the row-major index for the map into x,y coordinates."""
        x = np.mod(index, self.ogrid_sizeY)
        y = (index-x)/self.ogrid_sizeY
        return x,y
         
    def setOrigin(self):
        """ Sets the origin. """
        origin = Position()
        origin.x = self.ogrid_sizeX/2*self.grid_size
        origin.y = self.ogrid_sizeY/2*self.grid_size
        origin.theta = 0        
        
        self.current_map.info.height = self.ogrid_sizeX
        self.current_map.info.width = self.ogrid_sizeY
        self.current_map.info.resolution = self.grid_size
        
        return origin
        
        
    def setOccupancy(self, index, prior, Free):
        """ This function defines the occupancy of the cells."""
        # If there is no previous data, then use a prior of 0.5 (Beta distribution)
        if prior == -1:
            prior = 0.5
        # convert to probability (currently in range from 0 to 100)
        else:
            prior *= .01
            if prior < 0.01:
                prior = 0.01
        
        # Calculate eta
        eta = 1/(self.p_measurement_given_occupied*prior + self.p_measurement_given_notOccupied*(1-prior))

        if Free:
            prob_occupancy = eta*self.p_measurement_given_notOccupied*(prior)
        else:
            prob_occupancy = eta*self.p_measurement_given_occupied*prior
            
        self.current_map.data[index] = int(prob_occupancy*100)
    
    def updateNeighbors(self, scanX, scanY):
        """ Update the free occupancy grid cells between the robot and the   
            obstacle found using the laser scan.
        """
        camX, camY = self.xy2grid(self.camera_pos.x, self.camera_pos.y)
        prev_hit = [[scanX, scanY]]
        dx = scanX - camX
        dy = scanY - camY
        
        # Need to check that the path does not pass an object
        JumpCells=2*max(abs(dx),abs(dy))-1
        for K in range(1,JumpCells):
            # intermediate positions
            XPOS=int(round(K*1.0*dx/JumpCells))+camX
            YPOS=int(round(K*1.0*dy/JumpCells))+camY
            if [XPOS, YPOS] not in prev_hit or [XPOS, YPOS] not in self.scan_endpoints:
                prev_hit.append([XPOS, YPOS])
                # update the map
                row_major_index = self.xy2mapIndex(XPOS, YPOS)
                self.setOccupancy(row_major_index, self.current_map.data[row_major_index], True)
    

    def updateMap(self):
        """ This function updates the occupancy grid cells with the current 
            scan. """    
        self.getCameraPositon()
        prev=-1
        cur_ind = -1
        
        # The Current position of the robot should be free
        camGridX,camGridY=self.xy2grid(self.camera_pos.x, self.camera_pos.y)
        cam_index= self.xy2mapIndex(camGridX,camGridY)
        self.setOccupancy(cam_index, self.current_map.data[cam_index], True)
        
        laser_data=self.scan.ranges
        start_angle = normalize_angle(self.scan.angle_min+self.camera_pos.theta)
        
        self.scan_endpoints = []
        for i in range(len(laser_data)):
            if not np.isnan(laser_data[i]) and laser_data[i] < self.max_range and laser_data[i] > self.min_range:
                cur_ind =i
                if prev !=-1:
                    # Only update the map when the difference in the two scans
                    #   is less than +/- 0.05 meters (approx. +/- 2").  
                    if abs(laser_data[prev]-laser_data[i])< 0.05:
                        # Update the current angle to reflect the angle of the laser 
                        #  scan.
                        cur_angle = normalize_angle(self.scan.angle_increment*i + start_angle)
                        
                        # Determine the location of the obstacle found by the laser scan
                        scanX = laser_data[i]*np.cos(cur_angle)+self.camera_pos.x
                        scanY = laser_data[i]*np.sin(cur_angle)+self.camera_pos.y
                        self.scan_endpoints = [[scanX,scanY]]
                        
                        scanX_grid, scanY_grid = self.xy2grid(scanX, scanY)
                        
                        # Update the map between the obstacle and the robot
                        self.updateNeighbors(scanX_grid, scanY_grid)
                            
                        # Update the map to include the obstacle
                        row_major_index = self.xy2mapIndex(scanX_grid, scanY_grid)
                        self.setOccupancy(row_major_index, self.current_map.data[row_major_index], False)
                prev = cur_ind
        self.publishNewMap()
        
    def publishNewMap(self):
        """ This function publishes an occupancy grid to the ROS topic 
            /frontier_map."""
        self.current_map.header.frame_id = 'map'
        
        self.current_map.info.origin.position.x = -self.origin.x
        self.current_map.info.origin.position.y = -self.origin.y
        self.current_map.info.origin.orientation.w = 1
        
        self.pub_map.publish(self.current_map)


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
        
    def get_current_position(self):
        """ Callback to get the current position"""
        self.prev_pos = self.setPrevPosition(self.position)
        
        # get the current position
        trans, rot = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
        self.position.time = rospy.get_rostime().to_sec()
        self.position.theta= normalize_angle(self.rotation_distance(rot[2],rot[3]))
        
        self.position.x= trans[0]
        self.position.y = trans[1]
            
        
    def getCameraPositon(self):
        """ Store the Camera's Position"""
        self.prev_camera_pos = self.setPrevPosition(self.camera_pos)
        
        # Set Current position
        trans, rot = self.listener.lookupTransform('map', 'camera_depth_frame', rospy.Time(0))
        self.camera_pos.time = rospy.get_rostime().to_sec()
        self.camera_pos.theta= normalize_angle(self.rotation_distance(rot[2],rot[3]))
        
        self.camera_pos.x= trans[0]
        self.camera_pos.y = trans[1]
        
            
    def setPrevPosition(self, position):
        """ Stores the position as the previous position """
        prev_pos = Position()
        prev_pos.x = position.x
        prev_pos.y = position.y
        prev_pos.theta = position.theta
        prev_pos.time = position.time
        
        return prev_pos
        
    def __init__(self):
        """ Initialize """
        rospy.init_node('Make_Map')
        
        self.listener = tf.TransformListener()        
        
        # Get the parameters for the grid
        self.ogrid_sizeX = rospy.get_param('x_size', 500)
        self.ogrid_sizeY = rospy.get_param('y_size', 500)
        self.grid_size = rospy.get_param('grid_size', 0.05) # in meters/cell (5cm)
        
        # Sensor Meta data
        self.min_range = rospy.get_param('max_range',0.5)
        self.max_range = rospy.get_param('max_range',6.0)
        
        # reliability
        self.p_measurement_given_occupied = rospy.get_param('p_z|occ',0.9)
        self.p_measurement_given_notOccupied = rospy.get_param('p_z|notOcc',0.3)
        
        # Initialize some varables to store the objects to be published/subscribed
        self.position = Position()
        self.prev_pos = Position()
        self.camera_pos= Position()
        self.prev_camera_pos = Position()
        
        # map
        self.velocity = Twist()
        
        self.current_map = OccupancyGrid()
        self.meta = MapMetaData()
        self.scan = LaserScan()
        self.previous_scan = LaserScan()
        
        # Rotation Booleans
        self.start_spin = True
        self.done_spinning = True
        self.new_cmd = False
        
        # Initialize the map as unknown (-1)
        self.current_map.data = [-1]*self.ogrid_sizeX*self.ogrid_sizeY # row-major order
        self.scan_endpoints = []
        
        self.origin = self.setOrigin()
        
        self.r = rospy.Rate(50)
        
        # publishers for OccupancyGrid
        self.pub_map = rospy.Publisher('/frontier_map', OccupancyGrid, queue_size=100)
        
        # subscribers for odometry and position (/move_base/feedback) messages
        self.cmd_vel = rospy.Subscriber('/cmd_vel_mux/input/navi', Twist, self.vel_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
#        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.costmap_callback)
        
        self.listener.waitForTransform('/map', '/camera_depth_frame', rospy.Time(0), rospy.Duration(1))
        
        # Get First position and then set it to previous
        self.getCameraPositon()
        self.previous_scan.ranges = self.scan.ranges
        
        while (not rospy.is_shutdown()):
            self.updateMap()
        
        return
if __name__ == "__main__":
    m = MakeMap()