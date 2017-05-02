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

class Position:
    """ Helper class that describes the positon of the turtlebot:
            x,y and theta 
    """
    x = 0.0
    y = 0.0
    theta = 0.0
    
class RobotState():
    spin = 0
    spin_0_180 = 1
    spin_180_360 = 2
    move_base = 3
    
def normalize_angle(angle):
    """REDUCE ANGLES TO -pi pi"""
    angle %= np.pi*2
    if angle > np.pi:
        angle -= np.pi*2
    return angle
    
class MakeMap():
    def scan_callback(self, scan_msg):
        """ Callback for the /scan messages."""
        self.scan.ranges = scan_msg.ranges
        self.scan.angle_increment = scan_msg.angle_increment
        self.scan.angle_min = scan_msg.angle_min
    
    def costmap_callback(self, map_msg):
        """ Callback to handle Map messages. """        
        self.meta.resolution= map_msg.info.resolution
        self.meta.height = map_msg.info.height
        self.meta.width = map_msg.info.width
        self.meta.origin.position.x = map_msg.info.origin.position.x
        self.meta.origin.position.y = map_msg.info.origin.position.y
        
    def xy2grid(self, x,y):
        """ Converts the local X,Y coordinates to the grid coordinate system."""
        gridX = int(round(x/self.grid_size))+self.origin.x
        gridY = int(round(y/self.grid_size))+self.origin.y
        return gridX,gridY
        
    def grid2xy(self, gridX, gridY):
        """ Converts the grid coordinates to the local X,Y. """
        x = (gridX-self.origin.x)*self.grid_size
        y = (gridY-self.origin.y)*self.grid_size
        return x,y
        
    def xy2mapIndex(self, x,y):
        """ Converts the x,y grid coordinates into a row-major index for the 
            map. """
        if x>self.ogrid_sizeX or y>self.ogrid_sizeY:
            return self.ogrid_sizeX * self.ogrid_sizeY -1
        else:
            return int(y*self.ogrid_sizeY + x)
        
        
    def mapIndex2xy(self, index):
        """ Converts the row-major index for the map into x,y coordinates."""
        x = np.mod(index, self.ogrid_sizeY)
        y = (index-x)/self.ogrid_sizeY
        return x,y
         
    def setOrigin(self):
        origin = Position()
        origin.x = self.ogrid_sizeX/2
        origin.y = self.ogrid_sizeY/2
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
        
        # Calculate eta
        eta = 1/(self.p_measurement_given_occupied*prior + self.p_measurement_given_notOccupied*(1-prior))

        if Free:
            prob_occupancy = eta*self.p_measurement_given_notOccupied*(prior)
        else:
            prob_occupancy = eta*self.p_measurement_given_occupied*prior
            
        self.current_map.data[index] = int(prob_occupancy*100)
    
    def updateNeighbors(self, scanX, scanY, camera_pos):
        """ Update the free occupancy grid cells between the robot and the   
            obstacle found using the laser scan.
        """
        camX, camY = self.xy2grid(camera_pos.x, camera_pos.y)
        robotX, robotY = self.xy2grid(self.position.x, self.position.y)
        
        print camera_pos.x, camera_pos.y, camera_pos.theta, camX, camY
        print self.position.x, self.position.y, self.position.theta, robotX, robotY
        print 
        dx = scanX - camX
        dy = scanY - camY
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
                row_major_index = self.xy2mapIndex(camX+XPOS, camY+YPOS)
                self.setOccupancy(row_major_index, self.current_map.data[row_major_index], True)
    
    def updateMap(self):
        """ This function updates the occupancy grid cells from the """
        camera_pos = Position()     
        camera_pos = self.getCameraPositon()
        
        # The Current position of the robot should be free
        camGridX,camGridY=self.xy2grid(camera_pos.x, camera_pos.y)
        cam_index= self.xy2mapIndex(camGridX,camGridY)
        self.setOccupancy(cam_index, self.current_map.data[cam_index], True)
        
        print camera_pos.x, camera_pos.y, camera_pos.theta, self.xy2grid(camera_pos.x, camera_pos.y)
        print self.position.x, self.position.y, self.position.theta, self.xy2grid(self.position.x, self.position.y)
        print 
        
        laser_data=self.scan.ranges
        cur_angle = normalize_angle(self.scan.angle_min+camera_pos.theta)
#        print 'update',len(laser_data)
        for i in range(len(laser_data)):
            if not np.isnan(laser_data[i]):# and laser_data[i] < self.max_range and laser_data[i] > self.min_range:  
                # Update the current angle to reflect the angle of the laser 
                #  scan.
                cur_angle = normalize_angle(self.scan.angle_increment + cur_angle)
                print cur_angle
                
                # Determine the location of the obstacle found by the laser scan
                scanX = laser_data[i]*np.cos(cur_angle)+camera_pos.x
                scanY = laser_data[i]*np.sin(cur_angle)+camera_pos.y
                            
                scanX_grid, scanY_grid = self.xy2grid(scanX, scanY)
                
                # Update the map between the obstacle and the robot
#                self.updateNeighbors(scanX_grid, scanY_grid, camera_pos)
                    
                # Update the map to include the obstacle
                row_major_index = self.xy2mapIndex(scanX_grid, scanY_grid)
#                print row_major_index
                self.setOccupancy(row_major_index, self.current_map.data[row_major_index], False)           
        self.publishNewMap()
        
    def publishNewMap(self):
        """ This function publishes an occupancy grid to the ROS topic 
            /frontier_map."""
        self.current_map.header.frame_id = 'map'
#        originX = self.origin.x*self.grid_size
#        originY = self.origin.y*self.grid_size
#        cur_pos= Position()
##        cur_pos= self.get_current_position()
##        print cur_pos.x, cur_pos.y
##        print cur_pos.x-originX,  cur_pos.y-originY
        dif_in_middleX = self.origin.x*self.grid_size#-self.meta.width*self.meta.resolution
        dif_in_middleY = self.origin.y*self.grid_size#-self.meta.height*self.meta.resolution
#        print dif_in_middleX, dif_in_middleY
        self.current_map.info.origin.position.x = -dif_in_middleX
        self.current_map.info.origin.position.y = -dif_in_middleY
        self.current_map.info.origin.orientation.w = 1
#        print self.meta.origin.position.x, self.meta.origin.position.y
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
        trans, rot = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
        self.position.x= trans[0]
        self.position.y = trans[1]
        self.position.theta= normalize_angle(self.rotation_distance(rot[2],rot[3]))
        
    def getCameraPositon(self):
        position = Position()
        trans, rot = self.listener.lookupTransform('map', 'camera_depth_frame', rospy.Time(0))
        position.x= trans[0]
        position.y = trans[1]
        position.theta= normalize_angle(self.rotation_distance(rot[2],rot[3]))
        
        return position
    
    def __init__(self):
        """ Initialize """
        rospy.init_node('Make_Map')
        
        self.listener = tf.TransformListener()        
        
        # Get the parameters for the grid
        self.ogrid_sizeX = rospy.get_param('x_size', 250)
        self.ogrid_sizeY = rospy.get_param('y_size', 250)
        self.grid_size = rospy.get_param('grid_size', 0.15) # in meters/cell (25cm)
        
        # Sensor Meta data
        self.min_range = rospy.get_param('max_range',0.4)
        self.max_range = rospy.get_param('max_range',6.0)
        
        # reliability
        self.p_measurement_given_occupied = rospy.get_param('p_z|occ',0.9)
        self.p_measurement_given_notOccupied = rospy.get_param('p_z|notOcc',0.3)
        
        # Initialize some varables to store the objects to be published/subscribed
        self.position = Position()
        self.cur_odom = Position()
        self.prev_odom = Position()
        
        self.current_map = OccupancyGrid()
        self.meta = MapMetaData()
        self.scan = LaserScan()
        
        # Rotation Booleans
        self.start_spin = True
        self.done_spinning = True
        self.new_cmd = False
        
        # Initialize the map as unknown (-1)
        self.current_map.data = [-1]*self.ogrid_sizeX*self.ogrid_sizeY # row-major order
        
        self.origin = self.setOrigin()
        
        self.r = rospy.Rate(50)
        
        # publishers for OccupancyGrid
        self.pub_map = rospy.Publisher('/frontier_map', OccupancyGrid, queue_size=100)
        
        # subscribers for odometry and position (/move_base/feedback) messages
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.costmap_callback)
        
        self.listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(1))
        
        while (not rospy.is_shutdown()):
            self.get_current_position()
            self.updateMap()
            return
        
        return
if __name__ == "__main__":
    m = MakeMap()