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
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Twist, Vector3
#from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatus, GoalStatusArray

class Position:
    """ Helper class that describes the positon of the turtlebot:
            x,y and theta 
    """
    x = 0.0
    y = 0.0
    theta = 0.0
    
class RobotState():
    spin = 0
    move_base = 2
    
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
        self.meta.resolution= map_msg.info.resolution
        self.meta.height = map_msg.info.height
        self.meta.width = map_msg.info.width
        self.meta.origin.position.x = map_msg.info.origin.position.x
        self.meta.origin.position.y = map_msg.info.origin.position.y
        
        self.gmapping_map.data = map_msg.data
        self.ogrid_sizeX = map_msg.info.width
        self.ogrid_sizeY = map_msg.info.height
        
        self.origin.x = map_msg.info.origin.position.x
        self.origin.y = map_msg.info.origin.position.y
    
#    def scan_callback(self, scan_msg):
#        """ Callback for the /scan messages."""
#        self.scan.ranges = scan_msg.ranges
#        self.scan.angle_increment = scan_msg.angle_increment
#        self.scan.angle_min = scan_msg.angle_min
#        
    def status_callback(self, status_msg):
        """ """
        if  len(status_msg.status_list) >0:
            for i in range(len(status_msg.status_list)):
                ID = int(status_msg.status_list[i].goal_id.id.split('-')[1])
                # Store the stautus of all WPTs
                if len(self.move_base_status) >= ID+1:
                    self.move_base_status[ID] = status_msg.status_list[i].status
                else:
                    self.move_base_status.append(status_msg.status_list[i].status)
                    
    def odom_callback(self, odom_msg):
        """ callback to handle odometry messages"""        
        # convert those into x/y/theta positions
        self.cur_odom.x = odom_msg.pose.pose.position.x
        self.cur_odom.y = odom_msg.pose.pose.position.y
        self.cur_odom.theta = normalize_angle(self.rotation_distance(odom_msg.pose.pose.orientation.z,
                                                                     odom_msg.pose.pose.orientation.w))
                                                                     
    
    def xy2grid(self, x,y):
        """ Converts the local X,Y coordinates to the grid coordinate system."""
#        gridX = int(round(x/self.grid_size))+self.origin.x
#        gridY = int(round(y/self.grid_size))+self.origin.y
        gridX = int(round((x-self.origin.x)/self.grid_size))
        gridY = int(round((y-self.origin.y)/self.grid_size))
        return gridX,gridY
        
    def grid2xy(self, gridX, gridY):
        """ Converts the grid coordinates to the local X,Y. """
#        x = (gridX-self.origin.x)*self.grid_size
#        y = (gridY-self.origin.y)*self.grid_size
        x = gridX*self.grid_size+self.origin.x
        y = gridY*self.grid_size+self.origin.y
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
        
    def distanceFomula(self, x1,y1, x2,y2):
         dist =  np.sqrt(np.square(x2 - x1) + np.square(y2-y1))
         return dist
         
    def calcCost(self, centroidX, centroidY, frontierLength):
        """ Calculate the cost of the frontier's centroid using a combo of
            the distance to the centroid and the length of the centroid.
        """
        dist_weight = 1
        length_weight = 1
        dist = self.distanceFomula(centroidX, centroidY, self.position.x, self.position.y)
        cost = (length_weight*frontierLength)/(dist_weight*dist)
        return cost
        
    def calcCost_dist(self, X, Y):
        """ Calculate the cost of the frontier's centroid using just the
            distance to the centroid. 
        """
        dist = self.distanceFomula(X, Y, self.position.x, self.position.y)
        return dist
        
#    def setOccupancy(self, index, prior, Free):
#        """ This function defines the occupancy of the cells."""
#        # If there is no previous data, then use a prior of 0.5 (Beta distribution)
#        if prior == -1:
#            prior = 0.5
#        # convert to probability (currently in range from 0 to 100)
#        else:
#            prior *= .01
#        
#        # Calculate eta
#        eta = 1/(self.p_measurement_given_occupied*prior + self.p_measurement_given_notOccupied*(1-prior))
#
#        if Free:
#            prob_occupancy = eta*self.p_measurement_given_notOccupied*(prior)
#        else:
#            prob_occupancy = eta*self.p_measurement_given_occupied*prior
##            print prob_occupancy, index
#            
#        self.current_map.data[index] = int(prob_occupancy*100)
#    
#    def updateNeighbors(self, scanX, scanY, camera_pos):
#        """ Update the free occupancy grid cells between the robot and the   
#            obstacle found using the laser scan.
#        """
#        camX, camY = self.xy2grid(camera_pos.x, camera_pos.y)
#        robotX, robotY = self.xy2grid(self.position.x, self.position.y)
#        
#        dx = scanX - camX
#        dy = scanY - camY
#        prevX = 0
#        prevY = 0
#        # Need to check that the path does not pass an object
#        JumpCells=2*max(abs(dx),abs(dy))-1
#        for K in range(1,JumpCells):
#            # intermediate positions
#            YPOS=int(round(K*1.0*dy/JumpCells))
#            XPOS=int(round(K*1.0*dx/JumpCells))
#            if (prevX != XPOS) or (prevY != YPOS):
#                # update the map
#                row_major_index = self.xy2mapIndex(camX+XPOS, camY+YPOS)
#                self.setOccupancy(row_major_index, self.current_map.data[row_major_index], True)
#    
#    def updateMap(self):
#        """ This function updates the occupancy grid cells from the """
#        camera_pos = Position()     
#        camera_pos = self.getCameraPositon()
#        
#        # The Current position of the robot should be free
#        camGridX,camGridY=self.xy2grid(camera_pos.x, camera_pos.y)
#        cam_index= self.xy2mapIndex(camGridX,camGridY)
#        self.setOccupancy(cam_index, self.current_map.data[cam_index], True)
#        
#        laser_data=self.scan.ranges
#        cur_angle = normalize_angle(self.scan.angle_min+camera_pos.theta)
##        print 'update',len(laser_data)
#        for i in range(len(laser_data)):
#            if not np.isnan(laser_data[i]) and laser_data[i] < self.max_range and laser_data[i] > self.min_range:            
#                # Update the current angle to reflect the angle of the laser 
#                #  scan.
#                cur_angle = normalize_angle(self.scan.angle_increment + cur_angle)
#                
#                # Determine the location of the obstacle found by the laser scan
#                scanX = laser_data[i]*np.cos(cur_angle)+camera_pos.x
#                scanY = laser_data[i]*np.sin(cur_angle)+camera_pos.y
#                            
#                scanX_grid, scanY_grid = self.xy2grid(scanX, scanY)
#                
#                # Update the map between the obstacle and the robot
#                self.updateNeighbors(scanX_grid, scanY_grid, camera_pos)
#                    
#                # Update the map to include the obstacle
#                row_major_index = self.xy2mapIndex(scanX_grid, scanY_grid)
##                print row_major_index
#                self.setOccupancy(row_major_index, self.current_map.data[row_major_index], False)           
#        self.publishNewMap()
#        
#    def publishNewMap(self):
#        """ This function publishes an occupancy grid to the ROS topic 
#            /frontier_map."""
#        self.current_map.header.frame_id = 'map'
##        originX = self.origin.x*self.grid_size
##        originY = self.origin.y*self.grid_size
##        cur_pos= Position()
###        cur_pos= self.get_current_position()
###        print cur_pos.x, cur_pos.y
###        print cur_pos.x-originX,  cur_pos.y-originY
#        dif_in_middleX = self.origin.x*self.grid_size-self.meta.width*self.meta.resolution
#        dif_in_middleY = self.origin.y*self.grid_size-self.meta.height*self.meta.resolution
##        print dif_in_middleX, dif_in_middleY
#        self.current_map.info.origin.position.x = -dif_in_middleX
#        self.current_map.info.origin.position.y = -dif_in_middleY
#        self.current_map.info.origin.orientation.w = 1
##        print self.meta.origin.position.x, self.meta.origin.position.y
#        self.pub_map.publish(self.current_map)
                
        
    def onFrontier(self, index):
        """ This function takes in an index of a cell in the map and determines
            if it is part of the frontier. The frontier is defined as regions
            on the border of empty space and unknown space.
        """
        connected = False
        top = False
        bottom = False
        
        if (index>self.ogrid_sizeX):
            top = True
            # Check the cell above to see if its connected to a known 
            #   cell
            if self.gmapping_map.data[index-self.ogrid_sizeX] ==-1:#<50 and self.gmapping_map.data[index-self.ogrid_sizeX]>=0:
                connected = True
            
        if (index<(self.ogrid_sizeX*self.ogrid_sizeY-self.ogrid_sizeX)): # check this math
            bottom =True
            # Check the cell below to see if its connected to a known 
            #   cell
            if self.gmapping_map.data[index+self.ogrid_sizeX] ==-1:#<50 and self.gmapping_map.data[index+self.ogrid_sizeX]>=0:
                connected = True
            
        if (np.mod(index,self.ogrid_sizeX) != 0):
            # Check the cell to the left to see if its connected to a  
            #   known cell
            if self.gmapping_map.data[index-1] ==-1:#<50 and self.gmapping_map.data[index-1]>=0:
                connected = True
            # Check top left
            if top and self.gmapping_map.data[index-self.ogrid_sizeX-1] ==-1:
                connected = True
            # Check bottom left
            if bottom and self.gmapping_map.data[index+self.ogrid_sizeX-1] ==-1:
                connected = True
        
        if (np.mod(index,self.ogrid_sizeX) != self.ogrid_sizeX-1):
            # Check the cell to the right to see if its connected to a 
            #   known cell
            if self.gmapping_map.data[index+1] ==-1:#<50 and self.gmapping_map.data[index+1]>=0:
                connected = True
            # Check top right
            if top and self.gmapping_map.data[index-self.ogrid_sizeX+1] ==-1:
                connected = True
            # Check bottom right
            if bottom and self.gmapping_map.data[index+self.ogrid_sizeX+1] ==-1:
                connected = True
        
#        print index, self.mapIndex2xy(index), connected
                
        return connected
        
    def Frontier(self):
        """ This funtion finds the frontier on the map and returns a 1D vector 
            containing the indices of the cells on the frontier."""
        frontier = []
        for i in range(len(self.gmapping_map.data)):
            # Store the frontiers as a list
            if self.gmapping_map.data[i]<30 and self.gmapping_map.data[i]>=0:
                if self.onFrontier(i):
                    frontier.append(i)
        return frontier
        
    def blogDetection(self, frontier):
        """ This function runs the connected components algorithm on the 
            inputed frontier, which is a 1D vector containing the indices of 
            the cells on the frontier.
        """
        labels = np.zeros_like(frontier, dtype=np.int8)
        full_labels = np.ones_like(self.gmapping_map.data, dtype=np.int8)*-1
        equiv = []
        cur_label = -1
        cntr = -1
        # Do first pass and label frontiers
        for i in frontier:
            cntr +=1
            top = False
            left = False
            topLeft = False
            topRight = False
            if (i>self.ogrid_sizeX):
                topIndex = i-self.ogrid_sizeX
                top = full_labels[topIndex] !=-1
                # Check top Right
                if ((np.mod(i,self.ogrid_sizeX) != self.ogrid_sizeX-1)):
                    topRightIndex = i-self.ogrid_sizeX+1
                    topRight = full_labels[topRightIndex] !=-1
                    
            if (np.mod(i,self.ogrid_sizeX) != 0):
                leftIndex = i-1
                left = full_labels[leftIndex] !=-1
                if (i>self.ogrid_sizeX):
                    topLeftIndex = i-self.ogrid_sizeX-1
                    topLeft = full_labels[topLeftIndex] !=-1
            
            
            # Make a new label if none of the previously explored Moore 
            #   Neighboors are part of the frontier
            if not top and not left and not topRight and not topLeft:
                cur_label +=1
                label = cur_label
                
            # Mark equivalences when one of the previously explored Moore 
            #   Neighboors are part of the frontier
            if left and top:
                if (full_labels[topIndex] != full_labels[leftIndex]):
                    newEquiv =[]
                    newEquiv.append(full_labels[topIndex])
                    newEquiv.append(full_labels[leftIndex])
                    if newEquiv not in equiv:
                        equiv.append(newEquiv)
            if left and topLeft:
                if (full_labels[topLeftIndex] != full_labels[leftIndex]):
                    newEquiv =[]
                    newEquiv.append(full_labels[topLeftIndex])
                    newEquiv.append(full_labels[leftIndex])
                    if newEquiv not in equiv:
                        equiv.append(newEquiv)
            if left and topRight:
                if (full_labels[topRightIndex] != full_labels[leftIndex]):
                    newEquiv =[]
                    newEquiv.append(full_labels[topRightIndex])
                    newEquiv.append(full_labels[leftIndex])
                    if newEquiv not in equiv:
                        equiv.append(newEquiv)
            if topLeft and top:
                if (full_labels[topIndex] != full_labels[topLeftIndex]):
                    newEquiv =[]
                    newEquiv.append(full_labels[topIndex])
                    newEquiv.append(full_labels[topLeftIndex])
                    if newEquiv not in equiv:
                        equiv.append(newEquiv)
            if topLeft and topRight:
                if (full_labels[topRightIndex] != full_labels[topLeftIndex]):
                    newEquiv =[]
                    newEquiv.append(full_labels[topRightIndex])
                    newEquiv.append(full_labels[topLeftIndex])
                    if newEquiv not in equiv:
                        equiv.append(newEquiv)
            if top and topRight:
                if (full_labels[topIndex] != full_labels[topRightIndex]):
                    newEquiv =[]
                    newEquiv.append(full_labels[topIndex])
                    newEquiv.append(full_labels[topRightIndex])
                    if newEquiv not in equiv:
                        equiv.append(newEquiv)
            
            # Copy the label from the nearby pixel            
            if top:
                label = full_labels[topIndex]
            elif left:
                label = full_labels[leftIndex]
            elif topLeft:
                label = full_labels[topLeftIndex]
            elif topRight:
                label = full_labels[topRightIndex]
                
                
            full_labels[i] = label    
            labels[cntr] = label
            
        return labels, equiv
    
    def getFrontier(self):
        """ This function defines and labels the frontier. If the froniter is 
            smaller than a meter it is removed.""" 
        unlabeledFrontier = self.Frontier()
        labels, equiv = self.blogDetection(unlabeledFrontier)        
        
        # Initialize the frontier list
        frontier = []
        for n in range(max(labels)+1):
            frontier.append([0])
            
        # Second pass to remove equivilencies and store the frontiers    
        num_equiv = len(equiv) 
        for i in range(len(labels)):
            # Remove the equivalencies
            for ii in range(num_equiv):
                if labels[i] == equiv[ii][0]:
                    labels[i] = equiv[ii][1]
                    
            # Next store the index of the map into the correct row of the
            #   frontier 
            xy = []
            if frontier[labels[i]] == [0]:
                 x,y = self.mapIndex2xy(unlabeledFrontier[i])
                 xy.append(x)
                 xy.append(y)
                 frontier[labels[i]].append(xy)
                 frontier[labels[i]].pop(0)
            else:
                 x,y = self.mapIndex2xy(unlabeledFrontier[i])
                 xy.append(x)
                 xy.append(y)
                 frontier[labels[i]].append(xy)
        
        # Remove all frontiers smaller than 1 meter
        removed = 0
        for i in range(len(frontier)):
            index = i-removed
            if len(frontier[index])<int(1/self.grid_size):
                frontier.pop(index)
                removed+= 1
        
        return frontier 
        
    def calc_centroid(self, points):
        """ This function takes in a set of points and finds its centroid.
            
            Input:
                points - 2D array where the each row contains the X,Y location 
                    of the frontier cell
        """
        if len(points) == 0:
            return
        num_Rows = len(points)
        x_c, y_c = np.sum(points, axis=0)
        x_c /= int(np.round(1.0*num_Rows))
        y_c /= int(np.round(1.0*num_Rows))
        
        x,y = self.grid2xy(x_c, y_c)
        return x_c, y_c, self.calcCost_dist(x,y)
    
    def pickBestCentroid(self, frontiers):
        """ Takes in all frontiers (as a 3D array) and choses the best frontier"""
        self.centroidX = []
        self.centroidY = []
        self.centroidIndex = []
        self.cost = []
        
        centroid_index = 0
        for i in range(len(frontiers)):
            x_c, y_c, cost_c = self.calc_centroid(frontiers[i])
            # Store centroid if it is known and 
            index = self.xy2mapIndex(x_c, y_c)
            x_c, y_c = self.grid2xy(x_c, y_c)
            if self.gmapping_map.data[index]< 50:# and self.gmapping_map.data[index]>=0:
                if [x_c, y_c] not in self.unreachable_frontiers or [x_c, y_c] not in self.reached_centroids:
                    self.makeMarker(x_c, y_c, centroid_index)
                    self.centroidX.append(x_c)
                    self.centroidY.append(y_c)
                    self.cost.append(cost_c)
                    self.centroidIndex.append(centroid_index)
                    centroid_index += 1
                
        return self.bestCentroid()
            
    def updateBestCentoid(self):
        """ """
        for i in range(len(self.centroidX)):
            # Update the cost
            self.cost[i] = (self.calcCost_dist(self.centroidX[i], self.centroidY[i]))
            
        return self.bestCentroid()
        
    def bestCentroid(self):
        """ This function takes the precalculated x/y and cost values of the 
            centroid and picks the returns the index to the cell that has the minimum cost"""
            
        print 'picking between {} centroids'.format(len(self.centroidX))
        if len(self.centroidX)>0:
            # Determine which centroid is the closest 
            index = np.argmin(self.cost)
            marker_index = self.centroidIndex[index]
            
            # Mark the centroid to navigate to as green
            self.markerArray.markers[marker_index].color.r = 0.0
            self.markerArray.markers[marker_index].color.g = 1.0
            self.markerArray.markers[marker_index].color.b = 0.0
            
            # Publish the markerArray 
            self.marker_pub.publish(self.markerArray)
            print '\tlen: ', len(self.markerArray.markers)
                
            print 'Navigating to', self.centroidX[index], self.centroidY[index], index
            
            return index
        else:
            print '\tNo Valid centroids'
            return -1
        
        
    def makeMarker(self, centroidX, centroidY, ID, action = Marker.ADD):
        """ Creates a marker on RVIZ for the centroid of the frontier"""
        
        RVIZmarker = Marker()
        
        RVIZmarker.header.frame_id = '/map'
        RVIZmarker.header.stamp = rospy.Time(0)
        
        RVIZmarker.ns = 'Frontier Contour'
        RVIZmarker.id = ID
        RVIZmarker.type = Marker.CUBE
        RVIZmarker.action = action
        
        # Position with respect to the frame
        RVIZmarker.pose.position.x = centroidX
        RVIZmarker.pose.position.y = centroidY
        RVIZmarker.pose.position.z = 0.0
        RVIZmarker.pose.orientation.x = 0.0
        RVIZmarker.pose.orientation.y = 0.0
        RVIZmarker.pose.orientation.z = 0.0
        RVIZmarker.pose.orientation.w = 1.0
        
        # Define the scale (meter scale)
        RVIZmarker.scale.x = 0.3
        RVIZmarker.scale.y = 0.3
        RVIZmarker.scale.z = 0.3
        
        # Set the color 
        RVIZmarker.color.a = 1.0
        RVIZmarker.color.r = 0.0
        RVIZmarker.color.g = 0.0
        RVIZmarker.color.b = 1.0
        
        #Store the marker
        if len(self.markerArray.markers) <= ID:
            self.markerArray.markers.append(RVIZmarker)
        else:
            self.markerArray.markers[ID] = RVIZmarker
            
    def removeAllMarkers(self):
        """This function removes all markers from rviz"""
        for i in range(len(self.markerArray.markers)):
            self.markerArray.markers[i].action = Marker.DELETE
            
        self.marker_pub.publish(self.markerArray)
        
        self.markerArray = MarkerArray()
        
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
        
    def setOrigin(self):
        origin = Position()
        origin.x = -self.ogrid_sizeX/2/self.grid_size
        origin.y = -self.ogrid_sizeY/2/self.grid_size
        origin.theta = 0        
                
        return origin
    
    def newPose(self, x, y, rot_z=0.1,rot_w=0.1):
        """Takes in the new waypoint and sets the new goal position"""        
        new = PoseStamped()
        new.header.seq=1
        new.header.stamp.secs = rospy.get_rostime().to_sec()
        
        new.header.frame_id= 'map'
        
        # Positions in the map
        new.pose.position.x = x
        new.pose.position.y = y
        new.pose.position.z = 0.0
        
        new.pose.orientation.x = 0.0
        new.pose.orientation.y = 0.0
        new.pose.orientation.z = rot_z
        new.pose.orientation.w = rot_w
        
        # Publish the new position
        self.cmd_pose.publish(new)
    
    def angle_traveled(self):
        """ Calculates the angle traveled between the current and previous 
            angle.
        """
        diff = self.cur_odom.theta-self.prev_odom.theta
        # If there is a big jump, then it must have crossed the -180/180 
        #  boundary.
        if abs(diff)>np.pi:
            diff = abs(diff) - 2*np.pi
            np.sign(diff) == -1
            diff *= -1
        
        self.angle_trav += diff
        
    def spin360(self):
        """ Spins the robot 360 degrees w.r.t the odometer"""            
        self.angle_traveled() 
        
        if self.start_spin:
            self.removeAllMarkers()
            self.robotState = RobotState.spin
            self.angle_trav = 0
            self.start_spin = False
            self.done_spinning = False
            
        omega = .75 # Rad/sec
            
        if (self.angle_trav < np.pi*2):
            # Write a new twist message
            self.cmd_vel.publish(Twist(Vector3(0,0,0),Vector3(0,0,omega)))
        else:
            # turn off Twist
            self.cmd_vel.publish(Twist())
            self.done_spinning = True
        
        # Store previous odometery
        self.prev_odom.x =  self.cur_odom.x 
        self.prev_odom.y =  self.cur_odom.y
        self.prev_odom.theta =  self.cur_odom.theta 
        
    def move2frontier(self,X, Y):
        """ Navigate to the centroid of the chosen frontier"""
        self.newPose(X, Y)
        self.robotState = RobotState.move_base
                
    def run(self):
        """ Runs the frontier based exploration. """
        start = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec()-start) < 1:
            self.r.sleep()
        
        self.start_spin = True
        print 'Spinning'
        self.spin360()
        
        while not rospy.is_shutdown() and (rospy.Time.now().to_sec()-start) > 1:
            try:
                self.get_current_position()
                # If the robot is currently spinning, check if it completed a 
                #   rotation and if it has, find the frontiers.
                if self.robotState == RobotState.spin:
                    if self.done_spinning:
                        self.robotState = RobotState.move_base
                        self.current_failed = []               
                        self.WPT_ID +=1
                        frontiers = self.getFrontier()
                        if len(frontiers) == 0:
                            self.removeAllMarkers()
                            return
                        self.frontier_index = self.pickBestCentroid(frontiers)
                        if self.frontier_index == -1:
                            self.removeAllMarkers()
                            print '\tNo more frontiers'
                            return
                        self.move2frontier(self.centroidX[self.frontier_index], self.centroidY[self.frontier_index])
                    else:
                        if self.new_cmd:
                            self.start_spin=True
                            self.new_cmd = False
                        self.spin360()
                        
                # If the robot is trying to navigate to the centroid of a 
                #   frontier, check if the robot is in an end state
                if self.robotState == RobotState.move_base:
                    # If it could not reach the centroid, then try the next 
                    #   best one
                    if len(self.move_base_status) > self.WPT_ID:
                        if self.move_base_status[self.WPT_ID] in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                            print '\tFailed to reach frontier. Trying next one.'
                            self.unreachable_frontiers.append([self.centroidX[self.frontier_index], self.centroidY[self.frontier_index]])
                            self.WPT_ID +=1
                            # Try next best frontier
                            self.centroidX.pop(self.frontier_index)
                            self.centroidY.pop(self.frontier_index)
                            self.cost.pop(self.frontier_index)
                            marker_index = self.centroidIndex[self.frontier_index]
                            self.centroidIndex.pop(self.frontier_index)
                            print '\tOld: ', marker_index, self.frontier_index, len(self.centroidIndex)
                            
                            self.markerArray.markers[marker_index].color.r = 1.0
                            self.markerArray.markers[marker_index].color.g = 0.0
                            self.markerArray.markers[marker_index].color.b = 0.0
                            
                            self.marker_pub.publish(self.markerArray)
                            
                            self.frontier_index = self.updateBestCentoid()
                                
                            if self.frontier_index == -1 or len(self.centroidX)==0:
                                self.removeAllMarkers()
                                print '\tNo more frontiers'
                                return
                            self.move2frontier(self.centroidX[self.frontier_index], self.centroidY[self.frontier_index])
                                
                        elif self.move_base_status[self.WPT_ID] == GoalStatus.SUCCEEDED:
                            self.reached_centroids.append([self.centroidX[self.frontier_index], self.centroidY[self.frontier_index]])
                            self.start_spin = True
                            print 'Spining'   
                            self.spin360()
                            
                self.r.sleep()
                        
            except KeyboardInterrupt:
                print 'Exiting'     
            
        # At the end remove all rviz markers
        self.removeAllMarkers()
        for i in range(5):
            self.r.sleep()
        print 'exiting'
        
    def __init__(self):
        """ Initialize """
        rospy.init_node('Frontier_exploration')
        
        self.listener = tf.TransformListener()        
        
        # Get the parameters for the grid
        self.ogrid_sizeX = rospy.get_param('x_size', 250)
        self.ogrid_sizeY = rospy.get_param('y_size', 250)
        self.grid_size = rospy.get_param('grid_size', 0.05) # in meters/cell (25cm)
        
        # Sensor Meta data
        self.min_range = rospy.get_param('max_range',0.4)
        self.max_range = rospy.get_param('max_range',6.0)
        
        # reliability
        self.p_measurement_given_occupied = rospy.get_param('p_z|occ',0.9)
        self.p_measurement_given_notOccupied = rospy.get_param('p_z|notOcc',0.3)
        
        # Initialize some varables to store the objects to be published/subscribed
        self.position = Position()
        self.robotState = RobotState()
        self.frontierCentroid = Position()
        self.markerArray = MarkerArray()
        self.cur_odom = Position()
        self.prev_odom = Position()
        
        self.gmapping_map = OccupancyGrid()
        self.current_map = OccupancyGrid()
        self.meta = MapMetaData()
#        self.scan = LaserScan()
        self.new_pose = PoseStamped()
        
        # Rotation Booleans
        self.start_spin = True
        self.done_spinning = True
        self.new_cmd = False
        
        # Initialize the map as unknown (-1)
#        self.current_map.data = [-1]*self.ogrid_sizeX*self.ogrid_sizeY # row-major order
        self.unreachable_frontiers = []
        self.reached_centroids=[]
        self.WPT_ID = 0
        self.angle_trav = 0
        self.move_base_status = [0]
        
        self.origin = self.setOrigin()
        
        self.r = rospy.Rate(50)
        
        # publishers for OccupancyGrid and move base messages
#        self.pub_map = rospy.Publisher('/frontier_map', OccupancyGrid, queue_size=100)
        self.marker_pub = rospy.Publisher('/viz_marker_array', MarkerArray, queue_size=100)
        self.cmd_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=100)
        
        # subscribers for odometry and position (/move_base/feedback) messages
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
#        self.feedback_sub = rospy.Subscriber("/move_base/status", PoseStamped, self.feedback_callback)
#        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.sub_map = rospy.Subscriber('/frontier_map', OccupancyGrid, self.costmap_callback)
        self.sub_status = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        
        self.listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(1))
        
        self.run()
        
        return

if  __name__=="__main__":
    l = Frontier_Based_Exploration()

