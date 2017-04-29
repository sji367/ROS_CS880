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
        print map_msg.info
        self.new_msg = True 
        
        self.meta.resolution= map_msg.info.resolution
        self.meta.height = map_msg.height
        self.meta.width = map_msg.width
        self.meta.origin.position.x = map_msg.origin.position.x
        self.meta.origin.position.y = map_msg.origin.position.y
    
    def scan_callback(self, scan_msg):
        self.scan.ranges = scan_msg.ranges
        self.scan.angle_increment = scan_msg.angle_increment
        self.scan.angle_min = scan_msg.angle_min
        
#        print scan_msg
        
#    
#    
#    def feedback_callback(self, feedback_msg):
#        """ Callback to handle feedback messages so that we can stop the 
#                turtlebot at its current position."""        
#        # Store the position and orientation
#        self.pos.pose.position.x = feedback_msg.pose.position.x
#        self.pos.pose.position.y = feedback_msg.pose.position.y
#        self.pos.pose.orientation.z = feedback_msg.pose.orientation.z
#        self.pos.pose.orientation.w = feedback_msg.pose.orientation.w      
#    
#    def odom_callback(self, odom_msg):
#        """ callback to handle odometry messages"""
#        odom = Odometry()
#        # We only care about the x,y position and the z and w orientation
#        odom.pose.pose.position.x = odom_msg.pose.pose.position.x
#        odom.pose.pose.position.y = odom_msg.pose.pose.position.y
#        odom.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
#        odom.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w
#        
#        if not self.got_origin:
#            # convert those into x/y/theta positions
#            self.origin.x = odom.pose.pose.position.x
#            self.origin.y = odom.pose.pose.position.y
#            self.origin.theta = normalize_angle(self.rotation_distance(odom.pose.pose.orientation.z,
#                                                                         odom.pose.pose.orientation.w))
#            self.got_origin = True
#            self.current_map.info.origin.position.x,self.current_map.info.origin.position.y = self.xy2grid()
#            self.current_map.info.origin.position.z = 0
#            self.current_map.info.origin.orientation.x = 0
#            self.current_map.info.origin.orientation.y = 0
#            self.current_map.info.origin.orientation.z = 0
#            self.current_map.info.origin.orientation.w = 1
            
                                                                     
    
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
        
    def distanceFomula(self, x1,y1, x2,y2):
         return np.sqrt(np.square(x2 - x1) + np.square(y2-y1))
         
    def calcUtility(self, centroidX, centroidY, frontierLength):
        """ Calculate the utility of the frontier's centroid using a combo of
            the distance to the centroid and the length of the centroid.
        """
        dist_weight = 1
        length_weight = 1
        dist = self.distanceFomula(centroidX, centroidY, self.position.x, self.position.y)
        util = (length_weight*frontierLength)/(dist_weight*dist)
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
        # convert to probability (currently in range from 0 to 100)
        else:
            prior *= .01
        
        # Calculate eta
        eta = 1/(self.p_measurement_given_occupied*prior + self.p_measurement_given_notOccupied*(1-prior))

        if Free:
            prob_occupancy = eta*self.p_measurement_given_notOccupied*(prior)
        else:
            prob_occupancy = eta*self.p_measurement_given_occupied*prior
#            print prob_occupancy, index
            
        self.current_map.data[index] = int(prob_occupancy*100)
    
    def updateNeighbors(self, scanX, scanY, camera_pos):
        """ Update the free occupancy grid cells between the robot and the   
            obstacle found using the laser scan.
        """
        camX, camY = self.xy2grid(camera_pos.x, camera_pos.y)
        robotX, robotY = self.xy2grid(self.position.x, self.position.y)
        
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
        
        laser_data=self.scan.ranges
        cur_angle = normalize_angle(self.scan.angle_min+camera_pos.theta)
        print 'update',len(laser_data)
        for i in range(len(laser_data)):
            if not np.isnan(laser_data[i]):            
                # Update the current angle to reflect the angle of the laser 
                #  scan.
                cur_angle = normalize_angle(self.scan.angle_increment + cur_angle)
                
                # Determine the location of the obstacle found by the laser scan
                scanX = laser_data[i]*np.cos(cur_angle)+camera_pos.x
                scanY = laser_data[i]*np.sin(cur_angle)+camera_pos.y
                            
                scanX_grid, scanY_grid = self.xy2grid(scanX, scanY)
                
                # Update the map between the obstacle and the robot
                self.updateNeighbors(scanX_grid, scanY_grid, camera_pos)
                    
                # Update the map to include the obstacle
                row_major_index = self.xy2mapIndex(scanX_grid, scanY_grid)
#                print row_major_index
                self.setOccupancy(row_major_index, self.current_map.data[row_major_index], False)           
        self.publishNewMap()
        
    def publishNewMap(self):
        self.current_map.header.frame_id = 'map'
#        originX = self.origin.x*self.grid_size
#        originY = self.origin.y*self.grid_size
#        cur_pos= Position()
##        cur_pos= self.get_current_position()
##        print cur_pos.x, cur_pos.y
##        print cur_pos.x-originX,  cur_pos.y-originY
        dif_in_middleX = self.origin.x*self.grid_size-self.meta.width*self.meta.resolution
        dif_in_middleY = self.origin.y*self.grid_size-self.meta.height*self.meta.resolution
#        print dif_in_middleX, dif_in_middleY
        self.current_map.info.origin.position.x = -dif_in_middleX
        self.current_map.info.origin.position.y = -dif_in_middleY
        self.current_map.info.origin.orientation.w = 1
#        print self.meta.origin.position.x, self.meta.origin.position.y
        self.pub_map.publish(self.current_map)
                
    def blogDetection(self):
        """ """
        labels = np.ones_like(self.current_map.data, dtype=np.int8)*-1
        equiv = []
        cur_label = -1
        # Do first pass and label
        for i in range(len(self.current_map.data)):
            if self.current_map.data[i]==-1:
                top = False
                left = False
                if (i>self.ogrid_sizeY):
                    topIndex = i-self.ogrid_sizeY
                    top = self.current_map.data[topIndex] ==-1
                if (np.mod(i,self.ogrid_sizeY) != 0):
                    leftIndex = i-1
                    left = self.current_map.data[leftIndex] ==-1
                    
                if not top and not left:
                    # Make a new label
                    cur_label +=1
                    label = cur_label
                    print label
                elif top and left:
                    # mark equivalence
                    if (labels[topIndex] != labels[leftIndex]):
                        newEquiv =[]
                        newEquiv.append(labels[topIndex])
                        newEquiv.append(labels[leftIndex])
                        if newEquiv not in equiv:
                            equiv.append(newEquiv)
                elif top:
                    label = labels[topIndex]
                elif left:
                    label = labels[leftIndex]
                    
                labels[i] = label
                
        print equiv
        return labels, equiv     
        
        
    def getFrontier(self):
        """ """ 
        frontier = []
        cur_index = 0
        labels, equiv = self.blogDetection()
        labelIndex = np.ones((max(labels)+1,1), dtype=np.int8)*-1
        print 'start'
        #Second pass to remove equivilencies and store the frontiers
        num_equiv = len(equiv)
        for i in range(len(labels)):
            # Store the frontiers as a list
            if labels[i]!=-1:
                # Remove the equivalencies
                for ii in range(num_equiv):
                    if labels[i] == equiv[ii][0]:
                        labels[i] = equiv[ii][1]
                        
                # Check to see if the point is on a frontier (aka Moore Neighboor with an open space)
                if self.onFrontier(i):
                    print "on frontier!"
                    xy = []
                    # If this is the first time we have reached this label then 
                    #   store which row we are going to store that label
                    if labelIndex[labels[i]] == -1:
                        frontier.append([])
                        labelIndex[labels[i]] = cur_index
                        cur_index += 1
                        
                    # Next store the index of the map into the correct row of 
                    #   frontier (they are numpy arrays so they are nxmx1)
                    x,y = self.mapIndex2xy(i)
                    xy.append(x)
                    xy.append(y)
                    frontier[labelIndex[labels[i][0]]].append(xy)
                    
        print 'got frontiers'        
        return frontier
                
    def onFrontier(self, index):
        """ This function takes in an index of a cell in the map and determines
            if it is part of the frontier. The frontier is defined as regions
            on the border of empty space and unknown space.
        """
        connected = False
        top = False
        bottom = False
        if (index>self.ogrid_sizeY):
            top = True
            # Check the cell above to see if its connected to a known 
            #   cell
            if self.current_map.data[index-10] < 50 and self.current_map.data[index]>=0:
                connected = True
            
        if (index<(self.ogrid_sizeY**2-self.ogrid_sizeY)):
            bottom =True
            # Check the cell below to see if its connected to a known 
            #   cell
            if self.current_map.data[index+10] < 50 and self.current_map.data[index]>=0:
                connected = True
            
        if (np.mod(index,self.ogrid_sizeY) != 0):
            # Check the cell to the left to see if its connected to a  
            #   known cell
            if self.current_map.data[index-1] < 50 and self.current_map.data[index]>=0:
                connected = True
            # Check top left
            if top and self.current_map.data[index-11] < 50 and self.current_map.data[index]>=0:
                connected = True
            # Check bottom left
            if bottom and self.current_map.data[index+9] < 50 and self.current_map.data[index]>=0:
                connected = True
        
        if (np.mod(index,self.ogrid_sizeY) != self.ogrid_sizeY-1):
            # Check the cell to the right to see if its connected to a 
            #   known cell
            if self.current_map.data[index+1] < 50 and self.current_map.data[index]>=0:
                connected = True
            # Check top right
            if top and self.current_map.data[index-9] < 50 and self.current_map.data[index]>=0:
                connected = True
            # Check bottom right
            if bottom and self.current_map.data[index+11] < 50 and self.current_map.data[index]>=0:
                connected = True
                
        return connected

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
        return x_c, y_c, self.calcUtility(x_c, y_c, len(points))
        
    def pickBestCentroid(self, frontiers):
        """ Takes in all frontiers (as a 3D array) and choses the best frontier"""
        centroidX = []
        centroidY = []
        utility = []
        # Reinitialize the marker array
        self.markerArray = MarkerArray()
        print 'picking centroid ', len(frontiers)
        
        for i in range(len(frontiers)):
            x_c, y_c, util_c = self.calc_centroid(frontiers[i])
            # Store centroid if it is known and 
            index = self.xy2mapIndex(x_c, y_c)
            if self.current_map[index]< 50 and self.current_map[index]>=0:
#            print x_c, y_c
                self.makeMarker(x_c, y_c, i)
                centroidX.append(x_c)
                centroidY.append(y_c)
                utility.append(util_c)
        if len(centroidX)>0:
            # Determine which centroid is the closest 
            index = np.argmax(utility)
            
            # Mark the centroid to navigate to as red
            self.markerArray.markers[index].color.r = 1.0
            self.markerArray.markers[index].color.b = 0.0
            self.markerArray.markers[index].color.g = 0.0
            
            # Publish the markerArray
            self.marker_pub.publish(self.markerArray)
            print 'published marker'
            
            return centroidX[index], centroidY[index]
        else:
            print 'No Valid centroids'
            return None, None
        
    def makeMarker(self, centroidX, centroidY, ID):
        """ Creates a marker on RVIZ for the centroid of the frontier"""
        
        RVIZmarker = Marker()
        
        RVIZmarker.header.frame_id = '/map'
        RVIZmarker.header.stamp = rospy.Time(0)
        
        RVIZmarker.ns = 'Frontier Contour'
        RVIZmarker.id = ID
        RVIZmarker.type = Marker.CUBE
        RVIZmarker.action = Marker.ADD
        
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
        self.markerArray.markers.append(RVIZmarker)
        
    def get_current_position(self):
        """ Callback to get the current position"""
        position = Position()
        trans, rot = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
        position.x= trans[0]
        position.y = trans[1]
        position.theta= normalize_angle(self.rotation_distance(rot[2],rot[3]))
        
        return position
        
    def getCameraPositon(self):
        position = Position()
        trans, rot = self.listener.lookupTransform('map', 'camera_depth_frame', rospy.Time(0))
        position.x= trans[0]
        position.y = trans[1]
        position.theta= normalize_angle(self.rotation_distance(rot[2],rot[3]))
        
        return position
        
    def setOrigin(self):
        origin = Position()
        origin.x = self.ogrid_sizeX/2
        origin.y = self.ogrid_sizeY/2
        origin.theta = 0
        
#        trans, rot = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
        self.current_map.info.origin = self.meta.origin
#        self.current_map.info.origin.position.x = origin.x*self.grid_size
#        self.current_map.info.origin.position.y = origin.y*self.grid_size
#        self.current_map.info.origin.orientation.z = rot[2]
#        self.current_map.info.origin.orientation.w = rot[3]
        self.current_map.info.height = self.ogrid_sizeX
        self.current_map.info.width = self.ogrid_sizeY
        self.current_map.info.resolution = self.grid_size
        
        return origin
    
    def newPose(self, centroidX, centroidY):
        """Takes in the new waypoint and sets the new goal position"""
        # Convert the grid coordinates to XY coordinates
        gridX, gridY = self.grid2xy(centroidX, centroidY)
        
        new = PoseStamped()
        new.header.seq=1
        new.header.stamp.secs = rospy.get_rostime().to_sec()
        
        new.header.frame_id= 'map'
        
        # Positions in the map
        new.pose.position.x = gridX
        new.pose.position.y = gridY
        new.pose.position.z = 0.0
        
        new.pose.orientation.x = 0.0
        new.pose.orientation.y = 0.0
        new.pose.orientation.z = 0.0
        new.pose.orientation.w = 1.0
        
        return new
        
    def run(self):
        """ Runs the frontier based exploration. """
#        pass
        start = rospy.Time.now().to_sec()
        # IN THIS LOOP ROS SENDS AND RECEIVES  
        while not rospy.is_shutdown():
            start = rospy.Time.now().to_sec()
            self.updateMap()
            print rospy.Time.now().to_sec()-start
#            if (rospy.Time.now().to_sec()-start) > 1:
#                frontiers = self.getFrontier()
#                centroidX, centroidY = self.pickBestCentroid(frontiers)
#                print centroidX, centroidY
#            if (rospy.Time.now().to_sec()-start) > 2: 
#                return
            
#            if self.new_msg:
#                print 'inside'
#                frontiers = self.getFrontier()
#                centroidX, centroidY = self.pickBestCentroid(frontiers)
#                self.new_msg = False
            
            
            # Transform the odometry message to the map reference frame
            #   trans (Translation) - [x,y,z]
            #   rot (Rotation) - quentieron [x,y,z,w]
#            self.get_current_position()
#            
#            # if you are at your position goal, then 
#            if (self.position.x == self.frontierCentroid.x) and (self.position.y == self.frontierCentroid.y):
#                frontiers = self.findFrontiers()
#                centroidX, centroidY = self.pickBestCentroid(frontiers)
#                self.move2Centroid(centroidX, centroidY)
#            else:
#                while (len(self.new_scans)>0):
#                    self.updateMap()
            self.r.sleep()
        
    def __init__(self):
        """ Initialize """
        rospy.init_node('Frontier_exploration')
        
#        self.got_origin = False
        self.new_msg = False
        self.listener = tf.TransformListener()        
        
        # Get the parameters for the grid
        self.ogrid_sizeX = rospy.get_param('x_size', 250)
        self.ogrid_sizeY = rospy.get_param('y_size', 250)
        self.grid_size = rospy.get_param('grid_size', 0.15) # in meters/cell (25cm)
        
        # Sensor Meta data
        self.max_range = rospy.get_param('max_range',8.0)
        
        # reliability
        self.p_measurement_given_occupied = rospy.get_param('p_z|occ',0.9)
        self.p_measurement_given_notOccupied = rospy.get_param('p_z|notOcc',0.3)
        
        # Initialize some varables to store the objects to be published/subscribed
        self.position = Position()
        self.frontierCentroid = Position()
        self.markerArray = MarkerArray()
#        self.pos = PoseStamped()
        
        self.current_map = OccupancyGrid()
        self.meta = MapMetaData()
        self.scan = LaserScan()
        self.new_pose = PoseStamped()
        
        # Initialize the map as unknown (-1)
        self.current_map.data = [-1]*self.ogrid_sizeX*self.ogrid_sizeY # row-major order
#        print len(self.current_map.data)
        # loop rate
        self.r = rospy.Rate(50)
        
        # publishers for OccupancyGrid and move base messages
        self.pub_map = rospy.Publisher('/frontier_map', OccupancyGrid, queue_size=100)
        self.marker_pub = rospy.Publisher('/viz_marker_array', MarkerArray, queue_size=100)
#        self.cmd_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)
        
        # subscribers for odometry and position (/move_base/feedback) messages
#        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
#        self.feedback_sub = rospy.Subscriber("/move_base/status", PoseStamped, self.feedback_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.sub_map = rospy.Subscriber('/map/info', MapMetaData, self.costmap_callback)
        
        self.listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(1))
        
        self.origin = self.setOrigin()
        #self.listener.waitForTransform('/scan', '/base_link', rospy.Time(0), rospy.Duration(1)) # Need to test this
        self.run()
        
        return

if  __name__=="__main__":
    l = Frontier_Based_Exploration()

