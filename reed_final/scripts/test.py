#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 14:58:00 2017

@author: team3
"""

import rospy
import numpy as np
from visualization_msgs import Marker, MarkerArray
#import tf
#from skimage import measure
#from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
#from geometry_msgs.msg import PoseStamped
#from sensor_msgs.msg import LaserScan
class Position:
    """ Helper class that describes the positon of the turtlebot:
            x,y and theta 
    """
    x = 0.0
    y = 0.0
    theta = 0.0
    
class Test():
    def xy2mapIndex(self, x,y):
        """ Converts the x,y coordinates into a row-major index for the map"""
        return y*self.ogrid_sizeY + x
        
    def mapIndex2xy(self, index):
        x = np.mod(index, self.ogrid_sizeY)
        y = (index-x)/self.ogrid_sizeY
        return x,y
        
    def buildTestMap(self):
       testmap = np.zeros((10,10))
       testmap[0:1][:] = -1
       testmap[8][:] = -1
       testmap[9][:] = -1
       testmap[7][2] = -1
       
       for i in range(10):
           testmap[i][5] = -1
           testmap[i][6] = -1
           testmap[i][2] = -1
           
       testmap[5] = 0
       testmap[6] = 0
       testmap[3][2] =0
       testmap[5][2] =-1
       testmap[6][2] =-1
       testmap[1][2] =0
       testmap[0][3] = 0
       testmap[8][4] = 0
       testmap[9][4] = 0
       testmap[1][1] = 1
       testmap[1][2] = 1
       testmap[2][1] = 1
       testmap[2][2] = 1
       testmap[7][5]= 0
       self.testmap = np.reshape(testmap, (100,1))
       
    def blogDetection(self):
        """ """
        labels = np.ones_like(self.testmap, dtype=np.int8)*-1
        equiv = []
        cur_label = -1
        # Do first pass and label
        for i in range(len(self.testmap)):
            if self.testmap[i]==-1:
                top = False
                left = False
                if (i>self.ogrid_sizeY):
                    topIndex = i-self.ogrid_sizeY
                    top = self.testmap[topIndex] ==-1
                if (np.mod(i,self.ogrid_sizeY) != 0):
                    leftIndex = i-1
                    left = self.testmap[leftIndex] ==-1
                    
                if not top and not left:
                    # Make a new label
                    cur_label +=1
                    label = cur_label
                elif top and left:
                    # mark equivalence
                    if (labels[topIndex] != labels[leftIndex]):
                        newEquiv =[]
                        newEquiv.append(labels[topIndex][0])
                        newEquiv.append(labels[leftIndex][0])
                        if newEquiv not in equiv:
                            equiv.append(newEquiv)
                elif top:
                    label = labels[topIndex]
                elif left:
                    label = labels[leftIndex]
                    
                labels[i] = label
                
        return labels, equiv     
        
        
    def getFrontier(self):
        """ """ 
        frontier = []
        cur_index = 0
        labels, equiv = self.blogDetection()
        labelIndex = np.ones((max(labels)+1,1), dtype=np.int8)*-1
        
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
                    frontier[labelIndex[labels[i][0]][0]].append(xy)
                    
                
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
            if self.testmap[index-10] == 0:
                connected = True
            
        if (index<(self.ogrid_sizeY**2-self.ogrid_sizeY)):
            bottom =True
            # Check the cell below to see if its connected to a known 
            #   cell
            if self.testmap[index+10] == 0:
                connected = True
            
        if (np.mod(index,self.ogrid_sizeY) != 0):
            # Check the cell to the left to see if its connected to a  
            #   known cell
            if self.testmap[index-1] == 0:
                connected = True
            # Check top left
            if top and self.testmap[index-11] == 0:
                connected = True
            # Check bottom left
            if bottom and self.testmap[index+9] == 0:
                connected = True
        
        if (np.mod(index,self.ogrid_sizeY) != self.ogrid_sizeY-1):
            # Check the cell to the right to see if its connected to a 
            #   known cell
            if self.testmap[index+1] == 0:
                connected = True
            # Check top right
            if top and self.testmap[index-9] == 0:
                connected = True
            # Check bottom right
            if bottom and self.testmap[index+11] == 0:
                connected = True
                
        return connected

    def findFrontier(self, blobs, blobs_labels):
        """ This function cycles through each cell in the map and finds all 
            cells that are part of the frontier. The frontier is defined as 
            regions on the border of empty space and unknown space.
        """

        for i in range(len(self.testmap)):
            if self.testmap[i]==-1:
                connected = False
                if (i>self.ogrid_sizeY):
                    # Check the cell above to see if its connected to a known 
                    #   cell
                    if self.testmap[i-10] == 0:
                        connected = True
                    
                if (i<(self.ogrid_sizeY**2-self.ogrid_sizeY)):
                    # Check the cell below to see if its connected to a known 
                    #   cell
                    if self.testmap[i+10] == 0:
                        connected = True
                    
                if (np.mod(i,self.ogrid_sizeY) != 0):
                    # Check the cell to the left to see if its connected to a  
                    #   known cell
                    if self.testmap[i-1] == 0:
                        connected = True
                
                if (np.mod(i,self.ogrid_sizeY) != self.ogrid_sizeY-1):
                    # Check the cell to the right to see if its connected to a 
                    #   known cell
                    if self.testmap[i+1] == 0:
                        connected = True
                        
                # If any of the 4 neighbooring cells were unoccupied, keep the 
                #   true value, otherwise store it as false
                blobs[i] = connected
                
        # label the blobs
#        blobs_labels = measure.label(blobs, background=0)
        
        return np.reshape(blobs, (10,10))

    def calcUtil_dist(self, centroidX, centroidY):
        """ Calculate the utility of the frontier's centroid using just the
            distance to the centroid. 
        """
        dist = self.distanceFomula(centroidX, centroidY, self.position.x, self.position.y)
        return dist

    def distanceFomula(self, x1,y1, x2,y2):
         return np.sqrt(np.square(x2 - x1) + np.square(y2-y1))
         
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
        
    def pickBestCentroid(self, frontiers):
        """ Takes in all frontiers (as a 3D array) and choses the best frontier"""
        centroidX = []
        centroidY = []
        utility = []
        # Reinitialize the marker array
        self.markerArray = MarkerArray()
        
        for i in range(len(frontiers)):
            x_c, y_c, distance_c = self.calc_centroid(frontiers[i])
            self.makeMarker(x_c, y_c, i)
            centroidX.append(x_c)
            centroidY.append(y_c)
            utility.append(distance_c)
            
        # Determine which centroid is the closest 
        index = np.argmax(utility)
        
        # Mark the centroid to navigate to as red
        self.markerArray.markers[index].color.r = 1.0
        self.markerArray.markers[index].color.b = 0.0
        self.markerArray.markers[index].color.g = 0.0
        
        # Publish the markerArray
        self.marker_pub.publish(self.markerArray)
        
        return centroidX[index], centroidY[index]
        
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
        
#        # Renumber the marker IDs
#        id = 0
#        for m in self.markerArray.markers:
#            m.id = id
#            id += 1
        
        
    def __init__(self):
        self.position = Position()
        self.markerArray = MarkerArray()
        self.position.x = 2.0
        self.position.y = 4.0
        
        self.ogrid_sizeY = 10
        self.buildTestMap()
        
        self.marker_pub = rospy.Publisher('/viz_marker_array', MarkerArray)
        
        frontier = t.getFrontier()
        return self.pickBestCentroid(frontier)
        
#        frontiers = self.findFrontier()
#        self.pickBestCentroid(frontiers)
        
        
#        rospy.init_node('TEST')
#        
#        # loop rate
#        self.r = rospy.Rate(50)
#        
#        self.listener = tf.TransformListener() 
#        
#        self.listener.waitForTransform('/odom', 'map', rospy.Time(0), rospy.Duration(1))
#        while(not rospy.is_shutdown()):
#            (self.trans,self.rot) = self.listener.lookupTransform('/odom', 'map', rospy.Time(0))
#            print "odom to map: ", self.trans, self.rot
#            
#            (self.trans,self.rot) = self.listener.lookupTransform('/map', '/odom', rospy.Time(0))
#            print "map to odom: ", self.trans, self.rot
#            self.r.sleep()
#        return

t = Test()
#orig, label = t.findBlobs()
#blobs  = t.findFrontier(orig, label) #
#orig = np.reshape(orig, (10,10))
#label = np.reshape(label, (10,10))
#l,equiv = t.blogDetection()
#label2 = np.reshape(l, (10,10))

#frontier =np.reshape(frontier, (10,10))