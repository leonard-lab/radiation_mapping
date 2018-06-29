#!/usr/bin/env python
# Software License Agreement (BSD License)
#

## Main radiation mapping function
## Listens to radiation count data
## Updates map with radiation count data
## Updates map with dwell time data
## Performs Gaussian Process update
## 

import rospy
from std_msgs.msg import UInt32
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
import tf

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pylab
import matplotlib.image as mpimg
from matplotlib import cm
import matplotlib.mlab as mlab

#Global counter variables
global DwellTimeMap #Dwell time at each map voxel in seconds
global CountMap #Number of radiation coutns accumulated at each map voxel
DwellTimeMap = Map()
CountMap = Map()

def callback(data):
    global DwellTimeMap
    global CountMap
    rospy.loginfo(rospy.get_caller_id() + 'I heard detector 1 at %s', data.data)
    #CountMap += 1

def MappingMain1():
    global DwellTimeMap
    global CountMap
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('MappingMain1', anonymous=True)

    rospy.Subscriber('/gamma1', String, callback)

    #Set up transform listener
    listener = tf.TransformListener()


    ros_rate = 10
    rate = rospy.Rate(ros_rate) # 10hz
    while not rospy.is_shutdown():
        #Get transform to baselink
        try:
            (trans, rot_ = listener.lookupTransform('/base_link', 'map', rospy.Time(0)))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue



        #Update DwellTimeMap each time through the loop
        DwellTimeMap.set_cell(trans[0]. trans[1], 1.0/float(ros_rate), addto=True)

        rate.sleep()

if __name__ == '__main__':
    MappingMain1()
except rospy.ROSInterruptException: pass


class Map:
    """ 
    The Map class stores an occupancy grid as a two dimensional
    numpy array. 
    
    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters. 
        origin_x   --  Position of the grid cell (0,0) in 
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.
        
    
    Note that x increases with increasing column number and y increases
    with increasing row number. 
    """

    def __init__(self, origin_x=-2, origin_y=-2, resolution=.1, 
                 width=4, height=4):
        """ Construct an empty occupancy grid.
        
        Arguments: origin_x, 
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells 
                                in meters.
                   width, 
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.
                                
         The default arguments put (0,0) in the center of the grid. 
                                
        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width 
        self.height = height 
        self.grid = np.zeros((int(height/resolution), int(width/resolution)))

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
     
        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        grid_msg.data = list(np.round(flat_grid))
        return grid_msg

    def set_cell(self, x, y, val, addto=False):
        """ Set the value of a cell in the grid. 

        Arguments: 
            x, y  - This is a point in the map coordinate frame.
            val   - This is the value that should be assigned to the
                    grid cell that contains (x,y).
            addto - Flag tht indicates that val should be added to current value."""

        #Check for values outside of map, log error if so
        map_indices = self.xy_to_index(x, y)
        if map_indices == False:
            rospy.loginfo("Location outside of Radiation Map")
        elif addto == True:
            self.grid[map_indices[1]][map_indices[0]] += val
        else:
            self.grid[map_indices[1]][map_indices[0]] = val


    def xy_to_index(self, x, y):
        #Map x,y values in map frame to map indices
        #Returns False if outisde of map

        x_i = math.floor((x-self.origin_x)/self.resolution)
        y_i = math.floor((y-self.origin_y)/self.resolution)
        if x_i < 0 or x_i >= len(self.grid[1]):
            return False
        if y_i < 0 or y_i >= len(self.grid):
            return False

        return [x_i, y_i]

    def plot_map(self):
        #Plot map

        fig = plt.figure()

        im1 = plt.imshow(self.grid, interpolation='none', origin = 'lower', extent = [self.origin_x, self.origin_x+self.width, self.origin_y, self.origin_y+self.height])

        #im1 = plt.imshow(self.Map, 'XData', self.origin_x:.5:(self.origin_x+self.width), 'YData', self.origin_y:.5:(self.origin_y+self.height))
        #cax = plt.axes()
        plt.colorbar()
        plt.xlabel('x position (m)')
        plt.ylabel('y position (m)')


        plt.show()












