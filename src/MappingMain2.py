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
from std_msgs.msg import UInt32, String
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
import tf

import math
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pylab
import matplotlib.image as mpimg
from matplotlib import cm
import matplotlib.mlab as mlab

#Define Classes 
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
        self.im1 = False
        self.last_location = [0,0]

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

        #Temp for checking location
        #self.grid = self.grid*0

        map_indices = self.xy_to_index(x, y)
        if map_indices == False:
            rospy.loginfo("Location outside of Radiation Map")
            return
        elif addto == True:
            self.grid[map_indices[1]][map_indices[0]] += val
        else:
            self.grid[map_indices[1]][map_indices[0]] = val

        self.last_location = [x,y]


    def xy_to_index(self, x, y):
        #Map x,y values in map frame to map indices
        #Returns False if outisde of map

        x_i = math.floor((x-self.origin_x)/self.resolution)
        y_i = math.floor((y-self.origin_y)/self.resolution)
        if x_i < 0 or x_i >= len(self.grid[1]):
            return False
        if y_i < 0 or y_i >= len(self.grid):
            return False

        return [int(x_i), int(y_i)]

    def plot_map(self):
        #Plot map

        #fig = plt.figure()
        if self.im1 == False:
            self.fig= plt.figure()
            self.im1 = plt.imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [DwellTime_Map.origin_x, DwellTime_Map.origin_x+DwellTime_Map.width, DwellTime_Map.origin_y, DwellTime_Map.origin_y+DwellTime_Map.height])
            #self.location_marker = plt.scatter([self.last_location[0]],[self.last_location[1]], c='r', s=40)
            plt.show(block = False)
            print("Map Plot Created")
        else:
            self.im1.set_data(np.rot90(self.grid, k = -1))
            #self.location_marker.set_data([self.last_location[0]],[self.last_location[1]])
            
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            print("Map updated")


#Global counter variables
global DwellTime_Map #Dwell time at each map voxel in seconds
global Count_Map #Number of radiation coutns accumulated at each map voxel
global trans
DwellTime_Map = Map()
Count_Map = Map()


def ComputeGP(CPS_Map, lambda = 0.1):
    #Use CPS map with parameters to compute GP estimates at every point

    pass


def SimulationRadation(current_location):
    rad = 0

    background_rate = 2
    rad += GenerateRandomRadation(background_rate)

    #Source locations in map frame
    source_locations = [[1,1],[-.75, -.5]]


    for source in source_locations:
        #Generate radiation depending on source location
        pass

    return rad



def GenerateRandomRadation(source_activity): 
    #Generate random radiation samples based upon activity
    pass

#---------------CallBacks------------------
def callback(data):
    global DwellTime_Map
    global Count_Map 
    global trans
    rospy.loginfo(rospy.get_caller_id() + 'I heard detector 1 at %s', data.data)
    Count_Map.set_cell(trans[0], trans[1], 1.0/float(ros_rate), addto=True)
    

def MappingMain1():
    global DwellTime_Map
    global Count_Map
    global trans
    # Initialie node
    rospy.init_node('MappingMain1', anonymous=True)

    #Set up subscribers
    rospy.Subscriber('/gamma1', String, callback)

    #Set up transform listener
    listener = tf.TransformListener()

    #Set up CPS maps
    CPS_Map = Map()
    CPS_GP_Map = Map()

    ros_rate = 10
    last_plot_time = time.time()
    rate = rospy.Rate(ros_rate) # 10hz
    while not rospy.is_shutdown():
        #Get transform to baselink
        try:
            (trans, rot) = listener.lookupTransform('map', '/base_link',  rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print("trans" +  repr(trans))

        #Update DwellTime_Map each time through the loop
        DwellTime_Map.set_cell(trans[0], trans[1], 1.0/float(ros_rate), addto=True)

        #Update CPS maps

        #DwellTime_Map.set_cell(trans[0], trans[1], 5, addto=True) #temp or loc
        #print(time.time())

        #Only update map at a given rate (hz)
        map_rate = 1; 
        if time.time() > last_plot_time + 1/map_rate:
            last_plot_time = time.time()
            print("Max Grid Value: " + repr(DwellTime_Map.grid.max()))
            DwellTime_Map.plot_map()

        rate.sleep()

if __name__ == '__main__':
    try:
        MappingMain1()
    except rospy.ROSInterruptException:
        pass

"""ROS commands:
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=frontier_exploration 
rosrun radiation_mapping MappingMain1.py 
roslaunch turtlebot3_bringup turtlebot3_robot.launch
ssh bloodhound@192.168.207.14
rosrun rqt_tf_tree rqt_tf_tree
rosrun tf tf_echo /map /base_link


"""

















