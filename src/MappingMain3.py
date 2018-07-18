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
import RadiationMap
import RadiationSource

#Global counter variables
global DwellTime_Map #Dwell time at each map voxel in seconds
global Count_Map #Number of radiation coutns accumulated at each map voxel
global trans
DwellTime_Map = RadiationMap.Map()
Count_Map = RadiationMap.Map()


def ComputeGP(CPS_Map, lambda1 = 0.1):
    #Use CPS map with parameters to compute GP estimates at every point

    pass



def SimulationRadation(current_location, sources, delta_t):
    rad = 0

    for source in sources:
        rad += source.GenerateRandomRadiation(current_location, delta_t)

    return rad


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

    #Define radation sources for simulation
    Radiation_Simulation_Flag = True
    Radiation_Sources = []
    Radiation_Sources.append(RadiationSource.RadiationSource(2,[0,0],background_flag=True))
    Radiation_Sources.append(RadiationSource.RadiationSource(500,[1.5,1.5]))
    RadSimDelta_t1 = time.time()
    RadSimDelta_t2 = time.time()

    #Set up CPS maps
    CPS_Map = RadiationMap.Map()
    CPS_GP_Map = RadiationMap.Map()

    ros_rate = 10
    last_plot_time = time.time()
    rate = rospy.Rate(ros_rate) # 10hz
    while not rospy.is_shutdown():
        #Get transform to baselink
        try:
            (trans, rot) = listener.lookupTransform('map', '/base_link',  rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #print("trans" +  repr(trans))

        #Update DwellTime_Map each time through the loop
        DwellTime_Map.set_cell(trans[0], trans[1], 1.0/float(ros_rate), addto=True)

        #Generate Random Radation
        if Radiation_Simulation_Flag:
            RadSimDelta_t2 = time.time()
            Sim_Rad = SimulationRadation([trans[0],trans[1]], Radiation_Sources, RadSimDelta_t2 - RadSimDelta_t1)
            RadSimDelta_t1 = RadSimDelta_t2

            #Update Count_Map
            Count_Map.set_cell(trans[0], trans[1], Sim_Rad, addto=True)
        
        #Update CPS maps
        CPS = Count_Map.get_cell(trans[0], trans[1]) / DwellTime_Map.get_cell(trans[0], trans[1]) 
        CPS_Map.set_cell(trans[0], trans[1], CPS, addto=False)
        

        #DwellTime_Map.set_cell(trans[0], trans[1], 5, addto=True) #temp or loc
        #print(time.time())

        #Only update map at a given rate (hz)
        map_rate = 1; 
        if time.time() > last_plot_time + 1/map_rate:
            last_plot_time = time.time()
            print("Max Grid Value: " + repr(CPS_Map.grid.max()))
            DwellTime_Map.plot_map()

            CPS_Map.plot_map()

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

















