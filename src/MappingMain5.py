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
from map_msgs.msg import OccupancyGridUpdate
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
global Binary_Cost_Map
global Mask_Map

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
def callback_gamma1(data):
    global Count_Map 
    global listener
    #listener_gamma = tf.TransformListener()

    #Get transform to baselink
    try:
        (trans_gamma, rot_gamma) = listener.lookupTransform('map', '/gamma1_tf',  rospy.Time(0))

        #Log radiation counts at appropriate points
        rospy.loginfo(rospy.get_caller_id() + 'I heard detector 1 at %s', data.data)
        Count_Map.set_cell(trans_gamma[0], trans_gamma[1], 1, addto=True)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass



def callback_gamma2(data):
    global Count_Map 
    global listener
    #listener_gamma = tf.TransformListener()

    #Get transform to baselink
    try:
        (trans_gamma, rot_gamma) = listener.lookupTransform('map', '/gamma2_tf',  rospy.Time(0))

        #Log radiation counts at appropriate points
        rospy.loginfo(rospy.get_caller_id() + 'I heard detector 2 at %s', data.data)
        Count_Map.set_cell(trans_gamma[0], trans_gamma[1], 1, addto=True)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

def callback_gamma3(data):
    global Count_Map 
    global listener
    #listener_gamma = tf.TransformListener()

    #Get transform to baselink
    try:
        (trans_gamma, rot_gamma) = listener.lookupTransform('map', '/gamma3_tf',  rospy.Time(0))

        #Log radiation counts at appropriate points
        rospy.loginfo(rospy.get_caller_id() + 'I heard detector 3 at %s', data.data)
        Count_Map.set_cell(trans_gamma[0], trans_gamma[1], 1, addto=True)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
    
def callback_costmap(data):
    threshold = 50
    global last_cost_update_time
    global Binary_Cost_Map
    #rospy.loginfo("Costmap callback")
    map_rate = .5
    if time.time() > last_cost_update_time + 1/map_rate:
        last_cost_update_time = time.time()
        rospy.loginfo("Costmap callback started (fixed frequency)")

        #rospy.loginfo(Binary_Cost_Map.grid.shape)
        if Binary_Cost_Map.grid.shape[0] * Binary_Cost_Map.grid.shape[1] == len(data.data):
            tmp = np.array(data.data)
            tmp = (tmp >= threshold)
            Binary_Cost_Map.grid = tmp.reshape(Binary_Cost_Map.grid.shape[0], Binary_Cost_Map.grid.shape[1])
            Binary_Cost_Map.last_update = time.time()
        else:
            rospy.loginfo("Problem: Costmap and Binary_Cost_Map out of shape")
        #rospy.loginfo(tmp.shape)

        rospy.loginfo("Costmap callback ended (after {:.3f})".format(time.time() - last_cost_update_time)) 
        #rospy.loginfo(data.data)

def callback_map(data):
    global Binary_Cost_Map
    global Mask_Map
    rospy.loginfo("Map callback called")

    last_map_update_time = time.time()
    if Mask_Map.grid.shape[0] * Mask_Map.grid.shape[1] == len(data.data):
        tmp = np.array(data.data)
        tmp = tmp.reshape(Mask_Map.grid.shape[0], Mask_Map.grid.shape[1])
        # updating Mask_Map. right now it can even reset values (change between 0 and 1)
        # set mask to 0 where occupancy map is 0
        Mask_Map.grid[tmp == 0] = 0
        # set mask to 1 where cost map over threshold
        #print(Binary_Cost_Map.grid.dtype)
        Mask_Map.grid[Binary_Cost_Map.grid] = 1
        # print(len(Mask_Map.grid[Mask_Map.grid == 0]))
        # print(len(Mask_Map.grid[Mask_Map.grid == 1]))
    else:
        rospy.loginfo("Problem: Map and Mask_Map out of shape")
    rospy.loginfo("Map callback ended (after {:.3f})".format(time.time() - last_map_update_time)) 

def MappingMain1():
    global DwellTime_Map
    global Count_Map
    global Binary_Cost_Map
    global Mask_Map
    global trans
    global listener

    global last_cost_update_time

    #Set up CPS maps
    CPS_Map = RadiationMap.Map()
    CPS_GP_Map = RadiationMap.Map()
    GP_Map = RadiationMap.GP_Map()
    Binary_Cost_Map = RadiationMap.Map(-10, -10, .05, 19.20001, 19.20001)
    Binary_Cost_Map.grid = np.full((384, 384), False, dtype = bool)
    Mask_Map = RadiationMap.Map(-10, -10, .05, 19.20001, 19.20001)
    Mask_Map.grid = np.full((384, 384), -1)

    rospy.loginfo("Maps Created")

    # Initialie node
    rospy.init_node('MappingMain1', anonymous=True)

    #Set up transform listener
    listener = tf.TransformListener()

    #Set up subscribers
    rospy.Subscriber('/gamma1', UInt32, callback_gamma1)
    rospy.Subscriber('/gamma2', UInt32, callback_gamma2)
    rospy.Subscriber('/gamma3', UInt32, callback_gamma3)

    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, callback_costmap)
    rospy.Subscriber('/map', OccupancyGrid, callback_map)
    
    #Define radation sources for simulation
    Radiation_Simulation_Flag = False
    Radiation_Sources = []
    Radiation_Sources.append(RadiationSource.RadiationSource(2,[0,0],background_flag=True))
    Radiation_Sources.append(RadiationSource.RadiationSource(500,[0,1]))
    RadSimDelta_t1 = time.time()
    RadSimDelta_t2 = time.time()

    ros_rate = 10
    last_plot_time = time.time()
    last_cost_update_time = time.time()
    last_dwell_time = time.time()
    rate = rospy.Rate(ros_rate) # 10hz
    while not rospy.is_shutdown():
        #Get transform to baselink
        try:
            (trans, rot) = listener.lookupTransform('map', '/base_link',  rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #print("trans" +  repr(trans))

        #Update DwellTime_Map each time through the loop
        #print(time.time() - last_dwell_time)
        if Radiation_Simulation_Flag: #For simulation update with robot center
            DwellTime_Map.set_cell(trans[0], trans[1], time.time() - last_dwell_time, addto=True)
        else: #For actual runs use detector center
            try:
                (trans_gamma, rot_gamma) = listener.lookupTransform('map', '/gamma1_tf',  rospy.Time(0))
                DwellTime_Map.set_cell(trans_gamma[0], trans_gamma[1], time.time() - last_dwell_time, addto=True)
                CPS = Count_Map.get_cell(trans_gamma[0], trans_gamma[1]) / DwellTime_Map.get_cell(trans_gamma[0], trans_gamma[1]) 
                CPS_Map.set_cell(trans_gamma[0], trans_gamma[1], CPS, addto=False)
                #print("Gamma 1 CPS Updated")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("Gamma1 TF Exception")
                pass

            try:
                (trans_gamma, rot_gamma) = listener.lookupTransform('map', '/gamma2_tf',  rospy.Time(0))
                DwellTime_Map.set_cell(trans_gamma[0], trans_gamma[1], time.time() - last_dwell_time, addto=True)
                CPS = Count_Map.get_cell(trans_gamma[0], trans_gamma[1]) / DwellTime_Map.get_cell(trans_gamma[0], trans_gamma[1]) 
                CPS_Map.set_cell(trans_gamma[0], trans_gamma[1], CPS, addto=False)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("Gamma2 TF Exception")
                pass

            try:
                (trans_gamma, rot_gamma) = listener.lookupTransform('map', '/gamma3_tf',  rospy.Time(0))
                DwellTime_Map.set_cell(trans_gamma[0], trans_gamma[1], time.time() - last_dwell_time, addto=True)
                CPS = Count_Map.get_cell(trans_gamma[0], trans_gamma[1]) / DwellTime_Map.get_cell(trans_gamma[0], trans_gamma[1]) 
                CPS_Map.set_cell(trans_gamma[0], trans_gamma[1], CPS, addto=False)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("Gamma3 TF Exception")
                pass

        last_dwell_time = time.time()

        #Generate Random Radation
        if Radiation_Simulation_Flag:
            RadSimDelta_t2 = time.time()
            Sim_Rad = SimulationRadation([trans[0],trans[1]], Radiation_Sources, RadSimDelta_t2 - RadSimDelta_t1)
            RadSimDelta_t1 = RadSimDelta_t2

            #Update Count_Map
            Count_Map.set_cell(trans[0], trans[1], Sim_Rad, addto=True)


        
            #Update CPS maps
            try:
                CPS = Count_Map.get_cell(trans[0], trans[1]) / DwellTime_Map.get_cell(trans[0], trans[1]) 
                CPS_Map.set_cell(trans[0], trans[1], CPS, addto=False)
            except:
                print("Error Updating CPS Map")
                pass
        

        #DwellTime_Map.set_cell(trans[0], trans[1], 5, addto=True) #temp or loc
        #print(time.time())

        #Only update map at a given rate (hz)
        map_rate = .5; 
        if time.time() > last_plot_time + 1/map_rate:
            #Update CPS
            #CPS_map.grid = Count_Map.grid / DwellTime_Map.grid

            #Update GP
            GP_Map.Calc_GP(DwellTime_Map, CPS_Map)

            last_plot_time = time.time()
            #print("Max Grid Value: " + repr(CPS_Map.grid.max()))
            DwellTime_Map.plot_map()

            CPS_Map.plot_map()

            GP_Map.plot_map()

            #Mask_Map.plot_map()

            #Save Data
            np.save('Map_Data/GP_Map_' + repr(time.time()), GP_Map.grid)
            np.save('Map_Data/CPS_Map_' + repr(time.time()), CPS_Map.grid)
            np.save('Map_Data/Mask_Map_' + repr(time.time()), Mask_Map.grid)

        #### not working workaround...
        # map_rate = .5
        # if time.time() > last_cost_update_time + 1/map_rate:
        #     # Update Mask Map
        #     if subscribed:
        #         if received:
        #             sub_cm.unregister() 
        #             subscribed = False
        #             received = False
        #     else:
        #         sub_cm = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, callback_costmap)

        #     last_cost_update_time = time.time()

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
roslaunch turtlebot_teleop keyboard_teleop.launch
rosrun radiation_mapping Turtlebot_sensor_tf.p
rosrun rosserial_python serial_node.py _port:=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH06N802-if00-port0 _baud:=57600


SOME NOTES
Why costmap and updates: 
https://answers.ros.org/question/289710/global-costmap-updating-mysteriously/

to get full costmap to update: 
add following line to turtlebot3_navigation/param/global_costmap_params.yaml
  always_send_full_costmap: true
"""

















