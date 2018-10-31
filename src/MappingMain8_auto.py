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
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionGoal
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
from Bloodhound_Functions import *

#Define Classes 
import RadiationMap
import RadiationSource

#Global counter variables
global DwellTime_Map #Dwell time at each map voxel in seconds
global Count_Map #Number of radiation coutns accumulated at each map voxel
global trans
global Binary_Cost_Map
global Mask_Map
global RecordMapParameters
global GPMapParameters
global last_cost_update_time 
global Cost_Map

#Map parameters
MapWidth = 10.0001
MaskMapWidth = 19.200001
RecordMapParameters = [-MapWidth/2, -MapWidth/2, 0.05, MapWidth, MapWidth]
GPMapParameters = [-MapWidth/2, -MapWidth/2, 0.05, MapWidth, MapWidth]
MaskMapParameters = [-10, -10, 0.05, MaskMapWidth, MaskMapWidth]

DwellTime_Map = RadiationMap.Map(*RecordMapParameters, dtype = 'float32')
Count_Map = RadiationMap.Map(*RecordMapParameters, dtype = 'int16')

#Initial Time
last_cost_update_time = time.time()


#---------------CallBacks------------------
def callback_gamma1(data):
    global Count_Map 
    global listener
    #listener_gamma = tf.TransformListener()

    #Get transform to baselink
    try:
        (trans_gamma, rot_gamma) = listener.lookupTransform('map', '/gamma1_tf',  rospy.Time(0))

        #Log radiation counts at appropriate points
        #rospy.loginfo(rospy.get_caller_id() + 'I heard detector 1 at %s', data.data)
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
        #rospy.loginfo(rospy.get_caller_id() + 'I heard detector 2 at %s', data.data)
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
        #rospy.loginfo(rospy.get_caller_id() + 'I heard detector 3 at %s', data.data)
        Count_Map.set_cell(trans_gamma[0], trans_gamma[1], 1, addto=True)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

def callback_dwelltimeupdate(data):
    global DwellTime_Map
    global Count_Map
    global CPS_Map

    DwellTime_Map.set_cell(data.x, data.y, data.z, addto=True)
    CPS = Count_Map.get_cell(data.x, data.y) / DwellTime_Map.get_cell(data.x, data.y) 
    CPS_Map.set_cell(data.x, data.y, CPS, addto=False)

    if np.isnan(CPS_Map.get_cell(data.x, data.y)):
        print("ALERT: NAN GENERATED in callback_dwelltimeupdate")
        CPS_Map.set_cell(data.x, data.y, 1, addto=False)

    
def callback_costmap(data):
    threshold_hard = 80
    threshold_average = 67
    global last_cost_update_time
    global Binary_Cost_Map
    global Cost_Map
    global DwellTime_Map
    global Cost_Samples_Map

    #Costmap test function
    try:
        (trans, rot_gamma) = listener.lookupTransform('map', '/base_scan',  rospy.Time(0))

        #Log radiation counts at appropriate points
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

    #rospy.loginfo("Costmap callback")
    map_rate = .5
    if time.time() > last_cost_update_time + 1/map_rate:
        last_cost_update_time = time.time()
        #rospy.loginfo("Costmap callback started (fixed frequency)")

        #rospy.loginfo(Binary_Cost_Map.grid.shape)
        if Binary_Cost_Map.grid.shape[0] * Binary_Cost_Map.grid.shape[1] == len(data.data):
            tmp = np.array(data.data)

            new_info_weight = 0.2

            Cost_Map.updateGrid(Cost_Map.grid*(1-new_info_weight) + tmp.reshape(Binary_Cost_Map.grid.shape[0], Binary_Cost_Map.grid.shape[1]) * new_info_weight)
            
            #Cost_Samples_Map.updateGrid(Cost_Samples_Map.grid + (tmp.reshape(Binary_Cost_Map.grid.shape[0], Binary_Cost_Map.grid.shape[1]) >= 0))

            # print("Costmap at Current Location")
            # ind = Binary_Cost_Map.xy_to_index(trans[0],trans[1])
            # print(trans)
            # print(tmp.reshape(Binary_Cost_Map.grid.shape[0], Binary_Cost_Map.grid.shape[1])[ind[1]][ind[0]])
            

            #tmp = (tmp >= threshold_hard)

            Binary_Cost_Map.updateGrid( Cost_Map.grid >= threshold_average )

            Binary_Cost_Map.grid[ tmp.reshape( Binary_Cost_Map.grid.shape[0], Binary_Cost_Map.grid.shape[1] ) >= threshold_hard ] = True





        else:
            rospy.loginfo("Problem: Costmap and Binary_Cost_Map out of shape")
            print(Binary_Cost_Map.grid.shape[0] * Binary_Cost_Map.grid.shape[1])
            print(len(data.data))
        #rospy.loginfo(tmp.shape)

        #rospy.loginfo("Costmap callback ended (after {:.3f})".format(time.time() - last_cost_update_time)) 
        #rospy.loginfo(data.data)

def callback_map(data):
    global Binary_Cost_Map
    global Mask_Map
    global occupancy_Map
    #rospy.loginfo("Map callback called")

    last_map_update_time = time.time()
    if Mask_Map.grid.shape[0] * Mask_Map.grid.shape[1] == len(data.data):

        tmp = np.array(data.data)
        tmp = tmp.reshape(Mask_Map.grid.shape[0], Mask_Map.grid.shape[1])
        occupancy_Map.updateGrid(tmp)
        print("Occupancy Map Updated")
        # updating Mask_Map. right now it can even reset values (change between 0 and 1)
        # set mask to 1 where occupancy map is 0
        Mask_Map.grid[tmp == 0] = 1
        # set mask to 0 where cost map over threshold
        #print(Binary_Cost_Map.grid.dtype)
        Mask_Map.grid[Binary_Cost_Map.grid] = 0
        Mask_Map.last_update = time.time()
        # print(len(Mask_Map.grid[Mask_Map.grid == 0]))
        # print(len(Mask_Map.grid[Mask_Map.grid == 1]))

        #Set cells outide of the record maps to -1
        padStart = Binary_Cost_Map.xy_to_index(DwellTime_Map.origin_x, DwellTime_Map.origin_y)[0] + 5
        # padStart = (len(Binary_Cost_Map.grid) - len(DwellTime_Map.grid))/2 + 5
        padEnd = len(DwellTime_Map.grid) + padStart - 5
        Mask_Map.grid[0:padStart,:] = 0
        Mask_Map.grid[:,0:padStart] = 0
        Mask_Map.grid[padEnd:len(Mask_Map.grid),:] = 0
        Mask_Map.grid[:,padEnd:len(Mask_Map.grid)] = 0
    else:
        rospy.loginfo("Problem: Map and Mask_Map out of shape")
    #rospy.loginfo("Map callback ended (after {:.3f})".format(time.time() - last_map_update_time)) 

def MappingMain1():
    global DwellTime_Map
    global CPS_Map
    global Count_Map
    global Binary_Cost_Map
    global Mask_Map
    global Cost_Map
    global Cost_Samples_Map
    global trans
    global listener
    global RecordMapParameters
    global GPMapParameters
    global occupancy_Map

    global last_cost_update_time


    #Decision-making parameters
    Qthreshold = 1.6
    targetPosition = [0,0,0]

    #Set up CPS maps
    CPS_Map = RadiationMap.Map(*RecordMapParameters, dtype = 'float32')
    GP_Map = RadiationMap.GP_Map(*RecordMapParameters)
    Binary_Cost_Map = RadiationMap.Map(*MaskMapParameters, dtype = bool)
    Q_Map = RadiationMap.Map(*RecordMapParameters)
    Q2_Map = RadiationMap.Map(*RecordMapParameters)
    #Binary_Cost_Map.grid = np.full((384, 384), False, dtype = bool)
    Mask_Map = RadiationMap.Map(*MaskMapParameters, dtype = bool)
    occupancy_Map = RadiationMap.Map(*MaskMapParameters, dtype = bool)
    Cost_Map = RadiationMap.Map(*MaskMapParameters, dtype = 'int32')
    Cost_Samples_Map = RadiationMap.Map(*MaskMapParameters, dtype = 'int32')
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
    rospy.Subscriber('/bloodhound/dwelltimeupdate', Point, callback_dwelltimeupdate)
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, callback_costmap)
    rospy.Subscriber('/map', OccupancyGrid, callback_map)
    
    #Setup publisher
    GoalSimpletoPublish = PoseStamped()
    QuaternionPublisher = Quaternion()
    GoalSimplePublisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    GoalPublisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    GoalSimpletoPublish.header.frame_id = "map"


    GoaltoPublish = MoveBaseActionGoal()
    GoaltoPublish.goal.target_pose.header.frame_id = "map"

    ros_rate = 10
    last_gp_time = time.time()
    last_plot_time = time.time()
    last_target_time = time.time()
    last_save_time = time.time()
    last_dwell_time = time.time()
    rate = rospy.Rate(ros_rate) # 10hz

    while not rospy.is_shutdown():
        #Get transform to baselink
        try:
            (trans, rot) = listener.lookupTransform('map', '/base_scan',  rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #print("trans" +  repr(trans))


        run_name = 'run_30/'

        #ASYNCHRONOUS FUNCTIONS AND UPDATES
        #Only update map at a given rate (hz)
        gp_rate = 5; 
        if time.time() > last_gp_time + 1/gp_rate:
            #Update CPS
            #CPS_map.grid = Count_Map.grid / DwellTime_Map.grid

            #Update GP
            euler_rot = tf.transformations.euler_from_quaternion(rot)
            GP_Map.Calc_GP_Local(DwellTime_Map, CPS_Map, [trans[0], trans[1], np.degrees(euler_rot[2])])
            GP_Map.last_update = time.time()
            #print("Max of GP Map %f: " %(np.max(GP_Map.grid)))

            last_gp_time = time.time()
            #print("Max Grid Value: " + repr(CPS_Map.grid.max()))

            #Mask_Map.plot_map()

        plot_rate = 1; 
        if time.time() > last_plot_time + 1/plot_rate:
            #Update CPS
            #CPS_map.grid = Count_Map.grid / DwellTime_Map.grid

            
            #print("Max Grid Value: " + repr(CPS_Map.grid.max()))
            #DwellTime_Map.plot_map()

            #CPS_Map.plot_map()

            GP_Map.plot_map([trans[0], trans[1]], [0,5])
            print("GP var max: %f" %(np.max(GP_Map.grid2)))

            #Binary_Cost_Map.plot_map([trans[0], trans[1]], [0,5])

            #Q_Map.plot_map([trans[0], trans[1]], [-14,14])
            Q2_Map.plot_map([trans[0], trans[1]], [0,4])
            #Cost_Map.plot_map([trans[0], trans[1]], [0,100])

            Cost_Map.plot_map([trans[0], trans[1]], [0,100])

            #Mask_Map.plot_map()
            last_plot_time = time.time()


        #Choose target and publish
        target_rate = 5; 
        if time.time() > last_target_time + 1/target_rate:
            euler_rot = tf.transformations.euler_from_quaternion(rot)
            #[targetPosition, Q_orig, Q_binary] = TargetSelection([trans[0], trans[1], euler_rot[2]], GP_Map, Mask_Map, Qthreshold, [0,0])
            [targetPosition, Q_orig, Q_binary] = TargetSelection([trans[0], trans[1], euler_rot[2]], GP_Map, Mask_Map, Qthreshold)
            Q_Map.updateGrid(Q_orig)
            Q2_Map.updateGrid(Q_binary)

            last_target_time = time.time()
            #targetPosition = [0,0,0]

            #Publish target commands
            euler_rot = tf.transformations.euler_from_quaternion(rot)
            GoalQuaternion = tf.transformations.quaternion_from_euler(0,0,targetPosition[2])
            #print(targetPosition)


            GoalSimpletoPublish.pose.position.x = targetPosition[0]
            GoalSimpletoPublish.pose.position.y = targetPosition[1]
            GoalSimpletoPublish.pose.orientation.x = GoalQuaternion[0]
            GoalSimpletoPublish.pose.orientation.y = GoalQuaternion[1]
            GoalSimpletoPublish.pose.orientation.z = GoalQuaternion[2]
            GoalSimpletoPublish.pose.orientation.w = GoalQuaternion[3]
            

            GoaltoPublish.goal.target_pose.pose.position.x = targetPosition[0]
            GoaltoPublish.goal.target_pose.pose.position.y = targetPosition[1]
            GoaltoPublish.goal.target_pose.pose.orientation.x = GoalQuaternion[0]
            GoaltoPublish.goal.target_pose.pose.orientation.y = GoalQuaternion[1]
            GoaltoPublish.goal.target_pose.pose.orientation.z = GoalQuaternion[2]
            GoaltoPublish.goal.target_pose.pose.orientation.w = GoalQuaternion[3]

            if False: #Make true to save data

                np.save('/home/bloodhound/catkin_ws/src/radiation_mapping/src/' + 'Map_Data/' + run_name + 'Location_' + repr(time.time()), [trans[0], trans[1], euler_rot[2]])

            #GoaltoPublish.pose.orientation = tf.transformations.quaternion_from_euler(targetPosition[0], targetPosition[1], targetPosition[2])

            GoalSimplePublisher.publish(GoalSimpletoPublish)  #Disable for teleop control
            GoalPublisher.publish(GoaltoPublish)   #Disable for teleop control



        #Save data for later plotting
        
        save_rate = 1; 
        if time.time() > last_save_time + 1/save_rate:
            #Save all desired maps
            #Save Data
            if False: #Make true to save data
                np.save('Map_Data/' + run_name + 'GP_Map_' + repr(GP_Map.last_update), GP_Map.grid)
                np.save('Map_Data/' + run_name + 'GP_Map_var' + repr(GP_Map.last_update), GP_Map.grid2)
                np.save('Map_Data/' + run_name + 'Count_Map_' + repr(Count_Map.last_update), Count_Map.grid)
                np.save('Map_Data/' + run_name + 'DwellTime_Map_' + repr(DwellTime_Map.last_update), DwellTime_Map.grid)
                np.save('Map_Data/' + run_name + 'Q_Map_' + repr(Q_Map.last_update), Q_Map.grid)
                np.save('Map_Data/' + run_name + 'Occ_Map_' + repr(occupancy_Map.last_update), occupancy_Map.grid)


            last_save_time = time.time()




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

    StopPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    stop = twist()
    StopPublisher.publish(stop)


if __name__ == '__main__':
    try:
        MappingMain1()
    except rospy.ROSInterruptException:
        StopPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        stop = twist()
        StopPublisher.publish(stop)

"""ROS commands:
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=frontier_exploration 
rosrun radiation_mapping MappingMain1.py 
roslaunch turtlebot3_bringup turtlebot3_robot.launch
ssh bloodhound@192.168.207.14
rosrun rqt_tf_tree rqt_tf_tree
rosrun tf tf_echo /map /base_scan
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















#TODO:
#-Integrate decision making stuff, keep option of sim 
#-Map takes parameters as arguments
#-Change GP scaling

