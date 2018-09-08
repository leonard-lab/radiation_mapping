#!/usr/bin/env python

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
#Define Classes 
import RadiationMap
import RadiationSource

#General parameters
global DwellTimePub





def SimulationRadation(current_location, sources, delta_t):
    rad = 0

    for source in sources:
        rad += source.GenerateRandomRadiation(current_location, delta_t)

    return rad




def RadiationSimulator():


    rospy.init_node('RadiationSimulator', anonymous = True)

    #Publisher setup
    DwellTimePub = []
    DwellTimePub.append(rospy.Publisher('/gamma1', UInt32, queue_size=10))
    DwellTimePub.append(rospy.Publisher('/gamma2', UInt32, queue_size=10))
    DwellTimePub.append(rospy.Publisher('/gamma3', UInt32, queue_size=10))
    myRad = UInt32(1)

    #Listener setup
    listener = tf.TransformListener()

    sensorNames = ['/gamma1_tf','/gamma2_tf','/gamma3_tf']
    sensorTimes = [time.time(), time.time(), time.time()]


    #Radiation source setup
    Radiation_Sources = []
    Radiation_Sources.append(RadiationSource.RadiationSource(2,[0,0],background_flag=True))
    Radiation_Sources.append(RadiationSource.RadiationSource(500,[0.75,0.75]))

    #Control loop rate
    main_rate = 10
    rate = rospy.Rate(main_rate) # 10hz

    while not rospy.is_shutdown():

        #Simulate Radiation 
 
        #Update DwellTime_Map each time through the loop
        for i in range(0,len(sensorNames)):
            try:
                (trans_gamma, rot_gamma) = listener.lookupTransform('map', sensorNames[i],  rospy.Time(0))
                #Update simulation data for each point
                Sim_Rad = SimulationRadation([trans_gamma[0],trans_gamma[1]], Radiation_Sources,  time.time() - sensorTimes[i])
                sensorTimes[i] = time.time()

                #Publish radiation 
                for rad_i in range(0,Sim_Rad):
                    DwellTimePub[i].publish(myRad)
                    time.sleep(0.0001)

                #print("Gamma 1 CPS Updated")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print(sensorNames[i] + " TF Exception in Radiation Simulator")
                pass




if __name__ == '__main__':
    try:
        RadiationSimulator()
    except rospy.ROSInterruptException:
        pass