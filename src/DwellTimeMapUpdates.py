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

#General parameters
global DwellTimePub



def PublishPoint(trans, lasttime):
    global DwellTimePub
    myPoint = Point()
    myPoint.x = trans[0]
    myPoint.y = trans[1]
    myPoint.z = time.time() - lasttime
    print("Data from within DwellTimeUpdates: %f" %(myPoint.z))
    DwellTimePub.publish(myPoint)


def DwellTimeMapUpdates():
    global DwellTimePub

    rospy.init_node('DwellTimeMapUpdates', anonymous = True)

    #Publisher setup
    DwellTimePub = rospy.Publisher('/bloodhound/dwelltimeupdate', Point, queue_size=10)

    #Listener setup
    listener = tf.TransformListener()

    #Control loop rate
    main_rate = 10
    rate = rospy.Rate(main_rate) # 10hz

    while not rospy.is_shutdown():

        sensorNames = ['/gamma1_tf','/gamma2_tf','/gamma3_tf']
        #Update DwellTime_Map each time through the loop
        for name in sensorNames:
            try:
                (trans_gamma, rot_gamma) = listener.lookupTransform('map', name,  rospy.Time(0))
                PublishPoint(trans_gamma, last_dwell_time)

                #print("Gamma 1 CPS Updated")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print(name + " TF Exception")
                pass


        last_dwell_time = time.time()

        rate.sleep()

if __name__ == '__main__':
    try:
        DwellTimeMapUpdates()
    except rospy.ROSInterruptException:
        pass