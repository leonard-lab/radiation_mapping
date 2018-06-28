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

#Global counter variables
global DwellTimeMap #Dwell time at each map voxel in seconds
global CountMap #Number of radiation coutns accumulated at each map voxel
DwellTimeMap = 0
CountMap = 0

def callback(data):
    global DwellTimeMap
    global CountMap
    rospy.loginfo(rospy.get_caller_id() + 'I heard detector 1 at %s', data.data)
    CountMap += 1

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('MappingMain1', anonymous=True)

    rospy.Subscriber('/gamma1', String, callback)

    ros_rate = 10
    rate = rospy.Rate(ros_rate) # 10hz
    while not rospy.is_shutdown():
        #Update DwellTimeMap each time through the loop
        DwellTimeMap += 1.0/float(ros_rate)

        rate.sleep()

if __name__ == '__main__':
    MappingMain1()
except rospy.ROSInterruptException: pass
















