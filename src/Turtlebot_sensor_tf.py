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


if __name__ == '__main__':
    rospy.init_node('Turtlebot_sensor_tf')
    br1 = tf.TransformBroadcaster()
    br2 = tf.TransformBroadcaster()
    br3 = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        br1.sendTransform((0.09, 0.0, 0.0),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),"gamma1_tf","/base_link")
        br2.sendTransform((-0.045, 0.078, 0.0),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),"gamma2_tf","/base_link")
        br3.sendTransform((-0.045, -0.078, 0.0),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),"gamma3_tf","/base_link")
        rate.sleep()