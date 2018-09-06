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
main_rate = 10
rate = rospy.Rate(main_rate) # 10hz

#Publisher setup
DwellTimePub = rospy.Publisher('/bloodhound/dwelltimeupdate', Point, queue_size=10)

def PublishPoint(trans, lasttime):
    myPoint = Point()
    myPoint.x = trans[0]
    myPoint.y = trans[1]
    myPoint.z = time.time() - lasttime
    DwellTimePub.publish(myPoint)


def DwellTimeMapUpdates():
    while not rospy.is_shutdown():


        #Update DwellTime_Map each time through the loop
        try:
            (trans_gamma, rot_gamma) = listener.lookupTransform('map', '/gamma1_tf',  rospy.Time(0))
            PublishPoint(trans_gamma, time.time() - last_dwell_time)

            #print("Gamma 1 CPS Updated")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Gamma1 TF Exception")
            pass

        try:
            (trans_gamma, rot_gamma) = listener.lookupTransform('map', '/gamma2_tf',  rospy.Time(0))
            PublishPoint(trans_gamma, time.time() - last_dwell_time)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Gamma2 TF Exception")
            pass

        try:
            (trans_gamma, rot_gamma) = listener.lookupTransform('map', '/gamma3_tf',  rospy.Time(0))
            PublishPoint(trans_gamma, time.time() - last_dwell_time)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Gamma3 TF Exception")
            pass

        last_dwell_time = time.time()


        rate.sleep()

if __name__ == '__main__':
    try:
        DwellTimeMapUpdates()
    except rospy.ROSInterruptException:
        pass