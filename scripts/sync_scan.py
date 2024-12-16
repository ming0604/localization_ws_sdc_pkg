#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import LaserScan

def callback(front_scan, back_scan):
  # Solve all of perception here...

if __name__ == '__main__':
    rospy.init_node('sync_scan', anonymous=False)
    
    laser_sub_1 = message_filters.Subscriber('front_scan', LaserScan)
    laser_sub_2 = message_filters.Subscriber('back_scan', LaserScan)

    ts = message_filters.TimeSynchronizer([laser_sub_1, laser_sub_2], 10)
    ts.registerCallback(callback)
    rospy.spin()