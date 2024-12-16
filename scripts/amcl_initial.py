#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('amcl_initial_value')

amcl_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

initial_pose = PoseWithCovarianceStamped()

initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = rospy.Time.now()

initial_pose.pose.pose.position.x = 0.0
initial_pose.pose.pose.position.y = 0.0
initial_pose.pose.pose.orientation.w = 1.0

amcl_pose_pub.publish(initial_pose)

rospy.sleep(1.0)

amcl_pose_pub.unregister()