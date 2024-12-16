#!/usr/bin/env python3

import numpy as np
import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

sys.path = [p for p in sys.path if "python2.7" not in p]

import tf2_ros

rospy.init_node('amcl_initial_value')

amcl_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

initial_pose = PoseWithCovarianceStamped()

initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = rospy.Time.now()

initial_pose.pose.pose.position.x = -2.5
initial_pose.pose.pose.position.y = 0.0

r = 0.0
p = 0.0
yaw = 0.0
q = quaternion_from_euler(r, p, yaw)
initial_pose.pose.pose.orientation.x = q[0]
initial_pose.pose.pose.orientation.y = q[1]
initial_pose.pose.pose.orientation.z = q[2]
initial_pose.pose.pose.orientation.w = q[3]

while not rospy.is_shutdown():
    amcl_pose_pub.publish(initial_pose)
    rospy.sleep(1.0)

amcl_pose_pub.unregister()
