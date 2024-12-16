#!/usr/bin/env python3

import rospy
import sys
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovariance, Twist, PoseWithCovarianceStamped
import struct
import sysv_ipc as ipc
import numpy as np
import tf
import geometry_msgs
from tf.transformations import quaternion_from_euler

sys.path = [p for p in sys.path if "python2.7" not in p]

import tf2_ros

shm_ros = ipc.SharedMemory(888, ipc.IPC_CREAT, size = ipc.PAGE_SIZE)
sem_ros = ipc.Semaphore(999, ipc.IPC_CREAT)      # sem initial value is 0.
sem_ros.V()                                      # increase sem value to 1.

class OdomTransformNode:
    def __init__(self):
        rospy.init_node('odom_transform_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # self.odom_sub = rospy.Subscriber('t265/odom/sample', Odometry, self.odom_callback)
        # self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

        self.pose_sub = rospy.Subscriber('tracked_pose', PoseStamped, self.pose_callback)
        # self.pose_pub = rospy.Publisher('odom', Odometry, queue_size=10)

        print('start odom_transform_node')
    
    def pose_callback(self, msg):
        try:
            quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,msg.pose.orientation.w)
            (r, p, yaw) = tf.transformations.euler_from_quaternion(quaternion)
            pose = [msg.pose.position.x, msg.pose.position.y, yaw]

            print(pose)

            valid = (pose[0] != 0) and (pose[1] != 0)

            print(valid)

            if valid:
                BytesBuf = struct.pack("?",True)
                for each in pose:
                    BytesBuf += struct.pack("d",each)
            else:
                BytesBuf = struct.pack("?",False)
                print("  fail to get carto.")

            sem_ros.P()
            shm_ros.write(BytesBuf, offset=0)
            sem_ros.V()
            
            print("\r sharing carto data...") ###
            # transform = self.tf_buffer.lookup_transform('odom_frame','t265_odom_frame',rospy.Time(0), rospy.Duration(1.0))
            # print('transform')
            # print(transform)

            # odom_msg = Odometry()
            # odom_msg.header = 'odom_frame'
            # odom_msg.child_frame_id = 't265_odom_frame'

            # # Transform the pose
            # transformed_pose = self.transform_pose(msg.pose.pose, transform)
            # odom_msg.pose.pose = transformed_pose

            # # Transform the twist
            # transformed_twist = self.transform_twist(msg.twist.twist, transform)
            # odom_msg.twist.twist = transformed_twist

            # self.odom_pub.publish(odom_msg)
            # print("t265/odom")
            # print(msg)
            # print("odom")
            # print(odom_msg)

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('Failed to transform odom: {}'.format(e))

    def transform_pose(self, pose, transform):
        transformed_pose = PoseWithCovariance()

        # Apply translation transform
        transformed_pose.pose.position.x = transform.transform.translation.x + pose.position.x
        transformed_pose.pose.position.y = transform.transform.translation.y + pose.position.y
        transformed_pose.pose.position.z = transform.transform.translation.z + pose.position.z
        
        # Apply rotation transform
        transformed_pose.pose.orientation = transform.transform.rotation
        
        return transformed_pose
    
    def transform_twist(self, twist, transform):
        transformed_twist = Twist()
        
        # Apply translation transform
        transformed_twist.linear.x = twist.linear.x
        transformed_twist.linear.y = twist.linear.y
        transformed_twist.linear.z = twist.linear.z
        
        # Apply rotation transform
        transformed_twist.angular.x = twist.angular.x
        transformed_twist.angular.y = twist.angular.y
        transformed_twist.angular.z = twist.angular.z
        
        return transformed_twist

    def odom_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform('odom_frame','t265_odom_frame',rospy.Time(0), rospy.Duration(1.0))
            # print('transform')
            # print(transform)

            odom_msg = Odometry()
            odom_msg.header = 'odom_frame'
            odom_msg.child_frame_id = 't265_odom_frame'

            # Transform the pose
            transformed_pose = self.transform_pose(msg.pose.pose, transform)
            odom_msg.pose.pose = transformed_pose

            # Transform the twist
            transformed_twist = self.transform_twist(msg.twist.twist, transform)
            odom_msg.twist.twist = transformed_twist

            self.odom_pub.publish(odom_msg)
            # print("t265/odom")
            # print(msg)
            # print("odom")
            # print(odom_msg)

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('Failed to transform odom: {}'.format(e))

    def run(self):
        rate = rospy.Rate(200) # 200 Hz

        while not rospy.is_shutdown():
            rospy.spin()
            rate.sleep()

if __name__ == '__main__':
    node = OdomTransformNode()
    node.run()       