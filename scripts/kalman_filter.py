#!/usr/bin/env python3

import numpy as np
import struct
import rospy
import math
import sys
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import actionlib
from tf.transformations import euler_from_quaternion
import tf.transformations as tft
from rospy.rostime import Time

# print(sys.version)
# print(sys.path)
sys.path = [p for p in sys.path if "python2.7" not in p]
# print(sys.path)
# from shm import Segment
import sysv_ipc as ipc

import tf2_ros



interval = 0.01
running = ['\\', '/', '-']
i = 0

shm_ros = ipc.SharedMemory(888, ipc.IPC_CREAT, size = ipc.PAGE_SIZE)
sem_ros = ipc.Semaphore(999, ipc.IPC_CREAT)      # sem initial value is 0.
sem_ros.V()                                      # increase sem value to 1.

listen = [0, 0, 0, 0]
dt = 0.01  
noise_cov = 35.0

# oldStart = time.perf_counter() ###

class KalmanFilter:
    def __init__(self, dt, noise_cov):
        self.subscriber = rospy.Subscriber("freq_pose", PoseWithCovarianceStamped, self.amcl_callback)
        self.subscriber = rospy.Subscriber("t265/imu", Imu, self.imu_callback)

        self.amcl_flag = 0
        self.measurement = []
        self.imu = 0

        # state transition matrix
        # self.A = np.array([[1, 0, 0, dt, 0, 0],
        #                    [0, 1, 0, 0, dt, 0],
        #                    [0, 0, 1, 0, 0, dt],
        #                    [0, 0, 0, 1, 0, 0],
        #                    [0, 0, 0, 0, 1, 0],
        #                    [0, 0, 0, 0, 0, 1]])
        self.A = np.array([[1, 0, 0, 0, dt, 0],
                           [0, 1, 0, 0, 0, dt],
                           [0, 0, 1, dt, 0, 0],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])
        # observation matrix
        # self.H = np.array([[1, 0, 0, 0, 0, 0],
        #                    [0, 1, 0, 0, 0, 0],
        #                    [0, 0, 1, 0, 0, 0]])
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0]])
        
        # covariance of state vector estimate
        self.P = np.eye(6)
        
        # process noise covariance
        self.Q = noise_cov * np.eye(6)
        
       # measurement noise covariance
        # self.R = np.array([[0.01, 0, 0],
        #                    [0, 0.08, 0],
        #                    [0, 0, 0.017]])
        self.R = np.array([[0.01, 0, 0, 0],
                           [0, 0.08, 0, 0],
                           [0, 0, 0.017, 0],
                           [0, 0, 0, 0.001]])

    def predict(self):
        self.x = np.dot(self.A, self.x)
        # self.x = np.dot(slef.A, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
    
    # def update(self, z):
    def update(self):
        y = self.measurement - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        Ky = np.dot(K, y)
        # if Ky[0] > 0.06:
        #     Ky[0] = 0.06
        # elif Ky[0] < -0.06:
        #     Ky[0] = -0.06


        # if Ky[1] > 0.005:
        #     Ky[1] = 0.005
        # elif Ky[1] < -0.005:
        #     Ky[1] = -0.005
        self.x = self.x + Ky
        self.P = np.dot((np.eye(6) - np.dot(K, self.H)), self.P)
        
    def init_filter(self, initial_state):
        self.x = initial_state
        
    def get_state(self):
        return self.x

    def get_amcl_flag(self):
        return self.amcl_flag
    
    def get_measurement(self):
        return self.measurement
    
    def get_imu(self):
        return self.imu

    def set_amcl_flag(self, value):
        self.amcl_flag = value

    def imu_callback(self, msg):

        # self.imu[0] = Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_nsec()
        self.imu = msg.angular_velocity.y

        # rospy.logwarn('Imu: %lf', self.imu)

    def amcl_callback(self, msg):

        listen[0] = Time(msg.header.stamp.secs, msg.header.stamp.nsecs).to_nsec()
        listen[1] = msg.pose.pose.position.x
        listen[2] = msg.pose.pose.position.y
        a_q = msg.pose.pose.orientation
        (roll,pitch,yaw) = euler_from_quaternion([a_q.x,a_q.y,a_q.z,a_q.w])
        listen[3] = yaw
        listen[4] = self.get_imu

        # self.measurement = listen[1:4]
        self.measurement = listen[1:5]
        rospy.logwarn('Measurement: %lf, %lf, %lf', self.measurement[0], self.measurement[1], self.measurement[2])

        # kalman_filter.update(self.measurement)

        self.amcl_flag = 1
    
        # listen[4] = msg.twist.twist.linear.x
        # listen[5] = msg.twist.twist.linear.y
        # listen[6] = msg.twist.twist.angular.z

def checkJitter(start, oldStart):
	global sum_j, cnt
	d = (start-oldStart)*1000
	j = d-interval*1000
	if cnt>0: ###
		sum_j = sum_j + j ###
		avg_j = sum_j/cnt ###
	if cnt%int(1/interval)==1: ###
		print("\rduration: {0:2.4f} average jitter: ".format(d) + str(avg_j)) ###

if __name__ == "__main__":

    rospy.init_node('tf2_listener')

    # tfBuffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tfBuffer)

    kalman_filter = KalmanFilter(dt, noise_cov)

    # (x,y,yaw,Vx,Vy,Vyaw)
    initial_state = np.array([0, 0, 0, 0, 0, 0])  
    kalman_filter.init_filter(initial_state)

    # rospy.Subscriber("amcl_ekf", Odometry, amcl_callback)
    # rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_callback)

    # ros_time = rospy.Time.from_sec(0.005)

    # while(True):
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        # start = time.perf_counter()
        try:
            # transform = tfBuffer.lookup_transform("map", 'base_link', rospy.Time())
            # print(tf)
            # print(transform)

            # q = transform.transform.rotation
            # (r,p,y) = euler_from_quaternion([q.x,q.y,q.z,q.w])
            # time = transform.header.stamp.to_sec()
            # listen_pose = [time, transform.transform.translation.x, transform.transform.translation.y, y]
            # print(time)
            # print("tf")
            # print(listen_pose)
            # print("ekf")
            # print(listen)

            # valid = listen[1] and listen[2]
            # valid = listen_ekf[1] and listen_ekf[2]

            if kalman_filter.get_amcl_flag() == 1:
                kalman_filter.update()
                kalman_filter.set_amcl_flag(0)
            
            kalman_filter.predict()

            filtered_state = kalman_filter.get_state()
            imu = kalman_filter.get_imu()

            # print("predict: ",filtered_state)
            # print("amcl_flag: ", kalman_filter.get_amcl_flag())

            valid = filtered_state[0] and filtered_state[1]


            if valid:
                BytesBuf = struct.pack("?",True)
                # BytesBuf += struct.pack("d",listen[0])
                for each in filtered_state:
                    BytesBuf += struct.pack("d",each)
                BytesBuf += struct.pack("d",imu)
                # for each in listen_ekf:
                #     BytesBuf += struct.pack("d",each)
                rospy.loginfo("Filtered State: %lf, %lf, %lf", filtered_state[0], filtered_state[1], filtered_state[2])
                # print("Filtered State:", filtered_state)  # 输出滤波后的状态 (x, y, yaw)
            else:
                BytesBuf = struct.pack("?",False)
                rospy.loginfo(" fail to get amcl.")
                # print("  fail to get amcl.")

            sem_ros.P()
            shm_ros.write(BytesBuf, offset=0)
            sem_ros.V()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        
        rate.sleep()

        # print("\r sharing amcl data..."+running[int(i*interval)]+"   valid:"+str(valid), end="\n") ###

        # i = i+1
        # if i == 3/interval:
        #     i = 0

        # sleepTime = interval - (time.perf_counter()-start) - 0.000165
            
        # if sleepTime > 0:
        #     time.sleep(sleepTime)
        # else:
        #     print("\nlose a loop!")
