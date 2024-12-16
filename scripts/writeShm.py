import triad_openvr
import time
import struct
import sysv_ipc as ipc
import openvr
import numpy as np
from triad_openvr import vr_tracking_reference

interval = 0.01

#global cnt, sum_j
#cnt = 0 ###
#sum_j = 0 ###

shm = ipc.SharedMemory(666, ipc.IPC_CREAT, size = ipc.PAGE_SIZE)
sem = ipc.Semaphore(777, ipc.IPC_CREAT)      # sem initial value is 0.
sem.V()                                      # increase sem value to 1.

v = triad_openvr.triad_openvr()
# v = triad_openvr.triad_openvr("/home/lab816/agv_ws/src/sdc/scripts/config.json")
v.print_discovered_objects()

running = ['\\', '/', '-']
i = 0

pose = v.devices["tracker_1"].get_pose_matrix()
linVel = v.devices["tracker_1"].get_velocity()
angVel = v.devices["tracker_1"].get_angular_velocity()

def checkJitter(start, oldStart):
	global sum_j, cnt
	d = (start-oldStart)*1000
	j = d-interval*1000
	if cnt>0: ###
		sum_j = sum_j + j ###
		avg_j = sum_j/cnt ###
	if cnt%int(1/interval)==1: ###
		print("\rduration: {0:2.4f} average jitter: ".format(d) + str(avg_j)) ###



##########################################################################################
##                                        main loop                                     ##
##########################################################################################
oldStart = time.perf_counter() ###
while(True):
	# v = triad_openvr.triad_openvr()
	# v = triad_openvr.triad_openvr("/home/lab816/agv_ws/src/sdc/scripts/config.json")
	start = time.perf_counter()

	# print(v.devices["tracking_reference_1"].get_mode())
	# print(v.devices["tracker_1"].get_pose_matrix())

	# print("\nreference {}".format(len(v.object_names["Tracking Reference"])))

	if len(v.object_names["Tracking Reference"]) > 1:
		pose = v.devices["tracker_1"].get_pose_matrix()
		linVel = v.devices["tracker_1"].get_velocity()
		angVel = v.devices["tracker_1"].get_angular_velocity()
	else:
		print("1 tracking reference")
		pose = openvr.HmdMatrix34_t()
		linVel = openvr.HmdVector3_t()
		angVel = openvr.HmdVector3_t()
	# print("pose {}".format(pose))
	# print("linVel {}".format(linVel))
	# print("angVel {}".format(angVel))

	valid = isinstance(pose, openvr.HmdMatrix34_t) and isinstance(linVel, openvr.HmdVector3_t) and isinstance(angVel, openvr.HmdVector3_t)

	### old method ###
	"""
	if valid:
		txt = "1 "
		for row in pose:
			for each in row:
				txt += "%.4f" % each
				txt += " "
		for each in linVel:
			txt += "%.4f" % each
			txt += " "
		for each in angVel:
			txt += "%.4f" % each
			txt += " "
		txt += "\0"
	else:
		txt = "0 "
	"""
	
	if valid:
		BytesBuf = struct.pack("?",True)
		for row in pose:
			for each in row:
				BytesBuf += struct.pack("d",each)
		for each in linVel:
			BytesBuf += struct.pack("d",each)
		for each in angVel:
			BytesBuf += struct.pack("d",each)
	else:
		BytesBuf = struct.pack("?",False)
		print("  fail to get tracker.")
	
	sem.P()
	shm.write(BytesBuf, offset=0)
	sem.V()

	#checkJitter(start, oldStart) ###

	print("\r sharing tracker data..."+running[int(i*interval)]+"   valid:"+str(valid), end="") ###
	# v.print_discovered_objects()
	
	#oldStart = start ###
	#cnt = cnt+1 ###

	i = i+1
	if i == 3/interval:
		i = 0
	#if i%100==1:
	#	print(linVel)
	
	sleepTime = interval - (time.perf_counter()-start) - 0.000165
	if sleepTime > 0:
		time.sleep(sleepTime)
	else:
		print("\nlose a loop!")





