#!/usr/bin/env python3
import numpy as np
import triad_openvr
import pyUtilities as uty

# v = triad_openvr.triad_openvr()
v = triad_openvr.triad_openvr("/home/lab816/localization_ws/src/sdc/scripts/config.json")
v.print_discovered_objects()

poseList = uty.matrix2List(v.devices["tracker_1"].get_pose_matrix())
#print(type(pose))
#print(pose)
pose = np.array(poseList)
#pose = np.array([[0,1,0,1],[1,0,0,2],[0,0,1,3]]) # test transformationInverse()
#print(type(pose))
#print(pose.shape)
print("origin set:")
print(pose)

# poseInv_ = uty.transformationInverse(pose)
poseInv = np.identity(4)
poseInv[:3,:] = pose
poseInv = np.linalg.inv(poseInv)
print("T0 inverse:")
print(poseInv)
# print("T0 inverse_:")
# print(poseInv_)


fd = open("/home/lab816/high_precision_amr/controller/my_amm_demo/py/T0inv.txt","w")
# fd = open("/home/lab816/localization_ws/src/sdc/scripts/T0inv.txt","w")
txt = ""
for row in poseInv:
	for each in row:
		txt += "%lf" % each
		txt += " "
# print(txt)
fd.write(txt)
fd.close()

#np.savetxt('originInv.txt',poseInv.reshape(1,12),delimiter=' ')

""" ##### tested reading method #####
fd = open("calibrationOrigin.txt","r")
poseInvStr = fd.read()
poseInvRe = np.fromstring(poseInvStr, dtype=float, sep=' ')
print(poseInvRe.reshape(3,4))
fd.close()
"""
