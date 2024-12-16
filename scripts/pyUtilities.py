import numpy as np

def matrix2List(matrixObj):
	pose = [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
	for i in range(3):
		for j in range(4):
			pose[i][j] = matrixObj[i][j]
	return pose

def velocity2List(velObj):
	vel = [0.0,0.0,0.0]
	for i in range(3):
		vel[i] = velObj[i]
	return vel

def transformationInverse(pose):
	inv = np.zeros((3,4))
	for i in range(3):
		for j in range(3):
			inv[i,j] = pose[j,i]
	for i in range(3):
		inv[i,3] = -np.sum(pose[:3,3]*pose[:3,i])
	return inv
