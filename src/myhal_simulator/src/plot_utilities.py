#!/usr/bin/env python

import matplotlib.pyplot as plt

import sys
import os
import numpy as np
import pickle

import bag_tools as bt
import math_utilities as mu
import plot_utilities as pu
from scipy.spatial.transform import Rotation as R

def list_distances(traj):
	dist_list = [0]
	dist = 0
	
	prev = traj[0]

	for pose in traj[1:]:
		dx = pose['pos_x'] - prev['pos_x']
		dy = pose['pos_y'] - prev['pos_y']

		dist += np.sqrt(dx**2 + dy**2)

		dist_list.append(dist)

		prev = pose

	return (dist_list,dist)

def translation_error(traj, base_traj):
	x_error = traj['pos_x'] - base_traj['pos_x']
	y_error = traj['pos_y'] - base_traj['pos_y']
	error = np.sqrt(x_error*x_error + y_error*y_error)
	return error
	#plt.plot(mu.compute_distances(base_traj)[0] , error)

def rotation_error(traj, base_traj):
	error = []

	for i in range(len(traj)):
		r1 = R.from_quat([traj[i]['rot_x'], traj[i]['rot_y'], traj[i]['rot_z'], traj[i]['rot_w']])
		r2 = R.from_quat([base_traj[i]['rot_x'], base_traj[i]['rot_y'], base_traj[i]['rot_z'], base_traj[i]['rot_w']])
		error.append(np.abs(r1.as_euler('xyz')[2] - r2.as_euler('xyz')[2]))

	return error

if __name__ == "__main__":

	username = os.environ['USER']
	if ((len(sys.argv)-1) == 0):
		print "ERROR: must input filename"
		exit()
	filename = sys.argv[1] 
	print "Processing data for file", filename

	path = "/home/" + username + "/Myhal_Simulation/simulated_runs/" + filename + "/"
	logs_path = path + "logs-" + filename + "/"

	with open(logs_path + 'processed_data.pickle', 'rb') as handle:
		data = pickle.load(handle)

	keys = []
	for key in data:
		keys.append(key)

	gt_traj = data['gt_traj']

	if ('amcl_traj' in keys):
		loc_name = 'amcl_traj'
	if ('gmapping_traj' in keys):
		loc_name = 'gmapping_traj'

	plt.plot(list_distances(gt_traj)[0], rotation_error(data[loc_name],gt_traj))

	# plot_translation_error(data[loc_name], data['gt_traj'])
	
	plt.show()
	