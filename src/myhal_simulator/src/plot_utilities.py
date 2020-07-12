#!/usr/bin/env python

import matplotlib.pyplot as plt

import sys
import os
import numpy as np
import pickle

import bag_tools as bt
import math_utilities as mu
import plot_utilities as pu


def plot_trajectory(traj):
	plt.plot(traj['pos_x'],traj['pos_y'])


def plot_translation_error(traj, base_traj):
	x_error = traj['pos_x'] - base_traj['pos_x']
	y_error = traj['pos_y'] - base_traj['pos_y']
	error = np.sqrt(x_error*x_error + y_error*y_error)
	
	plt.plot(mu.compute_distances(base_traj)[0] , error)

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
		pickle_data = pickle.load(handle)

	keys = []
	for key in pickle_data:
		keys.append(key)





	if ('amcl_traj' in keys):
		loc_name = 'amcl_traj'
	if ('gmapping_traj' in keys):
		loc_name = 'gmapping_traj'


	plot_translation_error(pickle_data[loc_name], pickle_data['gt_traj'])
	
	plt.show()
	