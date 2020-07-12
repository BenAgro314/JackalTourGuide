#!/usr/bin/env python

import sys
import os
import numpy as np
import time as RealTime
import pickle

import rosbag
import plyfile as ply
import bag_tools as bt
import math_utilities as mu
import plot_utilities as pu


if __name__ == "__main__":


	start_time = RealTime.time()
	
	username = os.environ['USER']
	if ((len(sys.argv)-1) == 0):
		print "ERROR: must input filename"
		exit()
	filename = sys.argv[1]
	print "Processing data for file", filename

	path = "/home/" + username + "/Myhal_Simulation/simulated_runs/" + filename + "/"
	logs_path = path + "logs-" + filename + "/"


	try:
		bag = rosbag.Bag(path + "raw_data.bag")
	except:
		print "ERROR: invalid filename"
		exit()

	if (not os.path.isdir(path + "classified_frames")):
		try:
			os.mkdir(path + "classified_frames")
		except OSError:
			print ("Creation of the classifed_frames directory failed")
			exit()

	
	pickle_dict = {}

	print "Reading lidar frames"

	# read in lidar frames
	frames = bt.read_pointcloud_frames("/velodyne_points", bag)

	pickle_dict['lidar_frames'] = frames

	print "Writing lidar frames"

	# write lidar frames to .ply files 
	for frame in frames:
		time, points = frame
		dtype = [("x", np.float32), ("y",np.float32), ("z",np.float32), ("cat", np.int32)]
		arr = np.array([points['x'], points['y'], points['z'], points['intensity']])
		arr = np.core.records.fromarrays(arr, dtype = dtype)
		el = ply.PlyElement.describe(arr, "vertex")
		time = time.to_sec()
		time = "{:.6f}".format(time)
		
		ply.PlyData([el]).write(path + "classified_frames/" + time + ".ply");

	print "Reading trajectories"

	# read in ground truth pose
	gt_traj = bt.read_nav_odometry("/ground_truth/state",bag)
	
	#read in optimal traj
	optimal_traj = bt.read_nav_odometry("/optimal_path",bag, False)
	pickle_dict['optimal_traj'] = bt.trajectory_to_array(optimal_traj)

	#read in tour waypoints
	waypoints = bt.read_nav_odometry("/tour_data", bag, False)
	pickle_dict['waypoints'] = bt.trajectory_to_array(waypoints)

	#read in move_base results
	results = bt.read_action_result('/move_base/result', bag)
	pickle_dict['action_results'] = results

	# output ground truth pose to .ply file
	el = ply.PlyElement.describe(bt.trajectory_to_array(gt_traj), "trajectory")
	ply.PlyData([el]).write(path + "/gt_pose.ply")

	# read in amcl poses if they exist
	amcl_status = bool(bt.num_messages("/amcl_pose", bag))

	
	odom_to_base = bt.read_tf_transform("odom","base_link", bag)
	map_to_odom = bt.read_tf_transform("map","odom", bag)
	odom_to_base = bt.transforms_to_trajectory(odom_to_base)
	tf_traj = mu.transform_trajectory(odom_to_base, map_to_odom)

	# interplote gt_traj to the times of tf_traj
	
	gt_traj = mu.get_interpolations(tf_traj, gt_traj, False)
	
	pickle_dict['gt_traj'] = bt.trajectory_to_array(gt_traj)
	
	if (amcl_status):
		print "Saving amcl_traj"
		pickle_dict['amcl_traj'] = bt.trajectory_to_array(tf_traj)
		
	else:
		print "Saving gmapping_traj"
		pickle_dict['gmapping_traj'] = bt.trajectory_to_array(tf_traj)


	# pu.plot_trajectory(gt_traj)
	# pu.plot_trajectory(tf_traj)
	# pu.show()

	print "Dumping data to", logs_path + "processed_data.pickle"
	with open(logs_path + 'processed_data.pickle', 'wb') as handle:
		pickle.dump(pickle_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

	'''
	read in:
		- gt_pose
		- amcl_pose
		- gmapping_pose
		- hughes_pose

		- path data (TODO: publish path data)


	output:
		- raw trajectory (to json)
		- interpolated pose differences (to json + matplotlib + pickle)
		- path differences 
	'''

	bag.close()

	duration = RealTime.time() - start_time

	print "Data processed in", "{:.2f}".format(duration) ,"seconds"

