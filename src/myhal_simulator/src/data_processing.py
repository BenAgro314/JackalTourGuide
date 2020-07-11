#!/usr/bin/env python

import sys
sys.path.append('./include')

import os
import json
import matplotlib
import numpy as np
import time as RealTime

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

	print "Reading lidar frames"

	# read in lidar frames
	frames = bt.read_pointcloud_frames("/velodyne_points", bag)

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

	print "Reading poses"

	# read in ground truth pose
	gt_pose = bt.read_nav_odometry("/ground_truth/state",bag)

	optimal_traj = bt.read_nav_odometry("/optimal_path",bag, False)


	# output grond truth pose to .ply file
	el = ply.PlyElement.describe(bt.trajectory_to_array(gt_pose), "trajectory")
	ply.PlyData([el]).write(path + "/gt_pose.ply")

	odom_to_base = bt.read_tf_transform("odom","base_link", bag)
	map_to_odom = bt.read_tf_transform("map","odom", bag)

	odom_to_base = bt.transforms_to_trajectory(odom_to_base)
	#map_to_odom = bt.transforms_to_trajectory(map_to_odom)

	res = mu.get_interpolations(odom_to_base, map_to_odom)


	transformed = mu.transform_trajectory(odom_to_base, map_to_odom)

	pu.plot_trajectory(gt_pose)
	pu.plot_trajectory(transformed)
	#pu.show()

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

