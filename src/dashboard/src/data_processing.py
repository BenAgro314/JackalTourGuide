#!/usr/bin/env python

import sys
import os
import numpy as np
import time as RealTime
import pickle
import json
import subprocess
import rosbag
import plyfile as ply
import shutil
from utilities import bag_tools as bt
from utilities import math_utilities as mu
from utilities import plot_utilities as pu


if __name__ == "__main__":


    start_time = RealTime.time()
    
    username = os.environ['USER']
    if ((len(sys.argv)-1) == 0):
        print "ERROR: must input filename"
        exit()
    filename = sys.argv[1]
    print "Processing data for file", filename

    path = "/home/" + username + "/Myhal_Simulation/simulated_runs/" + filename + "/"
    if (not os.path.isdir(path)):
        print 'File ' + path + ' has been deleted, aborting data processing'
        exit()

    logs_path = path + "logs-" + filename + "/"

    # load in meta data

    file = open(logs_path + "meta.json", "r")
    meta_data = json.load(file)


    localization_test = True if (meta_data['localization_test'] == 'true') else False

    try:
        bag = rosbag.Bag(path + "raw_data.bag")
    except:
        try:
            bag = rosbag.Bag(path + "localization_test.bag")
            localization_test = True
        except:
            print "ERROR: invalid filename"
            exit()
        
    pickle_dict = {}

    print "Reading lidar frames"

    # read in lidar frames
    frames = bt.read_pointcloud_frames("/velodyne_points", bag)



    if(len(frames)):
        pickle_dict['lidar_frames'] = frames
        if (not os.path.isdir(path + "classified_frames")):
            try:
                os.mkdir(path + "classified_frames")
            except OSError:
                print ("Creation of the classifed_frames directory failed")
                exit()

        print "Writing",len(frames),"lidar frames"

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
    if (len(optimal_traj)):
        pickle_dict['optimal_traj'] = bt.trajectory_to_array(optimal_traj)

    #read in tour waypoints
    waypoints = bt.read_nav_odometry("/tour_data", bag, False)
    if (len(waypoints)):
        pickle_dict['waypoints'] = bt.trajectory_to_array(waypoints)

    #read in move_base results
    results = bt.read_action_result('/move_base/result', bag)
    if (len(results)):
        pickle_dict['action_results'] = results

    # output ground truth pose to .ply file
    el = ply.PlyElement.describe(bt.trajectory_to_array(gt_traj), "trajectory")
    ply.PlyData([el]).write(path + "/gt_pose.ply")

    # read in amcl poses if they exist
    amcl_status = bool(bt.num_messages("/amcl_pose", bag))
    
    map_frame = "hugues_map" if (localization_test) else "map"

    odom_to_base = bt.read_tf_transform("odom","base_link", bag)
    map_to_odom = bt.read_tf_transform(map_frame,"odom", bag)
    odom_to_base = bt.transforms_to_trajectory(odom_to_base)
    tf_traj = mu.transform_trajectory(odom_to_base, map_to_odom)

    # interplote tf_traj to the times of gt_traj
    
    tf_traj = mu.get_interpolations(gt_traj, tf_traj, False)
    
    #gt_traj = mu.get_interpolations(tf_traj, gt_traj, False)
    
    pickle_dict['gt_traj'] = bt.trajectory_to_array(gt_traj)
    
    if (amcl_status):
        print "Saving amcl_traj"
        pickle_dict['amcl_traj'] = bt.trajectory_to_array(tf_traj)
        
    else:
        print "Saving gmapping_traj"
        pickle_dict['gmapping_traj'] = bt.trajectory_to_array(tf_traj)

    print "Dumping data to", logs_path + "processed_data.pickle"
    with open(logs_path + 'processed_data.pickle', 'wb') as handle:
        pickle.dump(pickle_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
    
    # TODO: convert image files to .mp4 and save 

    duration = bt.bag_metadata(bag)['duration']

    static_vid_path = "/home/" + username + "/Myhal_Simulation/simulated_runs/" + filename + "/logs-" + filename + "/videos/static/"
    fpv_vid_path = "/home/" + username + "/Myhal_Simulation/simulated_runs/" + filename + "/logs-" + filename + "/videos/fpv/"
    
    static_vid_dirs = os.listdir(static_vid_path) if os.path.isdir(static_vid_path) else []
    print static_vid_dirs
    fpv_vid_dirs = os.listdir(fpv_vid_path) if os.path.isdir(fpv_vid_path) else []
    print fpv_vid_dirs
    vid_dirs = static_vid_dirs + fpv_vid_dirs

    for i in range(len(vid_dirs)):
        dir = vid_dirs[i]
        vid_path = static_vid_path if (i < len(static_vid_dirs)) else fpv_vid_path
        num_pics = len(os.listdir(vid_path + dir + "/"))
        fps = int(num_pics/duration) 
        print "Converting " + str(num_pics) +  " .jpg files at " + str(fps) + " fps to create " + dir + ".mp4 that is: " + str(num_pics/float(fps)) + "s long"
        FNULL = open(os.devnull, 'w')
        if (i < len(static_vid_dirs)):
            command = 'ffmpeg -r ' + str(fps) + ' -pattern_type glob -i ' + '"'+ static_vid_path + dir + '/default_' + dir + "_" + dir + '_link_my_camera*.jpg" -c:v libx264 ' + '"' +  static_vid_path + dir + '.mp4"'
        else: 
            command = 'ffmpeg -r ' + str(fps) + ' -pattern_type glob -i ' + '"'+ fpv_vid_path + dir + '/default_jackal_base_link_viewbot*.jpg" -c:v libx264 ' + '"' +  fpv_vid_path + dir + '.mp4"'

        print "running:\n" + command
        retcode = subprocess.call(command, shell=True, stdout=FNULL, stderr=subprocess.STDOUT)
        if (retcode == 0): # a success
            shutil.rmtree(vid_path + dir) 
        else:
            print 'Video creation failed'
        FNULL.close()

    for dir in static_vid_dirs:
        num_pics = len(os.listdir(vid_path + dir + "/"))
        fps = int(num_pics/duration) 
        
        print "Converting " + str(num_pics) +  " .jpg files at " + str(fps) + " fps to create " + dir + ".mp4 that is: " + str(num_pics/float(fps)) + "s long"

        FNULL = open(os.devnull, 'w')
        command = 'ffmpeg -r ' + str(fps) + ' -pattern_type glob -i ' + '"'+ vid_path + dir + '/default_' + dir + "_" + dir + '_link_my_camera*.jpg" -c:v libx264 ' + '"' +  vid_path + dir + '.mp4"'
        #print "running:\n" + command
        retcode = subprocess.call(command, shell=True, stdout=FNULL, stderr=subprocess.STDOUT)
        if (retcode == 0): # a success
           shutil.rmtree(vid_path + dir) 
        FNULL.close()

    bag.close()
        
    duration = RealTime.time() - start_time
    
    print "Data processed in", "{:.2f}".format(duration) ,"seconds"

