#!/usr/bin/env python

import tf2_ros
import tf2_geometry_msgs
import tf.transformations
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import rospy
import numpy as np

def compute_distances(traj):
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

def get_interpolations(target_times, trajectory, transform = True):
	'''
	given two trajectories, interploate the poses in trajectory to the times given in target_times (another trajectory)
	this modifies target_times so it stays in the range of trajectory's interpolations
	if transform = True, then trajectory stores transform messages not PoseStamped
	'''

	min_time = trajectory[0].header.stamp.to_sec()
	max_time = trajectory[-1].header.stamp.to_sec()

	res = []

	last = 0


	i = 0
	while i < len(target_times):

		target = target_times[i]
		time = target.header.stamp.to_sec()

		if (time < min_time or time > max_time):
			target_times.pop(i)
			continue
		
		lower_ind = last

		while (trajectory[lower_ind].header.stamp.to_sec() > time or trajectory[lower_ind+1] < time):
			if (trajectory[lower_ind].header.stamp.to_sec() > time):
				lower_ind-=1
			else:
				lower_ind+=1

		last = lower_ind+1
		if (transform):
			inter = interpolate_transform(time, trajectory[lower_ind], trajectory[lower_ind+1])
		else:
			inter = interpolate_pose(time, trajectory[lower_ind], trajectory[lower_ind+1])

		res.append(inter)
		i+=1
	
	return res



def transform_trajectory(trajectory, transformations):
	''' translate each point in trajectory by a transformation interpolated to the correct time, return the transformed trajectory'''

	# for each point in trajectory, find the interpolated transformation, then transform the trajectory point

	matching_transforms = get_interpolations(trajectory, transformations)

	res = []

	for i in range(len(matching_transforms)):
		trans = matching_transforms[i]
		traj_pose = trajectory[i]

		transformed = tf2_geometry_msgs.do_transform_pose(traj_pose, trans)
		res.append(transformed)

	return res

def interpolate_pose(time, pose1, pose2):
	''' given a target time, and two PoseStamped messages, find the interpolated pose between pose1 and pose2 ''' 

	t1 = pose1.header.stamp.to_sec();
	t2 = pose2.header.stamp.to_sec();

	alpha = 0
	if (t1 != t2):
		alpha = (time-t1)/(t2-t1)

	pos1 = pose1.pose.position
	pos2 = pose2.pose.position

	rot1 = pose1.pose.orientation
	rot1 = [rot1.x,rot1.y,rot1.z,rot1.w]
	rot2 = pose2.pose.orientation
	rot2 = [rot2.x,rot2.y,rot2.z,rot2.w]

	res = PoseStamped()

	res.header.stamp = rospy.Time(time)
	res.header.frame_id = pose1.header.frame_id

	res.pose.position.x = pos1.x + (pos2.x - pos1.x)*alpha
	res.pose.position.y = pos1.y + (pos2.y - pos1.y)*alpha
	res.pose.position.z = pos1.z + (pos2.z - pos1.z)*alpha

	res_rot = tf.transformations.quaternion_slerp(rot1,rot2,alpha)
	res.pose.orientation.x = res_rot[0]
	res.pose.orientation.y = res_rot[1]
	res.pose.orientation.z = res_rot[2]
	res.pose.orientation.w = res_rot[3]

	return res

def interpolate_transform(time, trans1, trans2):
	''' given a target time, and two TransformStamped messages, find the interpolated transform ''' 

	t1 = trans1.header.stamp.to_sec();
	t2 = trans2.header.stamp.to_sec();

	alpha = 0
	if (t1 != t2):
		alpha = (time-t1)/(t2-t1)

	pos1 = trans1.transform.translation
	pos2 = trans2.transform.translation

	rot1 = trans1.transform.rotation
	rot1 = [rot1.x,rot1.y,rot1.z,rot1.w]
	rot2 = trans2.transform.rotation
	rot2 = [rot2.x,rot2.y,rot2.z,rot2.w]

	res = TransformStamped()

	res.header.stamp = rospy.Time(time)
	res.header.frame_id = trans1.header.frame_id

	res.transform.translation.x = pos1.x + (pos2.x - pos1.x)*alpha
	res.transform.translation.y = pos1.y + (pos2.y - pos1.y)*alpha
	res.transform.translation.z = pos1.z + (pos2.z - pos1.z)*alpha

	res_rot = tf.transformations.quaternion_slerp(rot1,rot2,alpha)
	res.transform.rotation.x = res_rot[0]
	res.transform.rotation.y = res_rot[1]
	res.transform.rotation.z = res_rot[2]
	res.transform.rotation.w = res_rot[3]

	return res


if __name__ == "__main__":
	print "testing"





