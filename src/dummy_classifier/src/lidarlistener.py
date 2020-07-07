#!/usr/bin/env python

import rospy
import ros_numpy
import os
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
#import torch 

def network_interface_dummy(points):
	print("bye")
	# Set which gpu is going to be used o nthe machine
	GPU_ID = '1'    # Set this GPU as the only one visible by this python script

	os.environ['CUDA_VISIBLE_DEVICES'] = GPU_ID    # Convert points to a torch tensor

	points = torch.from_numpy(points)    

	if torch.cuda.is_available():
		device = torch.device("cuda:0")
	else:
		device = torch.device("cpu") 

	points = points.to(device)

	#####################
	# Network inference #
	#####################    
	
	# Instead of network inference, just create dummy classes    
	
	# Sum all points along the dimension 1. [N, 3] => [N]
	predictions = torch.sum(points, dim=1)    # Convert to integers
	predictions = torch.floor(predictions)
	predictions = predictions.type(torch.int32)    
	
	##########
	# Output #
	##########    
	
	# Convert from pytorch cuda tensor to a simple numpy array

	predictions = predictions.detach().cpu().numpy()    
	
	return predictions


def on_scan(cloud):
	print("hi")
	pc = ros_numpy.numpify(cloud)
	points=np.zeros((pc.shape[0],3))
	points[:,0]=pc['x']
	points[:,1]=pc['y']
	points[:,2]=pc['z']

	#print(points[0])

	#res = network_interface_dummy(points)

	print(points[0], res[0])


def lidarlistener():
	rospy.init_node('lidarlistener', anonymous=True)
	rospy.Subscriber('/velodyne_points', PointCloud2, on_scan)
	rospy.spin()


if __name__ == '__main__':
	lidarlistener()