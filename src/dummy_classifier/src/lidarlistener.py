#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import ros_numpy

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2


def on_scan(cloud):
	points = ros_numpy.numpify(cloud)
	print(points.shape)
	print(points['x'].shape)
	# list = point_cloud2.read_points_list(cloud)
	# for f in cloud.fields:
	# 	print(f)
		
	# print()
	# print(list[0])


def lidarlistener():
	rospy.init_node('lidarlistener', anonymous=True)
	rospy.Subscriber('/velodyne_points', PointCloud2, on_scan)
	rospy.spin()


if __name__ == '__main__':
	lidarlistener()