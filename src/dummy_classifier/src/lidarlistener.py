#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField


def on_scan(scan):
	print("here")


def lidarlistener():
	rospy.init_node('lidarlistener', anonymous=True)
	rospy.Subscriber('/velodyne_points', PointCloud2, on_scan)
	rospy.spin()


if __name__ == '__main__':
	lidarlistener()