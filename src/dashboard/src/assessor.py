#!/usr/bin/env python
''' A module which actively assesses the Jackals Preformance during the run and determines when
it needs to be shut down'''
import os
import logging
import numpy as np
import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Assessor(object):
    ''' A class which actively assesses the Jackals Preformance during the run and determines
    it needs to be shut down'''

    def __init__(self):
        ''' read ROS/system params and spin ros subscriber'''

        self.username = os.environ['USER']
        self.start_time = rospy.get_param("/start_time", None)
        self.mapping_status = rospy.get_param("/gmapping_mapping", False)
        if self.start_time is None:
            logging.warning("Error setting start time")
        try:
            self.log_file = open("/home/" + self.username +
                                 "/Myhal_Simulation/simulated_runs/"
                                 + self.start_time + "/log.txt", "a")
        except IOError:
            logging.warning("Could not find/open log file")
        self.shutdown_pub = rospy.Publisher("shutdown_signal", Bool, queue_size=1)
        rospy.init_node("assessor")
        rospy.Subscriber("ground_truth/state", Odometry, self.ground_truth_callback)
        rospy.spin()

    def ground_truth_callback(self, msg):
        ''' called whenever a ground truth pose message is recieved '''
        pos = np.array((msg.pose.pose.position.x, msg.pose.pose.position.y,
                        msg.pose.pose.position.z, msg.header.stamp.to_sec()),
                       dtype=[("x", np.float), ("y", np.float), ("z", np.float),
                              ("t", np.float)])
        if not self.mapping_status:
            print "Current Time: {:.2f}".format(pos["t"])
            print "Current Pos: {:.2f}".format(pos["x"])

if __name__ == "__main__":
    A = Assessor()
