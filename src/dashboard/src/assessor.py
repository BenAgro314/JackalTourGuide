#!/usr/bin/env python
''' A module which actively assesses the Jackals Preformance during the run and determines when
it needs to be shut down'''
import os
import numpy as np
import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

class Assessor(object):
    ''' A class which actively assesses the Jackals Preformance during the run and determines
    it needs to be shut down'''

    def __init__(self):
        ''' read ROS/system params and spin ros subscriber'''

        self.username = os.environ['USER']
        self.start_time = rospy.get_param("/start_time", None)
        self.mapping_status = rospy.get_param("/gmapping_mapping", False)
        if self.start_time is None:
            print "Error setting start time"
        try:
            self.log_file = open("/home/" + self.username +
                                 "/Myhal_Simulation/simulated_runs/"
                                 + self.start_time + "/logs-" + self.start_time + "/log.txt", "a")
        except IOError:
            print "Could not find/open log file"
            exit()
        self.avg_speed = 0
        timeout = 30
        self.max_samples = int(timeout/0.1)
        self.num_samples = 0
        self.last_msg = np.array((0, 0, 0, 0), dtype=[("x", np.float), ("y", np.float),
                                                      ("z", np.float), ("t", np.float)])
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
        inst_speed = np.hypot(self.last_msg['x']-pos['x'],
                              (self.last_msg['y'] - pos["y"])/
                              (self.last_msg["t"] - pos["t"]))
        self.running_average(inst_speed)
        self.last_msg = pos
        if not self.mapping_status:
            print "Time: {:.2f} s Pos: ({:.2f}, {:.2f}) m".format(pos["t"],
                                                                  pos["x"],
                                                                  pos["y"])
            print "{:.1f} s speed average: {:.2f} m/s\n".format(0.1 * self.num_samples,
                                                                self.avg_speed)
        if self.avg_speed < 0.05 and self.avg_speed > 0.03:
            print "Warning, Robot may be stuck"
        if self.num_samples >= self.max_samples and self.avg_speed < 0.03:
            print "Robot stuck, aborting run"
            self.log_file.write("Tour failed: robot got stuck\n")
            self.log_file.close()
            shutdown = Bool()
            shutdown.data = True
            self.shutdown_pub.publish(shutdown.data)

    def running_average(self, new_sample):
        ''' compute running average speed across max_samples '''
        if self.num_samples < self.max_samples:
            self.num_samples += 1
        self.avg_speed -= self.avg_speed/self.num_samples
        self.avg_speed += new_sample/self.num_samples

if __name__ == "__main__":
    A = Assessor()
