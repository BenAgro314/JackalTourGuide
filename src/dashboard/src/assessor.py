#!/usr/bin/env python
''' A module which actively assesses the Jackals Preformance during the run and determines when
it needs to be shut down'''
import os
import numpy as np
import rospy
import tf2_ros 
import tf2_geometry_msgs
import tf.transformations
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Int32

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
        timeout = 15
        self.max_samples = int(timeout/0.1)
        self.num_samples = 0
        self.last_msg = np.array((0, 0, 0, 0), dtype=[("x", np.float), ("y", np.float),
                                                      ("z", np.float), ("t", np.float)])
        self.shutdown_pub = rospy.Publisher("shutdown_signal", Bool, queue_size=1)
        self.odom_to_base = None
        self.map_to_odom = None
        self.tour_length = None
        self.curr_t = 1 
        rospy.init_node("assessor")
        rospy.Subscriber("ground_truth/state", Odometry, self.ground_truth_callback)
        rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.on_result)
        rospy.Subscriber("/tour_length", Int32, self.tour_length_callback)
        
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
            print "Time: {:.2f} s\nPos: ({:.2f}, {:.2f}) m".format(pos["t"],
                                                                  pos["x"],
                                                                  pos["y"])
            print "Average speed across {:.1f} s: {:.2f} m/s".format(0.1 * self.num_samples,
                                                                self.avg_speed)
        drift = 0
        if (self.tour_length is not None):
            if self.curr_t != -1:
                print "Seeking target {}/{}".format(self.curr_t, self.tour_length)
            else:
                print "Tour failed, Seeking target {}/{}".format(self.curr_t, self.tour_length) 
        if (self.odom_to_base is not None and self.map_to_odom is not None):

            otob = PoseStamped()
            otob.pose.position.x = self.odom_to_base.transform.translation.x
            otob.pose.position.y = self.odom_to_base.transform.translation.y
            otob.pose.position.z = self.odom_to_base.transform.translation.z
            otob.pose.orientation.x = self.odom_to_base.transform.rotation.x
            otob.pose.orientation.y = self.odom_to_base.transform.rotation.y
            otob.pose.orientation.z = self.odom_to_base.transform.rotation.z
            otob.pose.orientation.w = self.odom_to_base.transform.rotation.w

            est_pose = tf2_geometry_msgs.do_transform_pose(otob, self.map_to_odom)
            drift = np.hypot(est_pose.pose.position.x - pos['x'], est_pose.pose.position.y - pos['y'])
            if not self.mapping_status:
                print "Estimated Pos: ({:.2f}, {:.2f}) m".format(est_pose.pose.position.x, est_pose.pose.position.y)
            print "Drift: {:.2f} m".format(drift)

        lower_lim_speed = 0.03
        upper_lim_speed = 0.08
        upper_lim_drift = 2
        lower_lim_drift = 0.5
        if (self.num_samples >= self.max_samples):
            if (self.avg_speed < upper_lim_speed and self.avg_speed > lower_lim_speed) or (drift > lower_lim_drift and drift < upper_lim_drift):
                print "Warning, Robot may be stuck"
            if (self.avg_speed < lower_lim_speed) or (drift > upper_lim_drift):
                print "Robot stuck, aborting run"
                self.log_file.write("Tour failed: robot got stuck\n")
                self.log_file.close()
                shutdown = Bool()
                shutdown.data = False
                self.shutdown_pub.publish(shutdown.data)

        print "\n"

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id == "odom" and transform.child_frame_id == "base_link"):
                self.odom_to_base = transform
            if (transform.header.frame_id == "map" and transform.child_frame_id == "odom"):
                self.map_to_odom = transform


    def running_average(self, new_sample):
        ''' compute running average speed across max_samples '''
        if self.num_samples < self.max_samples:
            self.num_samples += 1
        self.avg_speed -= self.avg_speed/self.num_samples
        self.avg_speed += new_sample/self.num_samples

    def on_result(self, msg):
        if msg.status.status != 3:
            self.curr_t = -1
        if self.curr_t != -1:
            self.curr_t +=1

    def tour_length_callback(self, msg):
        self.tour_length = msg.data

if __name__ == "__main__":
    A = Assessor()
