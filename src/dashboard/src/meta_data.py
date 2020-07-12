#!/usr/bin/env python

import json
import os
import numpy as np
import rospy
import subprocess

from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseActionResult


class MetaHandler:

    def __init__(self):
        rospy.init_node('meta_data')

        self.read_params()

        self.path = "/home/" + self.username + "/Myhal_Simulation/"

        
        try:
            self.run_json = open(self.path + "run_data.json", "r+")
            self.table = json.load(self.run_json)
        except:
            self.run_json = open(self.path + "run_data.json", "w")
            self.table = {}
        
        self.start_subscribers()

        rospy.spin()

    def modify_table(self):

        self.table.setdefault("tour_names", {})
        self.table.setdefault("filter_status", {True: [], False: []})
        self.table.setdefault("localization_technique", {})
        self.table.setdefault("success_status", {True: [], False: []})
        self.table.setdefault("scenarios", {})

        # tour names
        if (self.tour_name in self.table['tour_names']):
            self.table['tour_names'][self.tour_name].append(self.start_time)
        else:
            self.table['tour_names'][self.tour_name] = [self.start_time]
        
        # filter status
        self.table['filter_status'][self.filter_status].append(self.start_time)

        # localization technique


        if (self.gmapping_status):
            tech = "gmapping"
        else:
            tech = "amcl"

        if (tech in self.table['localization_technique']):
            self.table['localization_technique'][tech].append(self.start_time)
        else:
            self.table['localization_technique'][tech] = [self.start_time]
        
        # succcess status

        self.table['filter_status'][self.successful].append(self.start_time)

        # scenarios

        taken = []

        for name in self.room_params:
            scenario = self.room_params[name]['scenario']

            if (scenario not in taken):
                if scenario in self.table['scenarios']:
                    self.table['scenarios'][scenario].append(self.start_time)
                else:
                    self.table['scenarios'][scenario] = [self.start_time]

            taken.append(scenario)
        

    def on_shutdown(self, msg):

        
        self.modify_table()
        self.run_json.seek(0)
        self.run_json.truncate()

        json.dump(self.table, self.run_json, indent = 4)
        self.run_json.close()  
        shutdown_script = "/home/"+self.username+"/catkin_ws/shutdown.sh"
        subprocess.call(shutdown_script, shell = True)

    def read_params(self):
        self.username = os.environ['USER']
        self.start_time = rospy.get_param("/start_time")
        self.tour_name = rospy.get_param("/tour_name")
        self.filter_status = rospy.get_param("/filter_status")
        self.classify_status = rospy.get_param("/classify")
        self.gmapping_status = rospy.get_param("/classify")

        room_names = rospy.get_param("/room_names")
        scenario_names = rospy.get_param("/room_names")

        self.scenario_params = {}

        for name in scenario_names:
            self.scenario_params[name] = rospy.get_param("/" + name)

        self.room_params = {}

        for name in room_names:
            self.room_params[name] = rospy.get_param("/" + name)

        #print self.room_params
        #print self.scenario_params
        # print(self.username)
        # print(self.start_time)
        # print(self.tour_name)
        # print(self.filter_status)
        # print(self.gmapping_status)
        # print(self.classify_status)

    def start_subscribers(self):
        rospy.Subscriber("/shutdown_signal", Bool, self.on_shutdown)
        self.tour_results = []
        self.successful = True
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.on_result)

    def on_result(self, msg):
        self.tour_results.append(msg)
        if msg.status.status != 3:
            self.successful = False


if __name__ == "__main__":
    '''
    create subscribers to relevant topics 
    read in relevant parameters 
    open global json file and modify it based on the read parameters 

    once shutdown message is recieved, dump to json and call data processing
    '''
    M = MetaHandler()

  
    
    