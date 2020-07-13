#!/usr/bin/env python

import os
import numpy as np
from utilities import math_utilities as mu
from utilities import plot_utilities as pu 
from utilities import query as Q
import pickle
import matplotlib.pyplot as plt
import enum

class PlotType(enum.Enum):
    translation_error = 'translation_error'
    rotation_error = 'rotation_error'
    trajectory_plot = 'trajectory_plot'
    path_diff = 'path_diff'

class Series:

    def __init__(self, series_name, query, tour_name = None, filter_status = None, localization_technique = None, success_status = None, scenarios = [], earliest_date = None, latest_date = None):
        self.series_name = series_name
        self.username = os.environ['USER']
        self.path = '/home/' + self.username + "/Myhal_Simulation/simulated_runs" 
        self.query = query
        self.files = self.query.find_runs(tour_name, filter_status, localization_technique, success_status, scenarios, earliest_date, latest_date)
        self.load_data()
        

    def load_data(self):
        '''
        fills the dictionary self.data_table
        keys: dates
        values: the processed data for those dates

        processed data contains:
             - waypoints
             - optimal_traj
             - action_results
             - gt_traj
             - amcl_traj or gmapping_traj
             - 
        '''
        
        self.data_table= {}

        for date in self.files:
            filepath = "/home/" + self.username + "/Myhal_Simulation/simulated_runs/" + date + "/logs-" + date + "/processed_data.pickle"
            with open(filepath, 'rb') as handle:
                data = pickle.load(handle)
                self.data_table[date] = data



class Display():

    def __init__(self, rows = 1, cols = 1):
        self.rows = rows
        self.cols = cols
        self.series_list = []
        self.plot_types = []

    def add_series(self,series):
        self.series_list.append(series)

    def add_plot_type(self, plot_type):
        self.plot_types.append(plot_type)

    def display(self):
        fig, axs = plt.subplots(self.rows,self.cols)

        for i in range(len(self.plot_types)):
            r = i%(self.rows)
            c = (i-(r*self.rows))%self.cols
            self.plot(axs[r][c], self.plot_types[i])

        plt.show()

    def plot(self, axs, plot_type):
        pass

if __name__ == "__main__":

    query = Q.Query()
    query.delete_old_runs()

    s1 = Series("series1", query)

    display = Display(1,1)
    display.add_series(s1)
    display.add_plot_type(PlotType.translation_error)
    display.display()

    pass