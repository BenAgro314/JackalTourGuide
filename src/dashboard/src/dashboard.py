#!/usr/bin/env python

import os
import numpy as np
from utilities import math_utilities as mu
from utilities import plot_utilities as pu 
from utilities import query as Q
import pickle
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import enum

class InfoType(enum.Enum):
    translation_error = 'translation_error'
    yaw_error = 'yaw_error'
    trajectory_plot = 'trajectory_plot'
    path_diff = 'path_diff'
    text = 'text'
    empty = "empty"

class Series:

    colors = mcolors.BASE_COLORS
    num_colors = 3

    def __init__(self, name, clist = [], tour_name = None, filter_status = None, localization_technique = None, success_status = None, scenarios = [], earliest_date = None, latest_date = None):
        self.color_list = []


        if (len(self.color_list) <= Series.num_colors):
            print Series.num_colors,'colors not specified, selecting randomly'

            for i in range(Series.num_colors):
                ind = np.random.randint(0,len(Series.colors.keys()))
                self.color_list.append(Series.colors[Series.colors.keys()[ind]])
                Series.colors.pop(Series.colors.keys()[ind])
                if (len(Series.colors.keys()) == 0):
                    Series.colors = mcolors.TABLEAU_COLORS
    

        self.name = name
        self.username = os.environ['USER']
        self.path = '/home/' + self.username + "/Myhal_Simulation/simulated_runs" 
        self.query = Q.Query()
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

    def set_colors(self, colors):
        self.color_list = colors

    def run_list(self):
        return self.files

class Display:

    def __init__(self, rows = 1, cols = 1):
        self.rows = rows
        self.cols = cols
        self.series_list = []
        self.plot_types = []

    def size(self):
        return self.rows*self.cols

    def add_series(self,series):
        self.series_list.append(series)

    def add_plot_type(self, plot_type):
        self.plot_types.append(plot_type)

    def display(self):
        fig, axs = plt.subplots(self.rows,self.cols)
        axs = np.array(axs)
        i = 0

        for ax in axs.reshape(-1):
            plot_type = InfoType.empty if (i >= len(self.plot_types)) else self.plot_types[i]
            self.plot(ax, plot_type )
            i+=1

        plt.show()

    def plot(self, ax, plot_type):
        if (plot_type == InfoType.empty):
            return

        if (plot_type == InfoType.translation_error):
            ax.set_title('Translation Error')
            ax.set(xlabel='Distance Travelled (m)', ylabel = 'Translation Error (m)')
            for series in self.series_list:
                for date in series.data_table:
                    data = series.data_table[date]
                    gt_traj = data['gt_traj']
                    loc_traj = data['amcl_traj'] if ('amcl_traj' in data) else data['gmapping_traj']
                    ax.plot(pu.list_distances(gt_traj)[0], pu.translation_error(loc_traj,gt_traj), label = series.name, color = series.color_list[0])

        if (plot_type == InfoType.yaw_error):
            ax.set_title('Yaw Error')
            ax.set(xlabel='Distance Travelled (m)', ylabel = 'Yaw Error (m)')
            for series in self.series_list:
                for date in series.data_table:
                    data = series.data_table[date]
                    gt_traj = data['gt_traj']
                    loc_traj = data['amcl_traj'] if ('amcl_traj' in data) else data['gmapping_traj']
                    ax.plot(pu.list_distances(gt_traj)[0], pu.yaw_error(loc_traj,gt_traj), label = series.name, color = series.color_list[0])

        if (plot_type == InfoType.trajectory_plot):
            ax.set_title('Trajectory Plot')
            ax.set(xlabel='x position (m)', ylabel = 'y position (m)')
            for series in self.series_list:
                for date in series.data_table:
                    data = series.data_table[date]
                    gt_traj = data['gt_traj']
                    loc_traj = data['amcl_traj'] if ('amcl_traj' in data) else data['gmapping_traj']
                    ax.plot(gt_traj['pos_x'],gt_traj['pos_y'],  label = series.name+" ground truth", color = series.color_list[0])
                    ax.plot(loc_traj['pos_x'],loc_traj['pos_y'],  label = series.name+" localization", color = series.color_list[1])

        ax.legend()

class Dashboard:

    def __init__(self):
        self.series_table = {}
        self.display = None
        self.query = Q.Query()
    
    def delete_old_runs(self):
        self.query.delete_old_runs()

    def add_series(self, series, aggregate = False):
        self.series_table[series.name] = (series,aggregate)

    def add_plot_type(self, plot_type):
        if (not self.display):
            print 'A display has yet to be initialized'
            return

        if (len(self.display.plot_types) + 1  > (self.display.size())):
            print 'Too many plot types for desired display dimensions, please run Dashboard.resize_display(rows, cols)'
        
        self.display.add_plot_type(plot_type)

    def create_display(self, rows = 1, cols = 1):
        self.display = Display(rows, cols)

    def show_display(self):
        self.display.series_list = []
        for name in self.series_table:
            self.display.add_series(self.series_table[name][0])
        self.display.display()

    def resize_display(self, rows, cols):
        if (not self.display):
            print 'A display has yet to be initialized'
            return

        self.display.rows = rows
        self.display.cols = cols

    def list_runs(self, series):
        if (type(series) == type('string')):
            l = self.series_table[series][0].run_list()
        else:
            l = series.run_list()

        for name in l:
            print name

def help():
    pass

if __name__ == "__main__":
    D = Dashboard()
    D.create_display(1,3)
    s1 = Series('gmapping', localization_technique= 'gmapping')
    s2 = Series('amcl', localization_technique= 'amcl')
   
    D.add_series(s1)
    D.add_series(s2)

    D.list_runs('gmapping')
    D.add_plot_type(InfoType.translation_error)
    D.add_plot_type(InfoType.trajectory_plot)
    D.add_plot_type(InfoType.yaw_error)
    D.show_display()
    

    pass