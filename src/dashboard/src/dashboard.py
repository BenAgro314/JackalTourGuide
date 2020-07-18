#!/usr/bin/env python

import os
import json
from utilities import math_utilities as mu
from utilities import plot_utilities as pu 
from utilities import query as Q
import numpy as np
import pickle
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import enum
import copy
from scipy import interpolate
import subprocess 

class Plot:

    def __init__(self, ax, series_list, aggregate = False):
        self.ax = ax

        self.series = {}
        for s in series_list:
            self.series[s.name] = s

        self.data = {'x_data':[],'y_data':[], 'series_name': [], 'color' : [], 'line': []}
        self.aggregate = aggregate
        self.label()
        
       
    def label(self):
        pass

    def collect_data(self):
        pass

    def init_axis(self):
        self.collect_data()
        for i in range(len(self.data['x_data'])):
            self.ax.plot(self.data['x_data'][i], self.data['y_data'][i], self.data['line'][i], label = self.data['series_name'][i], color = self.data['color'][i])

        self.ax.legend()

    def __str__(self):
        res = self.ax.get_title() + ", (x label: " + self.ax.get_xlabel() + ", y label: " + self.ax.get_ylabel() + ")"
        return res

    def add_series(series):
        self.series[series.name] = series

class TranslationError(Plot):
    
    def label(self):
        self.ax.set_title("Translation Error")
        self.ax.set(xlabel='Distance Travelled (m)', ylabel = 'Translation Error (m)')

    def collect_data(self):
        for name, series in self.series.items():    

            max_dist = 0
            temp_data = {'x_data':[],'y_data':[], 'series_name': [], 'color' : [], 'line': []}

            for n,run in series.data_table.items():
                gt_traj = run.get_data('gt_traj')
                loc_traj = run.get_data('amcl_traj') if ('amcl_traj' in run.keys()) else run.get_data('gmapping_traj')
                dists = pu.list_distances(gt_traj)
                temp_data['x_data'].append(dists[0])
                max_dist = max(max_dist, dists[1])
                temp_data['y_data'].append(pu.translation_error(loc_traj,gt_traj))
                temp_data['series_name'].append(name)
                temp_data['color'].append(series.color_list[0])
                temp_data['line'].append('-')

            if self.aggregate:
                
                x_bins = list(np.linspace(0,max_dist, max_dist/0.1))
                y_store = [0]*len(x_bins)

                for i in range(len(temp_data['x_data'])):
                    dists = temp_data['x_data'][i]
                    min_x = dists[0]
                    max_x = dists[-1]
                    diffs = temp_data['y_data'][i]
                    f = interpolate.interp1d(dists,diffs);

                    j = 0
                    for x in x_bins:
                        if (x <= max_x and x >= min_x):
                            if (y_store[j] != 0):
                                y_store[j][0]+=f(x)
                                y_store[j][1]+=1
                            else:
                                y_store[j] = [f(x), 1] # [0] stores diff sums, [1] stores number of additions
                        j+=1

                temp_data['x_data'] = [x_bins]
                y_data = [0]*len(x_bins)
                for i in range(len(y_store)):
                    y_data[i] = float(y_store[i][0])/(float(y_store[i][1]))

                temp_data['y_data'] = [y_data]

                temp_data['series_name'] = [name]
                temp_data['color'] = [series.color_list[0]]
                temp_data['line']= ['-']

            for key in self.data:
                self.data[key] += temp_data[key]
        

class YawError(Plot):

    def label(self):
        self.ax.set_title("Yaw Error")
        self.ax.set(xlabel='Distance Travelled (m)', ylabel = 'Yaw Error (m)')

    def collect_data(self):
        for name, series in self.series.items():    

            max_dist = 0
            temp_data = {'x_data':[],'y_data':[], 'series_name': [], 'color' : [], 'line': []}

            for n,run in series.data_table.items():
                gt_traj = run.get_data('gt_traj')
                loc_traj = run.get_data('amcl_traj') if ('amcl_traj' in run.keys()) else run.get_data('gmapping_traj')
                dists = pu.list_distances(gt_traj)
                temp_data['x_data'].append(dists[0])
                max_dist = max(max_dist, dists[1])
                temp_data['y_data'].append(pu.yaw_error(loc_traj,gt_traj))
                temp_data['series_name'].append(name)
                temp_data['color'].append(series.color_list[0])
                temp_data['line'].append('-')

            if self.aggregate:
                
                x_bins = list(np.linspace(0,max_dist, max_dist/0.1))
                y_store = [0]*len(x_bins)

                for i in range(len(temp_data['x_data'])):
                    dists = temp_data['x_data'][i]
                    min_x = dists[0]
                    max_x = dists[-1]
                    diffs = temp_data['y_data'][i]
                    f = interpolate.interp1d(dists,diffs);

                    j = 0
                    for x in x_bins:
                        if (x <= max_x and x >= min_x):
                            if (y_store[j] != 0):
                                y_store[j][0]+=f(x)
                                y_store[j][1]+=1
                            else:
                                y_store[j] = [f(x), 1] # [0] stores diff sums, [1] stores number of additions
                        j+=1

                temp_data['x_data'] = [x_bins]
                y_data = [0]*len(x_bins)
                for i in range(len(y_store)):
                    y_data[i] = float(y_store[i][0])/(float(y_store[i][1]))

                temp_data['y_data'] = [y_data]

                temp_data['series_name'] = [name]
                temp_data['color'] = [series.color_list[0]]
                temp_data['line']= ['-']

            for key in self.data:
                self.data[key] += temp_data[key]

class TrajectoryPlot(Plot):
    def label(self):
        self.ax.set_title('Trajectory Plot')
        self.ax.set(xlabel='x position (m)', ylabel = 'y position (m)')

    def collect_data(self):

        for name, series in self.series.items():    
            for n,run in series.data_table.items():
                gt_traj = run.get_data('gt_traj')
                loc_traj = run.get_data('amcl_traj') if ('amcl_traj' in run.keys()) else run.get_data('gmapping_traj')
                self.data['x_data'].append(gt_traj['pos_x'])
                self.data['y_data'].append(gt_traj['pos_y'])
                self.data['series_name'].append(series.name+" ground truth")
                self.data['color'].append(series.color_list[0])
                self.data['line'].append('-')
                self.data['x_data'].append(loc_traj['pos_x'])
                self.data['y_data'].append(loc_traj['pos_y'])
                self.data['series_name'].append(series.name+" localization")
                self.data['color'].append(series.color_list[1])
                self.data['line'].append('--')

class PathDifference(Plot):
    def label(self):
        self.ax.set_title("Path Difference")
        self.ax.set(xlabel='Series', ylabel = 'Average percent difference from optimal path length (%)')

    def init_axis(self):
        self.collect_data()
        self.ax.bar(self.data['x_data'], self.data['y_data'])
        
    def collect_data(self):
        
        for name, series in self.series.items():
            self.data['x_data'].append(series.name)
            # find average path difference for the runs of the current series
            avg_diff = 0
            for date,run in series.data_table.items():
                data = run.data
                meta = run.meta
                optimal_dist = pu.list_distances(data['optimal_traj'])[1]
                gt_dist = pu.list_distances(data['gt_traj'])[1]
                avg_diff += ((gt_dist-optimal_dist)/optimal_dist)*100

            avg_diff/=len(series.data_table)
            self.data['y_data'].append(avg_diff)
            
class SuccessRate(Plot):
     def label(self):
        self.ax.set_title("Success Rate")
        self.ax.set(xlabel='Series', ylabel = 'Success Rate (%)')


     def init_axis(self):
        self.collect_data()
        self.ax.bar(self.data['x_data'], self.data['y_data'])
        
     def collect_data(self):
        
        for name, series in self.series.items():
            self.data['x_data'].append(series.name)
            
            rate = 0
            for date,run in series.data_table.items():
                data = run.data
                meta = run.meta
                if (meta['success_status'] == 'true'):
                    rate += 1
                    
            rate/=len(series.data_table)
            self.data['y_data'].append(rate*100)
    
            
class Run:

    colors = mcolors.CSS4_COLORS

    def __init__(self, name, data, meta):
        self.name = name
        self.data = data
        self.meta = meta


    def get_data(self,key):
        return self.data[key]

    def keys(self):
        return self.data.keys()

    def __str__(self):
        res = 'Name: ' + self.name + "\n"
        for key,value in self.meta.items():
            if (type(value) == type([])):
                res += key + "\n"
                for i in value:
                    res += '\t' + i + '\n'
            else:
                res += key + ": " + str(value) + "\n"

        return res

    def show(self, plot_type, color_list = None):
        ''' Given the desired plot_type class, and a list of colors by priority, plot this run'''
        fig, axs = plt.subplots(1,1)

        series_rep = Series("Run: " + self.name, color_list, date = self.name)
        if (plot_type):
            p = plot_type(axs, [series_rep], False)
            p.init_axis()
        else:
            print 'No type specified for plot'

        fig.show()

    def visualize(self, rate = 1):
        '''play the bag file of this run along with rviz'''
        username = os.environ['USER']
        ls = os.listdir("/home/"+username+"/Myhal_Simulation/simulated_runs/" + self.name)
        if ('raw_data.bag' in ls):
            shutdown_script = "/home/"+username+"/catkin_ws/visualize_bag.sh -l " + self.name + " -r " + str(rate)
        else:
            shutdown_script = "/home/"+username+"/catkin_ws/visualize_bag.sh -l " + self.name + " -r " + str(rate) + " -n localization_test.bag"
        subprocess.call(shutdown_script, shell = True)


class Series:

    colors = mcolors.BASE_COLORS
    colors.pop('w')
    num_colors = 2

    def __init__(self, name, color_list = None, tour_name = None, filter_status = None, localization_technique = None, success_status = None, scenarios = [], earliest_date = None, latest_date = None, localization_test = None, class_method = None, load_world = None, date = None):
        self.color_list = color_list

        if (color_list is None or len(color_list) < 2):
            self.color_list = []
            print Series.num_colors,'colors not specified, selecting randomly'
            for i in range(Series.num_colors):
                ind = np.random.randint(0,len(Series.colors.keys()))
                self.color_list.append(Series.colors[Series.colors.keys()[ind]])
                Series.colors.pop(Series.colors.keys()[ind])
                if (len(Series.colors.keys()) <= 0):
                    Series.colors = mcolors.TABLEAU_COLORS

        self.name = name
        self.username = os.environ['USER']
        self.path = '/home/' + self.username + "/Myhal_Simulation/simulated_runs" 
        self.query = Q.Query()
        self.files = self.query.find_runs(tour_name, filter_status, localization_technique, success_status, scenarios, earliest_date, latest_date, localization_test, class_method, load_world, date)
        self.load_data()
        
    def __str__(self):
        res = 'Name: ' + self.name + '\nQuery:\n'
        for char,val in self.query.chars.items():
            if not val:
                val = str(val)
            res+= '\t' + char + ": " + val + "\n"

        res += "Run List:\n"

        for r in self.files:
            res += '\t' + r + '\n'

        res += "Colors:\n"
        for c in self.color_list:
            res += "\t" + str(c) + "\n"
        

        return res

    def get_run(self, name):
        if name in self.data_table:
            return self.data_table[name]
        else:
            print 'Run name ' + name + ' not found in series'
            return None

    def load_data(self):
        
        self.data_table= {}

        for date in self.files:
            filepath = "/home/" + self.username + "/Myhal_Simulation/simulated_runs/" + date + "/logs-" + date 
            meta = json.load(open(filepath + "/meta.json"))
            with open(filepath+ "/processed_data.pickle", 'rb') as handle:
                data = pickle.load(handle)
                self.data_table[date] = Run(date,data, meta)

    def set_colors(self, colors):
        '''Input a list of colors from matplotlib (in order of priority) for this series to use'''
        self.color_list = colors

    def reload(self):
        self.files = self.query.reload()
        self.load_data()

class Display:

    def __init__(self, rows = 1, cols = 1):
        self.rows = rows
        self.cols = cols
        self.series_list = []
        self.plot_types = []


    def dim(self):
        return (self.rows,self.cols)

    def size(self):
        return self.rows * self.cols

    def add_series(self,series):
        self.series_list.append(series)

    def add_plot_type(self, plot_type, aggregate = False):
        self.plot_types.append((plot_type, aggregate))

    def display(self):
        fig, axs = plt.subplots(self.rows,self.cols)
        axs = np.array(axs)
        i = 0
        for ax in axs.reshape(-1):
            if (i>= len(self.plot_types)):
                print 'Not enough plot types specified for the desired display dimensions. There will be blanks'
            else:
                plot_type = self.plot_types[i][0]
                p = plot_type(ax, self.series_list, self.plot_types[i][1])
                p.init_axis()

            i+=1
        fig.show()

class Dashboard:
    '''A class for displaying and analyzing information from simulated runs'''

    def __init__(self):
        
        self.series_table = {}
        self.runs = {}
        self.display = None
        self.query = Q.Query()

    def __str__(self):

        if (len(self.series_table.keys()) == 0):
            res = "No series in display\n"
        else:
            res = "Series:\n"

        for s in self.series_table:
            res += '\t- ' + s + '\n'

        if not self.display:
            res += "No display initialized"
        else:

            res += "Display shape: " + str(self.display.dim()) + '\n'
            
            res += 'Plot Types:\n'
            f,a = plt.subplots(1,1)
           
            for p in self.display.plot_types:
                res += '\t' + str(p[0](a,[]))+ '\n'
            plt.close()
        
        return res

    def reload(self):
        for name,series in self.series_table.items():
            series.reload()
    
    def get_series(self, name):
        '''returns the series of the desired name if it exists'''
        if name in self.series_table:
            return self.series_table[name]
        else:
            print 'Series name ' + name + ' not found in dashboard'
            return None

    
    def get_run(self, name):
        '''returns run of name name if it exists'''

        if name in self.runs:
            return self.runs[name]
        else:
            print 'Run name ' + name + ' not found in dashboard'
            return None

    def clean_runs(self):
        '''Cleans deleted runs from run_data.json'''
        self.query.delete_old_runs()

    def add_series(self, series, aggregate = False):
        '''Add a series to the Dashboard. (NOTE: aggregate not implemented yet). Pass aggregate = True if series values should be averaged for plots'''
        self.series_table[series.name] = series
        for name, run in series.data_table.items():
            self.runs[name] = run

    def remove_series(self, series_name):
        '''remove a series by a given name'''
        if series_name not in self.series_table:
            print 'Series name ' + series_name + ' not found in dashboard'
            return None

        for name,run in self.runs.copy().items():
            if name in self.series_table[series].data_table:
                self.runs.pop(name)

        self.series_table.pop(series_name)

    def add_plot(self, plot_type, aggregate = False):
        ''' add a desired plot type to the Dashboard's display'''
        if (not self.display):
            print 'A display has yet to be initialized, initializing a (1,1) display'
            self.init_display()
            

        if (len(self.display.plot_types) + 1  > (self.display.size())):
            print 'Too many plot types for desired display dimensions, automatically adding row to display. Please run Dashboard.resize(rows, cols) for different display size'
            self.resize(self.display.rows+1, self.display.cols)
        
        self.display.add_plot_type(plot_type, aggregate)

    def init_display(self, rows = 1, cols = 1):
        self.display = Display(rows, cols)

    def show(self):
        '''Show the current display with the desired plot types and series'''

        self.display.series_list = []
        for name in self.series_table:
            self.display.add_series(self.series_table[name])
        print self
        self.display.display()

    def resize(self, rows, cols):
        if (not self.display):
            print 'A display has yet to be initialized, initializing a (' + str(rows) + ", " + str(cols) + ") sized display"
            self.init_display(rows,cols)
            return 

        self.display.rows = rows
        self.display.cols = cols


if __name__ == "__main__":
    D = Dashboard()
    D.create_display(1,3)
    s1 = Series('gmapping', localization_technique= 'gmapping')
    s2 = Series('amcl', localization_technique= 'amcl')
   
    D.add_series(s1)
    D.add_series(s2)

    D.list_runs('gmapping')
    D.add_plot_type(TranslationError)
    D.add_plot_type(TrajectoryPlot)
    D.add_plot_type(YawError)
    D.show_display()
    

    pass
