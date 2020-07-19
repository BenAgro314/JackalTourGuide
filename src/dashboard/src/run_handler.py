#!/usr/bin/env python

from utilities import math_utilities as mu
from utilities import plot_utilities as pu
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from itertools import cycle
import cPickle as pickle
import time
import json
import os
import numpy as np
import copy
import logging
import psutil

logging.basicConfig(level=logging.DEBUG, format = '%(levelname)s - %(message)s')

class Run:

    def __init__(self, name, meta, data = None):
        self.name = name
        self.data = data
        self.meta = meta

    def get_data(self,field):
        return self.data[field]
    
    def keys(self):
        return self.data.keys()

class Plot:

    def __init__(self, aggregate = False):
        self.data = {'x_data':[],'y_data':[], 'series_name': [], 'color' : [], 'line': []}
        self.aggregate = aggregate
        self.series_list = []

    def add_ax(self, ax):
        self.ax = ax
        self.label()
        
    def init_axis(self):
        self.collect_data()
        for i in range(len(self.data['x_data'])):
            self.ax.plot(self.data['x_data'][i], self.data['y_data'][i], self.data['line'][i], label = self.data['series_name'][i], color = self.data['color'][i])

        self.ax.legend()

    def __str__(self):
        res = self.ax.get_title() + ", (x label: " + self.ax.get_xlabel() + ", y label: " + self.ax.get_ylabel() + ")"
        return res

    def add_series(series):
        self.series_list.append(series)

class TranslationError(Plot):
    
    def label(self):
        self.ax.set_title("Translation Error")
        self.ax.set(xlabel='Distance Travelled (m)', ylabel = 'Translation Error (m)')

    def collect_data(self):
        for series in self.series_list:
            name = series.name

            max_dist = 0
            temp_data = {'x_data':[],'y_data':[], 'series_name': [], 'color' : [], 'line': []}

            for run in series.runs:
                gt_traj = run.get_data('gt_traj')
                loc_traj = run.get_data('amcl_traj') if ('amcl_traj' in run.keys()) else run.get_data('gmapping_traj')
                dists = pu.list_distances(gt_traj)
                temp_data['x_data'].append(dists[0])
                max_dist = max(max_dist, dists[1])
                temp_data['y_data'].append(pu.translation_error(loc_traj,gt_traj))
                temp_data['series_name'].append(name)
                temp_data['color'].append(series.colors[0])
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
                temp_data['color'] = [series.colors[0]]
                temp_data['line']= ['-']

            for key in self.data:
                self.data[key] += temp_data[key]
        
    def info(self):
        res = ""
        self.data.clear()
        self.data = {'x_data':[],'y_data':[], 'series_name': [], 'color' : [], 'line': []}
        self.aggregate = True
        self.collect_data()
        self.aggregate = False

        for i in range(len(self.data['series_name'])):
            x = self.data['x_data'][i]
            y = self.data['y_data'][i]
            avg_error = 0
            for j in range(len(self.data['x_data'][i])-1):

               avg_error += ((y[j]+y[j+1])/2.0)*(x[j+1]-x[j])
            
            avg_error = avg_error/(x[-1]-x[0])

            res+= "Series: " + self.data['series_name'][i] + " has an average translation error of {:.3f} m\n".format(avg_error)

        return res

class YawError(Plot):

    def label(self):
        self.ax.set_title("Yaw Error")
        self.ax.set(xlabel='Distance Travelled (m)', ylabel = 'Yaw Error (m)')

    def collect_data(self):
        for series in self.series_list:
            name = series.name

            max_dist = 0
            temp_data = {'x_data':[],'y_data':[], 'series_name': [], 'color' : [], 'line': []}

            for run in series.runs:
                gt_traj = run.get_data('gt_traj')
                loc_traj = run.get_data('amcl_traj') if ('amcl_traj' in run.keys()) else run.get_data('gmapping_traj')
                dists = pu.list_distances(gt_traj)
                temp_data['x_data'].append(dists[0])
                max_dist = max(max_dist, dists[1])
                temp_data['y_data'].append(pu.yaw_error(loc_traj,gt_traj))
                temp_data['series_name'].append(name)
                temp_data['color'].append(series.colors[0])
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
                temp_data['color'] = [series.colors[0]]
                temp_data['line']= ['-']

            for key in self.data:
               self.data[key] += temp_data[key]
               
    def info(self):
        res = ""
        self.data.clear()
        self.data = {'x_data':[],'y_data':[], 'series_name': [], 'color' : [], 'line': []}
        self.aggregate = True
        self.collect_data()
        self.aggregate = False

        for i in range(len(self.data['series_name'])):
            x = self.data['x_data'][i]
            y = self.data['y_data'][i]
            avg_error = 0
            for j in range(len(self.data['x_data'][i])-1):

               avg_error += ((y[j]+y[j+1])/2.0)*(x[j+1]-x[j])
            
            avg_error = avg_error/(x[-1]-x[0])

            res+= "Series: " + self.data['series_name'][i] + " has an average yaw error of {:.5f} rad\n".format(avg_error)
            
        return res


class TrajectoryPlot(Plot):
    def label(self):
        self.ax.set_title('Trajectory Plot')
        self.ax.set(xlabel='x position (m)', ylabel = 'y position (m)')

    def collect_data(self):

        for series in self.series_list:
            for run in series.runs:
                gt_traj = run.get_data('gt_traj')
                loc_traj = run.get_data('amcl_traj') if ('amcl_traj' in run.keys()) else run.get_data('gmapping_traj')
                self.data['x_data'].append(gt_traj['pos_x'])
                self.data['y_data'].append(gt_traj['pos_y'])
                self.data['series_name'].append(series.name+" ground truth")
                self.data['color'].append(series.colors[0])
                self.data['line'].append('-')
                self.data['x_data'].append(loc_traj['pos_x'])
                self.data['y_data'].append(loc_traj['pos_y'])
                self.data['series_name'].append(series.name+" localization")
                self.data['color'].append(series.colors[1])
                self.data['line'].append('--')


    def info(self):
        res = "Trajectory plot: no printable info available"
        return res

class PathDifference(Plot):
    def label(self):
        self.ax.set_title("Path Difference")
        self.ax.set(xlabel='Series', ylabel = 'Average percent difference from optimal path length (%)')

    def init_axis(self):
        self.collect_data()
        self.ax.bar(self.data['x_data'], self.data['y_data'])
        
    def collect_data(self):
        
        for series in self.series_list:
            self.data['x_data'].append(series.name)
            # find average path difference for the runs of the current series
            avg_diff = 0
            for run in series.runs:
                data = run.data
                meta = run.meta
                optimal_dist = pu.list_distances(data['optimal_traj'])[1]
                gt_dist = pu.list_distances(data['gt_traj'])[1]
                avg_diff += ((gt_dist-optimal_dist)/optimal_dist)*100

            avg_diff/=len(series.runs)
            self.data['y_data'].append(avg_diff)

    def info(self):
        if (len(self.data['x_data']) == 0):
            self.collect_data()
        
        res = ""
        for i in range(len(self.data['x_data'])):
            series = self.data['x_data'][i]
            diff = self.data['y_data'][i]
            res += "Series: " + str(series) + " has a path that deviates {:.3f} % from the optimal path\n".format(diff)

        return res

class SuccessRate(Plot):
     def label(self):
        self.ax.set_title("Success Rate")
        self.ax.set(xlabel='Series', ylabel = 'Success Rate (%)')


     def init_axis(self):
        self.collect_data()
        self.ax.bar(self.data['x_data'], self.data['y_data'])
        
     def collect_data(self):
        
        for series in self.series:
            self.data['x_data'].append(series.name)
            
            rate = 0
            for run in series.runs:
                data = run.data
                meta = run.meta
                if (meta['success_status'] == 'true'):
                    rate += 1
                    
            rate/=len(series.runs)
            self.data['y_data'].append(rate*100)

class RunHandler:

    def __init__(self):
        '''Handles the storage, searching, and modification of runs in the run_data.json file'''
        # initialize variables 
        username = os.environ['USER']
        self.filepath = '/home/' +username +'/Myhal_Simulation/'
        try:
            self.json_f = open(self.filepath+'run_data.json', 'r+')
            self.table = json.load(self.json_f)
        except:
            self.json_f = open(self.filepath+'run_data.json', 'w')
            self.table = {
                'tour_names': {},
                'filter_status': {'true': [], 'false':[]},
                'localization_technique': {},
                'success_status':{'true':[], 'false':[]},
                'scenarios':{},
                'class_method':{},
                'localization_test':{'true':[],'false':[]},
                'load_world':{},
                'times':[]}


        # add new runs 
        logging.info('Loading run metadata') 
        t1 = time.time()
        run_list = os.listdir(self.filepath + 'simulated_runs/')
        self.run_map = {}

        for name in run_list:
            self.read_run(name)

        # clean old runs
        for name in self.table['times']:
            if (name not in run_list):
                self.delete_run(name)

        # rewrite to run_data.json
        self.update_json()
        logging.info("Loaded and updated metadata in {:.2f} s".format(time.time() - t1))

    def update_json(self):
        self.json_f.seek(0)
        self.json_f.truncate()
        logging.info('Writing to run_data.json')
        json.dump(self.table, self.json_f, indent = 4, sort_keys=True)
        self.json_f.close()
        self.json_f = open(self.filepath+'run_data.json', 'r+')

    def delete_run(self, name):
        self.table['times'].remove(name)
        for key, value in self.table.items():
            if (key == 'times'):
                continue
            
            for k,v in value.items():
                if name in v:
                    v.remove(name)
        logging.info('Deleted ' + name + ' from run_data.json')

    def read_run(self, name):
        ''' adds the run of name run to self.run_map if it has valid metadata. If it is not already present, adds the run to self.table '''

        def add_or_append(field, meta_d):
            if (type(meta_d[field]) == list):
                for att in meta_d[field]:
                    if (att in self.table[field] and name not in self.table[field][att]):
                        self.table[field][att].append(name)
                    elif (att not in self.table[field]):
                        self.table[field][att] = [name]
                return
                           
            if (meta_d[field] in self.table[field]):
                self.table[field][meta_d[field]].append(name)
            else:
                self.table[field][meta_d[field]] = [name]
            
        try:
            meta_f = open(self.filepath + 'simulated_runs/'+name + '/logs-' +name + '/meta.json')
            meta_d = json.load(meta_f)
            meta_f.close()
        except:
            logging.debug(name + " has a malformed or missing meta.json file")
            return
        

        self.run_map[name] = Run(name, meta_d)

        if (name in self.table['times']):
            return

        for field in self.table:
            if (field == 'times'):
                continue
            add_or_append(field,meta_d)

        self.table['times'].append(name)
        logging.info('Added ' + name + ' to run_data.json')

    def gather_run(self, name):
        ''' deserializes the pickle data for the given run and stores it in self.run_map, returns true if the run exists and has its data loaded'''

        if (name not in self.run_map):
            return False

        if (self.run_map[name].data != None):
            return True

        runpath =self.filepath + 'simulated_runs/'+name + '/logs-' +name + '/' 
        logging.debug('Adding data for ' + name)
        try:
            data_f = open(runpath + 'processed_data.pickle')
            data_d = pickle.load(data_f)
            data_f.close()
            RAM = psutil.virtual_memory().percent 
            logging.debug('RAM taken: ' + str(RAM))
            if (RAM > 95):
                logging.critical('TOO MUCH RAM TAKEN TO LOAD DATA')
                exit()
        except:
            logging.warning('Could no deserialize processed_data.pickle for ' + name)
            return False

        self.run_map[name].data = data_d
        return True


    def search(self, tour_names = None, filter_status = None, localization_technique = None, success_status = None, scenarios = None, earliest_date = None, latest_date = None, localization_test = None, class_method = None, load_world = None, date = None):
        '''returns a list of run corrisponding the given search, loads the data for those runs and adds it to self.run_map'''

        loc_l = locals()
        # create a set of all available runs (by name)
        results = set(self.run_map.keys())

        # first search by time field:
        if (date):
            results.intersection_update(set([date]))

        if (earliest_date):
            ed_int = mu.date_to_int(earliest_date)
            for rem in list(results):
                rem_int = mu.date_to_int(rem)
                if (rem_int < ed_int):
                    results.remove(rem)

        if (latest_date):
            lt_int = mu.date_to_int(latest_date)
            for rem in list(results):
                rem_int = mu.date_to_int(rem)
                if (rem_int > lt_int):
                    results.remove(rem)

        #handle all other fields with a single function 
        for par, arg in loc_l.items():
            if (par not in self.table or not arg):
                continue
            if (arg not in self.table[par]):
                return set()

            c_set = set(self.table[par][arg])
            results.intersection_update(c_set)

        res = []   
        for name in results:
            self.gather_run(name)
            res.append(self.run_map[name])

        return res
            
    

class Series:

    cl = mcolors.BASE_COLORS.keys()
    cl.remove('w')
    np.random.shuffle(cl)
    c_cycle = cycle(cl)

    def __init__(self, name, runs, colors = None):
        ''' an object for storing runs of a common set of characeristics which are going to be plotted'''

        self.runs = runs # stores a list of Run objects
        if (not colors or len(colors) < 2):
            self.select_colors()
        else:
            self.colors = colors
        self.name = name
        
    def select_colors(self):
       self.colors = [Series.c_cycle.next(), Series.c_cycle.next()] 

class Display:

    def __init__(self, rows, cols):
        self.rows = rows 
        self.cols = cols 
        self.plots = []
        self.series_list = []

    def dim(self):
        return (self.rows,self.cols)

    def size(self):
        return self.rows * self.cols

    def add_series(self,series):
        self.series_list.append(series)

    def add_plot(self, plot):
        self.plots.append(plot)

    def display(self):
        fig, axs = plt.subplots(self.rows,self.cols)
        axs = np.array(axs)
        i = 0
        for ax in axs.reshape(-1):
            self.plots[i].add_ax(ax)
            self.plots[i].series_list = self.series_list
            self.plots[i].init_axis()
            i+=1
        
        plt.show()


class Dashboard:

    def __init__(self):
        self.H = RunHandler()
        self.series_map = {}

if __name__ == "__main__":
    H = RunHandler()

    s = Series('s', H.search(date = '2020-07-17-12-47-30')) 

    d = Display(1,1)
    d.add_series(s)
    d.add_plot(TranslationError())
    d.add_series(s)
    d.display()



    


