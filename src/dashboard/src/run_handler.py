#!/usr/bin/env python

from utilities import  math_utilities as mu
import cPickle as pickle
import time
import json
import os
import numpy
import copy
import logging
import psutil
logging.basicConfig(level=logging.DEBUG, format = '%(levelname)s - %(message)s')

class Run:

    def __init__(self, name, meta, data = None):
        self.name = name
        self.data = data
        self.meta = meta


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
        '''returns a list of run names corrisponding the given search, loads the data for those runs and adds it to self.run_map'''

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

        for name in results:
            self.gather_run(name)

        return list(results)
            


            
if __name__ == "__main__":
    H = RunHandler()
    print H.search()

