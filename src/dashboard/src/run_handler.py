#!/usr/bin/env python

from utilities import  math_utilities as mu
import json
import os
import numpy
import copy

class RunHandler:

    def __init__(self):
        '''Handles the storage, searching, and modification of runs in the run_data.json file'''
        # initialize variables 
        username = os.environ['USER']
        self.filepath = '/home/' +username +'/Myhal_Simulation/'
        try:
            self.json_f = open(self.filepath+'run_data.json', 'r+')
            self.table = json.load(json_f)
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

        run_list = os.listdir(self.filepath + 'simulated_runs/')
        for name in run_list:
            if (name not in self.table['times']):
                self.read_run(name)

        # clean old runs
        for name in self.table['times']:
            if (name not in run_list):
                self.delete_run(name)

        # rewrite to run_data.json
        self.update_json()


    def update_json(self):
        self.json_f.seek(0)
        self.json_f.truncate()
        print 'Writing to run_data.json'
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
        print "Deleted " + name + " from run_data.json"

    def read_run(self, name):

        def add_or_append(field, meta_d):
            if (type(meta_d[field]) == list):
                for att in meta_d[field]:
                    if (att in self.table[field] and name not in self.table[field][att]):
                        self.table[field][att].append(name)
                    else:
                        self.table[field][att] = [name]
                return
                           
            if (meta_d[field] in self.table[field]):
                self.table[field][meta_d[field]].append(name)
            else:
                self.table[field][meta_d[field]] = [name]
            
        try:
            meta_f = open(self.filepath + 'simulated_runs/'+name + '/logs-' +name + '/meta.json')
            meta_d = json.load(meta_f)
        except:
            print "Run: " + name + " has a malformed or missing meta.json file"
            return

        for field in self.table:
            if (field == 'times'):
                continue
            add_or_append(field,meta_d)

        self.table['times'].append(name)
        print 'Added ' + name + ' to run_data.json' 



    def search(self, tour_name = None, filter_status = None, localization_technique = None, success_status = None, scenarios = None, earliest_date = None, latest_date = None, localization_test = None, class_method = None, load_world = None, date = None):
        '''returns a list of tuples: (run_name, run_data, run_meta_data) corrisponding the listed criteria'''
        pass


    def delete_old_runs(self):
        '''Deletes runs from run_data.json that no longer exist'''

        pass


    def add_new_runs(self):
        '''adds new runs to run_data.json that were recently created'''

        pass







if __name__ == "__main__":
    H = RunHandler()

