#!/usr/bin/env python

import math_utilities as mu
import json
import os
import numpy
import copy

class Query:

    def __init__(self):

        self.username = os.environ['USER']
        self.path = '/home/' + self.username + "/Myhal_Simulation/"

        self.files = os.listdir(self.path+'/simulated_runs/')

        self.open_json()
        self.add_new_runs()
        self.chars = {}
        
    def reload(self):
        self.update_json()
        self.files = os.listdir(self.path+'/simulated_runs/')
        self.open_json()
        self.add_new_runs()
        self.delete_old_runs()
        if (self.chars):
            return self.find_runs(self.chars['tour_name'], self.chars['filter_status'], self.chars['localization_technique'], self.chars['success_status'], self.chars['scenarios'], self.chars['earliest_date'], self.chars['latest_date'], self.chars['localization_test'], self.chars['class_method'], self.chars['load_world'])

    def open_json(self):
        try:
            self.run_json = open(self.path + "run_data.json", "r+")
            self.table = json.load(self.run_json)
        except:
            self.run_json = open(self.path + "run_data.json", "w")
            self.init_table()


    def find_runs(self, tour_name = None, filter_status = None, localization_technique = None, success_status = None, scenarios = [], earliest_date = None, latest_date = None, localization_test = None, class_method = None, load_world = None):
        '''
        given a set of conditions, return a list of all runs that satisfy those conditions,
        '''
       
        self.chars['tour_name'] = tour_name
        self.chars['filter_status'] = filter_status
        self.chars['localization_technique'] = localization_technique
        self.chars['success_status'] = success_status
        self.chars['scenarios'] = scenarios
        self.chars['earliest_date'] = earliest_date
        self.chars['latest_date'] = latest_date
        self.chars['localization_test'] = localization_test
        self.chars['class_method'] = class_method
        self.chars['load_world'] = load_world
        res = set(self.files)

        res = res.intersection(self.set_from_dict('tour_names', tour_name))
        res = res.intersection(self.set_from_dict('filter_status', filter_status))
        res = res.intersection(self.set_from_dict('success_status', success_status))
        res = res.intersection(self.set_from_dict('localization_technique', localization_technique))
        res = res.intersection(self.set_from_dict('localization_test', localization_test))
        res = res.intersection(self.set_from_dict('class_method', class_method))
        res = res.intersection(self.set_from_dict('load_world', load_world))

        for scenario in scenarios:
            res = res.intersection(self.set_from_dict('scenarios', scenario))

        time_set = set()

        if (earliest_date):
            earliest_date = mu.date_to_int(earliest_date)[0]
        else:
            earliest_date = -1

        if (latest_date ):
            latest_date  = mu.date_to_int(latest_date)[0]
        else:
            latest_date  = numpy.inf

        for date in self.table['times']:
            i = mu.date_to_int(date)[0]
            if (i >= earliest_date and i <= latest_date):
                time_set.add(date)

        res = res.intersection(time_set)

        return list(res)
        

    def set_from_dict(self, fieldname, attribute):
        
        res = set()
        if (attribute is None):
            return set(self.files)
        
        if (attribute in self.table[fieldname]):
            for date in self.table[fieldname][attribute]:
                res.add(date)

        return res

    def delete_old_runs(self):
        ''' delete runs that no longer exist from json table'''

        # look at the lits of files, find which ones no longer exist, and remove them from the dictionary
        self.files = os.listdir(self.path+'/simulated_runs/')

        def remove_from_subdict(field, run):
            for key in self.table[field]:
                k_runs = self.table[field][key][:]
                for kr in k_runs:
                    if (kr == run):
                        self.table[field][key].remove(kr)

        times = self.table['times'][:]

        for time in times:
            if (time not in self.files):
                remove_from_subdict('localization_technique', time)
                remove_from_subdict('tour_names', time)
                remove_from_subdict('scenarios', time)
                remove_from_subdict('success_status', time)
                remove_from_subdict('filter_status', time)
                remove_from_subdict('localization_test', time)
                remove_from_subdict('class_method', time)
                remove_from_subdict('load_world', time)
                self.table['times'].remove(time)

        self.update_json()
        self.open_json()

    def init_table(self):
        self.table = {}
        self.table.setdefault("tour_names", {})
        self.table.setdefault("filter_status", {'true': [], 'false': []})
        self.table.setdefault("localization_technique", {})
        self.table.setdefault("success_status", {'true': [], 'false': []})
        self.table.setdefault("scenarios", {})
        self.table.setdefault("class_method", {})
        self.table.setdefault("localization_test", {'true':[],'false':[]})
        self.table.setdefault("load_world", {})
        self.table.setdefault("times", [])

    def add_new_runs(self):
        # find all files in self.files that have meta.json file that are not in the table

        def create_or_append(key, field):
            if (key in self.table[field]):
                self.table[field][key].append(file)
            else:
                self.table[field][key] = [file]

        for file in self.files:
            path = self.path + "simulated_runs/" + file + "/logs-" + file + "/meta.json"
            if (os.path.exists(path) and (file not in self.table['times'])):
                print "adding file " + file + " to run_data.json"

                # load in json
                meta_json = open(path, 'r')
                data = json.load(meta_json)
                # tour names
                create_or_append(data['tour_names'], 'tour_names')
                
                # filter status
                
                self.table['filter_status'][data['filter_status']].append(file)

                create_or_append(data['localization_technique'], 'localization_technique')
                
                # succcess status

                self.table['success_status'][data['success_status']].append(file)

                # scenarios

                taken = []

                for scenario in data['scenarios']:
                    if (scenario not in taken):
                        create_or_append(scenario, 'scenarios')
                    taken.append(scenario)

                self.table['times'].append(file)

                self.table['localization_test'][data['localization_test']].append(file)

                create_or_append(data['class_method'], 'class_method')
                create_or_append(data['load_world'], 'load_world')

                

        self.update_json()
        self.open_json()

    
    
    # def remove_from_dict(self, field_name):
    #     attributes = self.table[field_name].copy()
    #     for att in attributes:
    #         dates = attributes[att][:]
    #         for date in dates:
    #             if (date not in self.files):
    #                 self.table[field_name][att].remove(date)


    def update_json(self):
        self.run_json.seek(0)
        self.run_json.truncate()
        json.dump(self.table, self.run_json, indent = 4, sort_keys=True)
        self.run_json.close()  

    def get_table(self):
        return copy.deepcopy(self.table)

if __name__ == "__main__":
    print 'testing query'
  
    Q = Query()
    #Q.delete_old_runs()
    #print Q.find_runs(tour_name = 'short_test', earliest_date='2020-07-12-18-06-40')
