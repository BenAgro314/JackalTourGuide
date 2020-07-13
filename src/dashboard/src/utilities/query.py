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

        try:
            self.run_json = open(self.path + "run_data.json", "r+")
            self.table = json.load(self.run_json)
        except:
            print "Cannot Find JSON file, aborting"
            exit()

        

    def find_runs(self, tour_name = None, filter_status = None, localization_technique = None, success_status = None, scenarios = [], earliest_date = None, latest_date = None):
        '''
        given a set of conditions, return a list of all runs that satisfy those conditions,
        '''

        # find the intersections of the sets of runs that satisfy each conditions

        res = set(self.files)

        res = res.intersection(self.set_from_dict('tour_names', tour_name))
        res = res.intersection(self.set_from_dict('filter_status', filter_status))
        res = res.intersection(self.set_from_dict('success_status', success_status))
        res = res.intersection(self.set_from_dict('localization_technique', localization_technique))

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

        self.remove_from_dict('localization_technique')
        self.remove_from_dict('tour_names')
        self.remove_from_dict('scenarios')
        self.remove_from_dict('success_status')
        self.remove_from_dict('filter_status')

        times = self.table['times'][:]

        for time in times:
            if (time not in self.files):
                self.table['times'].remove(time)

        self.update_json()

    
    def remove_from_dict(self, field_name):
        attributes = self.table[field_name].copy()
        for att in attributes:
            dates = attributes[att][:]
            for date in dates:
                if (date not in self.files):
                    self.table[field_name][att].remove(date)


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
    Q.delete_old_runs()
    print Q.find_runs(tour_name = 'short_test', earliest_date='2020-07-12-18-06-40')