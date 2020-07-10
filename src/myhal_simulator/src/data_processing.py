#!/usr/bin/env python

import os
import sys
import json
import matplotlib
import bag_tools
import plyfile

if __name__ == "__main__":
    username = os.environ['USER']
    if ((len(sys.argv)-1) == 0):
        print "ERROR: must input filename"
        exit()
    filename = sys.argv[1]
    print "Running diagnostics on", filename
    bag_tools.hi()