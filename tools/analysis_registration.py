# -*- coding: utf-8 -*-

import  numpy as np
import pylab as pl
import os
import csv
import time



file = open("/home/alorli/test_data/huilongguan/velodyne/compensation/test.txt")

while True:
    text = file.readline()

    if not text:
        break

    print text
 
 
file.close()