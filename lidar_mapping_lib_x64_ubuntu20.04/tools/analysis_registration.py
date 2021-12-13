# -*- coding: utf-8 -*-

import  numpy as np
import pylab as pl
import os
import csv
import time



file = open("/home/alorli/test_data/huilongguan/velodyne/compensation/test.txt")

while True:
    text = file.readline()  # 只读取一行内容

    # 判断是否读取到内容
    if not text:
        break

    # 每读取一行的末尾已经有了一个 `\n`
    print text
 
 
file.close()