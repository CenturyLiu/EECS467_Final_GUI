#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  28 15:02:02 2021

@author: ShaoxiongYao
"""


import json
import time
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
from play_sounds import play_file, DEFAULT_SONG
import winsound

import sys
sys.path.append("D:\\UM Class\\EECS467_Final_GUI\\pyqt-official\\scripts\\kinectData")
from kinectUDPServer import kinectServer



if __name__ == '__main__':
    
    # filename = '../sounds/example.wav'
    # winsound.PlaySound(filename, winsound.SND_FILENAME | winsound.SND_ASYNC)
    # print("async play sound")
    # time.sleep(3)

    filename = '../sounds/c2.wav'
    winsound.PlaySound(filename, winsound.SND_FILENAME | winsound.SND_ASYNC)
    print("play one sound")
    time.sleep(10)
    # play without blocking
    # play_file(DEFAULT_SONG, block=False) 
    