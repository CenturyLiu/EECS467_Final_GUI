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

import cv2

def playDanceImages():

    dance_images_folder = '../dance_images/'

    width = 480

    for ii in range(1, 10):
        filename = dance_images_folder + str(ii) + '.jpg'
        img = cv2.imread(filename)
        h, w, _ = img.shape
        if w >= width:
            height = int((width/w)*h)
        else:
            height = h
        output = img.resize()
        #calculate the 50 percent of original dimensions
        dsize = (width, height)
        # resize image
        output = cv2.resize(img, dsize)
        cv2.imshow('dance image', output)
        k = cv2.waitKey(5000)
        # cv2.destroyAllWindows()

if __name__ == '__main__':
    # wait simulation
    # time.sleep(10)
    playDanceImages()
    
    