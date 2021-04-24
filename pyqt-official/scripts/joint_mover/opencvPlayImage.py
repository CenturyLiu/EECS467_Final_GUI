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

def displayImage(filename, window_name, rescale=False, width = 480):
    img = cv2.imread(filename)
    if rescale:
        h, w, _ = img.shape
        if w >= width:
            height = int((width/w)*h)
        else:
            height = h
        #calculate the 50 percent of original dimensions
        dsize = (width, height)
        # resize image
        output = cv2.resize(img, dsize)
    else:
        output = img

    cv2.namedWindow(window_name)
    cv2.moveWindow(window_name,40,30)
    cv2.imshow(window_name, output)
    k = cv2.waitKey(5000)

def playDanceImages():

    dance_images_folder = '../dance_images/'

    displayImage(dance_images_folder+"start.png", "start")
    cv2.destroyAllWindows()

    for ii in range(1, 10):
        filename = dance_images_folder + str(ii) + '.jpg'
        displayImage(filename, "draw pose", rescale=True)
    cv2.destroyAllWindows()

    displayImage(dance_images_folder+"hard.png", "hard")
    cv2.destroyAllWindows()

    for ii in range(1, 6):
        filename = dance_images_folder + "occlude_" + str(ii) + '.jpg'
        displayImage(filename, "draw pose", rescale=True)
    for ii in range(1, 5):
        filename = dance_images_folder + "waist" + str(ii) + '.jpg'
        displayImage(filename, "draw pose", rescale=True)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # wait simulation
    time.sleep(10)
    playDanceImages()
    
    