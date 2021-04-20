#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed April  7 16:43:02 2021

@author: ShaoxiongYao
"""
import json
import numpy as np
from numpy.linalg import norm


def get_human_base_link(skeletonDataList):
    """
        Takes a list of skeletonData as input
        Calculate coorindate and origin of base_link

        Input:
            list of skeltonData
        Output:
            base_coordinate, base_origin
    """

    HipPositionList = []
    HeadPositionList = []
    LAnklePositionList = []
    RAnklePositionList = []

    for skeletonData in skeletonDataList:
        HipPositionList.append(np.array(skeletonData["HipCenter"]))
        HeadPositionList.append(np.array(skeletonData["Head"]))
        LAnklePositionList.append(np.array(skeletonData["LAnkle"]))
        RAnklePositionList.append(np.array(skeletonData["RAnkle"]))

    HipPosition = np.mean(np.array(HipPositionList), axis=0)
    HeadPosition = np.mean(np.array(HeadPositionList), axis=0)
    LAnklePosition = np.mean(np.array(LAnklePositionList), axis=0)
    RAnklePosition = np.mean(np.array(RAnklePositionList), axis=0)

    yAxis = LAnklePosition - RAnklePosition
    yAxis /= norm(yAxis)
    refAxis = HeadPosition - HipPosition

    xAxis = np.cross(yAxis, refAxis)
    xAxis /= norm(xAxis)

    zAxis = np.cross(xAxis, yAxis)
    zAxis /= norm(zAxis)

    base_coordinate = np.array([xAxis, yAxis, zAxis])
    base_origin = (LAnklePosition+RAnklePosition)/2

    # print(base_coordinate, base_origin)

    return base_coordinate, base_origin

if __name__ == "__main__":
    skeletonDataList = None
    with open('kinectData/skeletonApr-07-20-27.json', 'r') as f:
        skeletonDataList = json.load(f)
    
    get_human_base_link(skeletonDataList[:10])
