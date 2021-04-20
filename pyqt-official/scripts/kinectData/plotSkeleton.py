#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  28 15:02:02 2021

@author: ShaoxiongYao
"""


import json
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

bonePairList = [
    ["LHip", "LKnee"], ["RHip", "RKnee"], 
    ["HipCenter", "LHip"], ["HipCenter", "RHip"],
    ["HipCenter", "ShoulderCenter"], ["ShoulderCenter", "Head"], 
    ["ShoulderCenter", "LShoulder"], ["LShoulder", "LElbow"], ["LElbow", "LWrist"], 
    ["ShoulderCenter", "RShoulder"], ["RShoulder", "RElbow"], ["RElbow", "RWrist"]
]

def draw_skeleton(skeletonData, ax=None):
    if ax is None:
        fig = plt.figure(frameon=False)
        ax = fig.add_subplot(111, projection='3d')
    else:
        ax = ax

    for bonePair in bonePairList:
        jointName1, jointName2 = bonePair
        if jointName1 in skeletonData.keys() and jointName2 in skeletonData.keys():
            jointPosition1 = skeletonData[jointName1]
            jointPosition2 = skeletonData[jointName2]

            # before data transformation
            # ax.plot([-jointPosition1[2], -jointPosition2[2]],
            #         [-jointPosition1[0], -jointPosition2[0]],
            #         [jointPosition1[1], jointPosition2[1]],
            #         color='r', linestyle='-', linewidth=2, marker='o', markersize=5)

            # after data transformation
            ax.plot([jointPosition1[0], jointPosition2[0]],
                    [jointPosition1[1], jointPosition2[1]],
                    [jointPosition1[2], jointPosition2[2]],
                    color='r', linestyle='-', linewidth=2, marker='o', markersize=5)
    
    return ax 

if __name__ == '__main__':
    skeletonDataList = None
    with open('skeletonApr-10-22-21.json', 'r') as f:
        skeletonDataList = json.load(f)
    
    fig = plt.figure()
    ax = plt.axes(projection='3d', )

    try:
        for skeletonData in skeletonDataList:

            draw_skeleton(skeletonData, ax=ax)
            ax.set_xlim(-4, -2)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            plt.pause(0.3)
            plt.cla()
    except KeyboardInterrupt:
        print("Stopped, received keyboad interrupt")
    
