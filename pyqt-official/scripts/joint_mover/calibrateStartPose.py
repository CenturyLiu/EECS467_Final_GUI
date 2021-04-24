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
import sys
sys.path.append("D:\\UM Class\\EECS467_Final_GUI\\pyqt-official\\scripts\\kinectData")
from kinectUDPServer import kinectServer


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

            # after data transformation
            ax.plot([jointPosition1[0], jointPosition2[0]],
                    [jointPosition1[1], jointPosition2[1]],
                    [jointPosition1[2], jointPosition2[2]],
                    color='r', linestyle='-', linewidth=2, marker='o', markersize=5)
    
    return ax 

def draw_move_direction(hipPosition, ax=None):
    if ax is None:
        fig = plt.figure(frameon=False)
        ax = fig.add_subplot(111, projection='3d')
    else:
        ax = ax

    x, y, z = hipPosition
    x0, y0, z0 = -3, 0, 0

    u, v, w = x0-x, y0-y, z0-z
    instructStr = "Please move "
    if u < 0:
        instructStr += "back: "
    else:
        instructStr += "forward: "
    instructStr += f"{abs(u):.2f}m, "

    if v < 0:
        instructStr += "right: "
    else:
        instructStr += "left: "
    instructStr += f"{abs(v):.2f}m"
    ax.set_title(instructStr, fontsize=20)
    ax.quiver(x, y, z, u, v, w, length=1.0)

    return ax
            

if __name__ == '__main__':
    
    collector = kinectServer()
    fig = plt.figure(figsize=(15, 12))
    ax = plt.axes(projection='3d', )

    try:
        while True:
            collector.dataLock.acquire()
            print("number of data:", len(collector.skeletonDataList))
            collector.dataLock.release()

            lastSkeletonDict = collector.readLastData()
            print(json.dumps(lastSkeletonDict, indent=2))

            # draw sphere
            r = 0.2
            u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
            x = r*np.cos(u)*np.sin(v) - 3
            y = r*np.sin(u)*np.sin(v)
            z = r*np.cos(v)
            ax.plot_wireframe(x, y, z, color="b")

            if not lastSkeletonDict:
                continue

            hipXYPosition = np.array(lastSkeletonDict["HipCenter"][:2]) 
            hipXYTargetPosition = np.array([-3, 0])          
            if np.linalg.norm(hipXYPosition - hipXYTargetPosition) < r:
                print("Reached target position")
                break
            
            draw_move_direction(lastSkeletonDict["HipCenter"], ax=ax)
            draw_skeleton(lastSkeletonDict, ax=ax)
            ax.set_xlim(-4, -2)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            plt.pause(0.3)
            plt.cla()
        
        # stop server when program exit
        collector.stopServer()

    except KeyboardInterrupt:
        collector.stopServer()
        print("Stopped, received interrupt")
    
