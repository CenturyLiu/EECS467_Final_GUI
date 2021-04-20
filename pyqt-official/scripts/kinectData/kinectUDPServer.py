#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  28 15:02:02 2021

@author: ShaoxiongYao
"""


import sys
import socket
import json
import threading
import time
from datetime import datetime
import signal
from threading import Lock

def skeletonStrToDict(skeletonStr):
    skeletonSubStrList = skeletonStr.split('\n')
    skeletonSubStrList.pop(-1)
    skeletonDict = {"time": time.time()}
    for subStr in skeletonSubStrList:
        jointName, positionStr = subStr.split(':')
        xStr, yStr, zStr = positionStr.split(",")
        x, y, z = float(xStr), float(yStr), float(zStr)
        skeletonDict[jointName] = [-z, -x, y]
    return skeletonDict  

class kinectServer:

    def __init__(self):
        
        bind_ip = '127.1.0.0'
        bind_port = 9999

        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.bind((bind_ip, bind_port))

        print('Listening on {}:{}'.format(bind_ip, bind_port))

        # initialize data list
        self.skeletonDataList = []

        # initialize data lock
        self.shutdown = False
        self.dataLock = Lock()

        self.collectDataThread = threading.Thread(
            target=self.loopGetNextData
        )
        self.collectDataThread.start()

    def saveData(self):
        filename = "skeleton"+datetime.now().strftime("%b-%d-%H-%M") + '.json'
        print(type(self.skeletonDataList))
        with open(filename, 'w') as f:
            self.dataLock.acquire()
            json.dump(self.skeletonDataList, f, indent=2)
            self.dataLock.release()
            print("saved skeleton data")
    
    def loopGetNextData(self):

        while not self.shutdown:
            message, address = self.server.recvfrom(1024)
            
            requestStr = message.decode('utf-8')
            
            skeletonDict = skeletonStrToDict(requestStr)

            self.dataLock.acquire()
            # print("received new data")
            self.skeletonDataList.append(skeletonDict)
            self.dataLock.release()
    
    def readLastData(self):
        self.dataLock.acquire()
        if self.skeletonDataList != []:
            lastSkeletonDict = self.skeletonDataList[-1]
        else:
            lastSkeletonDict = None
        self.dataLock.release()

        return lastSkeletonDict
    
    def stopServer(self):
        self.dataLock.acquire()
        self.shutdown = True
        self.dataLock.release()

        self.collectDataThread.join()

if __name__ == "__main__":

    collector = kinectServer()

    try:
        while True:
            time.sleep(0.5)
            collector.dataLock.acquire()
            print("number of data:", len(collector.skeletonDataList))
            collector.dataLock.release()

            lastSkeletonDict = collector.readLastData()
            print(json.dumps(lastSkeletonDict, indent=2))
    except KeyboardInterrupt:
        collector.stopServer()
        collector.saveData()
