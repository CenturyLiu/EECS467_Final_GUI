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

def skeletonStrToDict(skeletonStr):
    skeletonSubStrList = skeletonStr.split('\n')
    skeletonSubStrList.pop(-1)
    skeletonDict = {"time": time.time()}
    for subStr in skeletonSubStrList:
        jointName, positionStr = subStr.split(':')
        xStr, yStr, zStr = positionStr.split(",")
        skeletonDict[jointName] = [float(xStr), float(yStr), float(zStr)]
    return skeletonDict  

class kinectServer:

    def __init__(self):
        
        bind_ip = '127.1.0.0'
        bind_port = 9999

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((bind_ip, bind_port))
        self.server.listen(5)  # max backlog of connections

        print('Listening on {}:{}'.format(bind_ip, bind_port))

        # initialize data list
        self.skeletonDataList = []

    def saveData(self):
        filename = "skeleton"+datetime.now().strftime("%b-%d-%H-%M") + '.json'
        with open(filename, 'w') as f:
            json.dump(skeletonDataList, f, indent=2)
            print("saved skeleton data")
    
    def getNextData(self):
        client_sock, address = self.server.accept()
        print('Accepted connection from {}:{}'.format(address[0], address[1]))
        
        request = client_sock.recv(1024)
        print('Received {}'.format(request))
        requestStr = request.decode('utf-8')
        if requestStr == "stop":
            self.saveData()
            return False
        else:
            skeletonDict = skeletonStrToDict(requestStr)
            self.skeletonDataList.append(skeletonDict)
            print(skeletonDict)
            client_sock.send('ACK!'.encode('utf8'))
            client_sock.close()
            return True

if __name__ == "__main__":

    collector = kinectServer()
    while True:
        result = collector.getNextData()
        if not result:
            break
