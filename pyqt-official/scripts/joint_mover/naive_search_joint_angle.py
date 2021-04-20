"""

takes human skeleton 
calculate angle feature dict and find nearest neighbor in database

"""
import os
import csv
import numpy as np
import time
from sklearn.neighbors import KDTree

class jointDataMap:
    def __init__(self):
        self.dataMatrix = []
        self.dataTree = None

class jointAngleDataBase:

    def __init__(self, database_dir_pathname=None):
        self.dataDict = {
            "LShoulder": jointDataMap(), 
            "LElbow": jointDataMap(), 
            "RShoulder": jointDataMap(),
            "RElbow": jointDataMap(),
            "Hip": jointDataMap(),
        }

        self.database_dir_pathname = database_dir_pathname

        self.loadTree("LShoulder")
        self.loadTree("LElbow")
        self.loadTree("RShoulder")
        self.loadTree("RElbow")
        self.loadTree("Hip")
    
    def loadTree(self, joint_name):
        if self.database_dir_pathname is None:
            filename = os.path.join("joint_angle_database", "Robot"+joint_name+".csv")
        else:
            filename = os.path.join(self.database_dir_pathname, 
                                    "joint_angle_database", "Robot"+joint_name+".csv")

        with open(filename, 'r') as f:
            csvReader = csv.reader(f)
            for idx, line in enumerate(csvReader):
                if idx%2 == 1:
                    self.dataDict[joint_name].dataMatrix.append(
                        [float(entry) for entry in line] )
            dataX = np.array(self.dataDict[joint_name].dataMatrix)
            self.dataDict[joint_name].dataTree = KDTree(dataX[:, 2:])
        
        # debug print first 5 columns
        # print(dataX[:5, :])
    
    def direction_match(self, joint_name, direction, verbose=False):
        dist, ind = self.dataDict[joint_name].dataTree.query([direction], k=3)

        if verbose:
            print("distance list:", dist)
            print("index list:", ind)

        idx = ind[0][0]
        return self.dataDict[joint_name].dataMatrix[idx][:2]

if __name__ == "__main__":
    start_time = time.time()
    testDataBase = jointAngleDataBase()
    end_time = time.time()
    print("load data time:", end_time-start_time)

    start_time = time.time()
    for i in range(1000):
        direction = np.random.rand(3)
        angle_list = testDataBase.direction_match("RElbow", direction)
    end_time = time.time()
    print("search time:", end_time-start_time)
