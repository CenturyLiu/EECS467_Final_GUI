#!/usr/bin/env python

import sys
import pathlib
import matplotlib.pyplot as plt
import json

from naive_search_joint_angle import jointAngleDataBase
from single_robot import SingleRobot, extract_human_keypoints 
from calibrate_utils import get_human_base_link

if __name__ == '__main__':

    
    y = [1, 2, 3, 100, 10, 20]

    plt.plot(y, label="test curve")

    with open('../kinectData/skeletonApr-18-19-07.json', 'r') as f:
        skeletonDataList = json.load(f)

    plt.show()

    
