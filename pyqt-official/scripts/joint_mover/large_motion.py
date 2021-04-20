import time
import json
import numpy as np
import pybullet as p
from qibullet import SimulationManager
from naive_search_joint_angle import jointAngleDataBase
from single_robot import SingleRobot, extract_human_keypoints 
from calibrate_utils import get_human_base_link


if __name__ == "__main__":
    
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True) 
    single_robot = SingleRobot(simulation_manager, client, save_joint_limits = False)
    single_robot.setPosture()
    

    with open('kinectData/robot_motionApr-18-23-45.json', 'r') as f:
        chosen_pose_list = json.load(f)

    
    time.sleep(10)

    speed_fraction = 0.5

    for ii in range(5):
        for pose in chosen_pose_list:
            '''
            print('\n---')
            print(pose['name_list'])
            print(pose['angle_list'])
            '''

            single_robot.joint_control(
                pose['name_list'],
                pose['angle_list'], speed_fraction * np.ones(len(pose['name_list'])))

            time.sleep(0.1)
    

    
    