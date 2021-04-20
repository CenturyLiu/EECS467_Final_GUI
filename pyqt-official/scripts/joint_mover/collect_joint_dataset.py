"""

create joint dataset:
    1. robot dataset 
    2. human dataset

    store database files:
    each file (joint angle1, joint angle2) -> relative direction
    query phase: given relative direction, find the two joint angles
    current design, LElbowTable.csv, LShoulderTable.csv, RElbowTable.csv, RShoulderTable.csv

"""
import sys
from qibullet import SimulationManager
from configobj import ConfigObj
import numpy as np
import time
import os
import csv
import json
import matplotlib.pyplot as plt
import pybullet as p
from itertools import product

from single_robot import SingleRobot, fake_human_keypoints


if __name__ == "__main__":

    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True) 
    
    single_robot = SingleRobot(simulation_manager, client, save_joint_limits = False)

    with open('kinectData/skeletonTest.json', 'r') as f:
        skeletonDataList = json.load(f)

    points_per_dim = 20#20
    #points_per_dim_hip = 20

    # plot skeleton
    # for skeletonData in skeletonDataList:

    #     # TMP: convert data
    #     for key, data in skeletonData.items():
    #         skeletonData[key] = np.array([-data[2], -data[0], data[1]+1.0])
        
    #     key_points_pos_dict = extract_human_keypoints(skeletonData)
    #     single_robot.get_angles_from_keypoints(key_points_pos_dict)

    #     time.sleep(0.1)

    try:
        joint_name = single_robot.get_joint_names()
        joint_limits = single_robot.get_joint_upper_lower_limit()
        
        single_robot.setPosture("StandZero")

        joint_name_list = ['LShoulderPitch', 'LShoulderRoll',
                      'LElbowYaw', 'LElbowRoll', 
                      'RShoulderPitch', 'RShoulderRoll', 
                      'RElbowYaw', 'RElbowRoll',
                      'HipPitch', 'HipRoll']
        


        
        database_filename_dict = {
            "LShoulder": "RobotLShoulder.csv", 
            "LElbow": "RobotLElbow.csv",
            "RShoulder": "RobotRShoulder.csv",
            "RElbow": "RobotRElbow.csv",
            "Hip": "Hip.csv",
        }
        
        for key in database_filename_dict.keys():
            database_filename_dict[key] = \
                os.path.join("joint_angle_database", database_filename_dict[key])
            
        single_robot.setPosture("StandZero")
        
        time.sleep(5.0)
        
        joint_range_dict = {
            joint_name: np.linspace(float(joint_limits[joint_name]['lower_limit']), 
                                    float(joint_limits[joint_name]['upper_limit']), points_per_dim)
            for joint_name in joint_name_list
        }

        # turn joint limit of hip to be smaller
        #smaller_factor = 0.75
        #joint_range_dict["HipPitch"] *= smaller_factor 
        #joint_range_dict["HipRoll"] *= smaller_factor
        smaller_factor = 0.5
        joint_range_dict['HipPitch'] = np.linspace(float(joint_limits['HipPitch']['lower_limit'] * smaller_factor), 
                                    float(joint_limits['HipPitch']['upper_limit']) * smaller_factor, int(points_per_dim / 2))

        joint_range_dict['HipRoll'] = np.linspace(float(joint_limits['HipRoll']['lower_limit'] * smaller_factor), 
                                    float(joint_limits['HipRoll']['upper_limit']) * smaller_factor, int(points_per_dim / 2))

        '''
        # store LShoulder data
        LShoulderAngleRange = product(joint_range_dict["LShoulderPitch"], joint_range_dict["LShoulderRoll"])
        with open(database_filename_dict["LShoulder"], 'w', newline='') as outcsv:
            writer = csv.writer(outcsv)
            writer.writerow(["LShoulderPitch", "LShoulderRoll", "localX", "localY", "localZ"])
        
        for idx, (LShoulderPitchAngle, LShoulderRollAngle) in enumerate(LShoulderAngleRange):
            print("angles:", LShoulderPitchAngle, LShoulderRollAngle)

            # set robot joint
            single_robot.joint_control(["LShoulderPitch", "LShoulderRoll"],
                                    [LShoulderPitchAngle, LShoulderRollAngle], [1.0, 1.0])

            if idx % points_per_dim == 0:
                time.sleep(5)
            else:
                time.sleep(0.2)

            # debug robot joints
            joint_name_pos_dict = single_robot.get_robot_joint_world_pose()
            # single_robot.debug_robot_joints()
            
            # calculate and debug local coordinate systems
            key_points_pos_dict = fake_human_keypoints(joint_name_pos_dict)
            angle_dict = single_robot.get_angles_from_keypoints(key_points_pos_dict)
            print("direction:", angle_dict["LShoulder"])

            with open(database_filename_dict["LShoulder"], 'a', newline='') as f:
                writer = csv.writer(f)
                data_list = [
                    LShoulderPitchAngle, LShoulderRollAngle,
                    angle_dict["LShoulder"][0], angle_dict["LShoulder"][1], angle_dict["LShoulder"][2]
                ]
                # add new row to database
                writer.writerow(data_list)
        
        # reset robot pose
        single_robot.setPosture("StandZero")        
        time.sleep(5.0)

        # store LElbow data
        LElbowAngleRange = product(joint_range_dict["LElbowYaw"], joint_range_dict["LElbowRoll"])
        with open(database_filename_dict["LElbow"], 'w', newline='') as outcsv:
            writer = csv.writer(outcsv)
            writer.writerow(["LElbowYaw", "LElbowRoll", "localX", "localY", "localZ"])
        
        for idx, (LElbowYawAngle, LElbowRollAngle) in enumerate(LElbowAngleRange):

            # set robot joint
            single_robot.joint_control(["LElbowYaw", "LElbowRoll"],
                                       [LElbowYawAngle, LElbowRollAngle], [1.0, 1.0])
            
            if idx % points_per_dim == 0:
                time.sleep(5)
            else:
                time.sleep(0.2)

            # debug robot joints
            joint_name_pos_dict = single_robot.get_robot_joint_world_pose()
            # single_robot.debug_robot_joints()
            
            # calculate and debug local coordinate systems
            key_points_pos_dict = fake_human_keypoints(joint_name_pos_dict)
            angle_dict = single_robot.get_angles_from_keypoints(key_points_pos_dict)

            with open(database_filename_dict["LElbow"], 'a', newline='') as f:
                writer = csv.writer(f)
                data_list = [
                    LElbowYawAngle, LElbowRollAngle,
                    angle_dict["LElbow"][0], angle_dict["LElbow"][1], angle_dict["LElbow"][2]
                ]
                writer.writerow(data_list)
        
        # reset robot pose
        single_robot.setPosture("StandZero")
        time.sleep(5.0)

        # store RShoulder data
        RShoulderAngleRange = product(joint_range_dict["RShoulderPitch"], joint_range_dict["RShoulderRoll"])
        with open(database_filename_dict["RShoulder"], 'w', newline='') as outcsv:
            writer = csv.writer(outcsv)
            writer.writerow(["RShoulderPitch", "RShoulderRoll", "localX", "localY", "localZ"])
        
        for idx, (RShoulderPitchAngle, RShoulderRollAngle) in enumerate(RShoulderAngleRange):

            # set robot joint
            single_robot.joint_control(["RShoulderPitch", "RShoulderRoll"],
                                       [RShoulderPitchAngle, RShoulderRollAngle], [1.0, 1.0])

            if idx % points_per_dim == 0:
                time.sleep(5)
            else:
                time.sleep(0.2)
                    
            # debug robot joints
            joint_name_pos_dict = single_robot.get_robot_joint_world_pose()
            # single_robot.debug_robot_joints()
            
            # calculate and debug local coordinate systems
            key_points_pos_dict = fake_human_keypoints(joint_name_pos_dict)
            angle_dict = single_robot.get_angles_from_keypoints(key_points_pos_dict)

            with open(database_filename_dict["RShoulder"], 'a', newline='') as f:
                writer = csv.writer(f)
                data_list = [
                    RShoulderPitchAngle, RShoulderRollAngle,
                    angle_dict["RShoulder"][0], angle_dict["RShoulder"][1], angle_dict["RShoulder"][2]
                ]
                # add new row to database
                writer.writerow(data_list)

        # reset robot pose
        single_robot.setPosture("StandZero")        
        time.sleep(5.0)

        # store RElbow data
        RElbowAngleRange = product(joint_range_dict["RElbowYaw"], joint_range_dict["RElbowRoll"])
        with open(database_filename_dict["RElbow"], 'w', newline='') as outcsv:
            writer = csv.writer(outcsv)
            writer.writerow(["RElbowYaw", "RElbowRoll", "localX", "localY", "localZ"])
        
        for idx, (RElbowYawAngle, RElbowRollAngle) in enumerate(RElbowAngleRange):

            # set robot joint
            single_robot.joint_control(["RElbowYaw", "RElbowRoll"],
                                       [RElbowYawAngle, RElbowRollAngle], [1.0, 1.0])
            
            if idx % points_per_dim == 0:
                time.sleep(5)
            else:
                time.sleep(0.2)

            # debug robot joints
            joint_name_pos_dict = single_robot.get_robot_joint_world_pose()
            # single_robot.debug_robot_joints()
            
            # calculate and debug local coordinate systems
            key_points_pos_dict = fake_human_keypoints(joint_name_pos_dict)
            angle_dict = single_robot.get_angles_from_keypoints(key_points_pos_dict)

            with open(database_filename_dict["RElbow"], 'a', newline='') as f:
                writer = csv.writer(f)
                data_list = [
                    RElbowYawAngle, RElbowRollAngle,
                    angle_dict["RElbow"][0], angle_dict["RElbow"][1], angle_dict["RElbow"][2]
                ]
                # add new row to database
                writer.writerow(data_list)
        '''
        
        # store Hip data
        #HipRange = product(joint_range_dict["HipPitch"], joint_range_dict["HipRoll"])
        HipRange = product(joint_range_dict["HipRoll"], joint_range_dict["HipPitch"])
        with open(database_filename_dict["Hip"], 'w', newline='') as outcsv:
            writer = csv.writer(outcsv)
            writer.writerow(["HipPitch", "HipRoll", "localX", "localY", "localZ"])
        
        for idx, (HipPitchAngle, HipRollAngle) in enumerate(HipRange):

            # set robot joint
            single_robot.joint_control(["HipPitch", "HipRoll"],
                                       [HipPitchAngle, HipRollAngle], [1.0, 1.0])
            
            if idx % points_per_dim == 0:
                time.sleep(5)
            else:
                time.sleep(0.2)

            # debug robot joints
            joint_name_pos_dict = single_robot.get_robot_joint_world_pose()
            # single_robot.debug_robot_joints()
            
            # calculate and debug local coordinate systems
            key_points_pos_dict = fake_human_keypoints(joint_name_pos_dict)
            angle_dict = single_robot.get_angles_from_keypoints(key_points_pos_dict)

            with open(database_filename_dict["Hip"], 'a', newline='') as f:
                writer = csv.writer(f)
                data_list = [
                    HipPitchAngle, HipRollAngle,
                    angle_dict["Hip"][0], angle_dict["Hip"][1], angle_dict["Hip"][2]
                ]
                # add new row to database
                writer.writerow(data_list)




    except KeyboardInterrupt:
        simulation_manager.stopSimulation(client)
        pass
    finally:
        simulation_manager.stopSimulation(client)