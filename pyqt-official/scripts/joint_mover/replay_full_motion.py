import time
import json
import numpy as np
import pybullet as p
from qibullet import SimulationManager
from naive_search_joint_angle import jointAngleDataBase
from single_robot import SingleRobot, extract_human_keypoints , fake_human_keypoints
from calibrate_utils import get_human_base_link
from large_motion import smooth_angle_list



if __name__ == "__main__":
    
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True) 
    single_robot = SingleRobot(simulation_manager, client, save_joint_limits = False)
    single_robot.setPosture()
    

    joint_value_matrix = np.loadtxt('robot_motion.out')

    joint_name_list = [ "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
                        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
                        "HipPitch", "HipRoll" ]

    #chosen_pose_list = smooth_angle_list(joint_value_matrix[:100])   


    chosen_pose_list = joint_value_matrix[0:50]
    print("number of chosen:", len(chosen_pose_list)) 
    
    time.sleep(3)

    speed_fraction = 0.5

    for pose in chosen_pose_list:

        # for debug use
        # get robot joints poses, create fake skeleton
        joint_name_pos_dict = single_robot.get_robot_joint_world_pose()

        # calculate and debug local coordinate systems
        key_points_pos_dict = fake_human_keypoints(joint_name_pos_dict)
        angle_dict = single_robot.get_angles_from_keypoints(key_points_pos_dict)


        single_robot.joint_control(
            joint_name_list, pose, 
            speed_fraction * np.ones(pose.shape))

        time.sleep(0.1)
    
    single_robot.debug_keypoint_trajectory(['RWrist'])
    input()
    
    