import time
import json
import numpy as np
import pybullet as p
from qibullet import SimulationManager
from naive_search_joint_angle import jointAngleDataBase
from single_robot import SingleRobot, extract_human_keypoints 
from calibrate_utils import get_human_base_link

def smooth_angle_list(joint_value_matrix):

    h = 1
    # derivative of the 3rd term to last 3rd term
    derivative = 1 / (12 * h) * (joint_value_matrix[0:-4,:] 
                                 - 8 * joint_value_matrix[1:-3,:] 
                                 + 8 * joint_value_matrix[3:-1,:]
                                 - joint_value_matrix[4:,:])

    # choose terms
    selected_poses = [ joint_value_matrix[0, :] ]

    for ii in range(0,derivative.shape[0]):

        joint_value_list = joint_value_matrix[ii + 2, :]
        prev_joint_value_list = selected_poses[-1]
        unmoved_joints = np.abs(joint_value_list - prev_joint_value_list) < 0.5

        if max(np.abs(derivative[ii,:])) > 0.4 or not unmoved_joints.all():
            joint_value_list[unmoved_joints] = prev_joint_value_list[unmoved_joints]
            selected_poses.append(joint_value_list)

    return selected_poses


if __name__ == "__main__":
    
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True) 
    single_robot = SingleRobot(simulation_manager, client, save_joint_limits = False)
    single_robot.setPosture()
    

    joint_value_matrix = np.loadtxt('robot_motion.out')

    joint_name_list = [ "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
                        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
                        "HipPitch", "HipRoll" ]
    chosen_pose_list = smooth_angle_list(joint_value_matrix)   
    print("number of chosen:", len(chosen_pose_list)) 
    
    time.sleep(10)

    speed_fraction = 0.5

    for pose in chosen_pose_list:

        single_robot.joint_control(
            joint_name_list, pose, 
            speed_fraction * np.ones(pose.shape))

        time.sleep(0.1)
    

    
    