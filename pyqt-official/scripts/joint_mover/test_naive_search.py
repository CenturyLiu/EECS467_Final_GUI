import time
import json
import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
from qibullet import SimulationManager
from naive_search_joint_angle import jointAngleDataBase
from single_robot import SingleRobot, extract_human_keypoints 
from calibrate_utils import get_human_base_link
from datetime import datetime
from scipy.interpolate import CubicSpline
from numpy import polyfit

if __name__ == "__main__":

    start_time = time.time()
    testDataBase = jointAngleDataBase("..")
    end_time = time.time()
    print("load data time:", end_time-start_time)

    start_time = time.time()
    for i in range(1000):
        direction = np.random.rand(3)
        
    end_time = time.time()
    print("search time:", end_time-start_time)

    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True) 

    with open('../kinectData/skeletonApr-18-19-07.json', 'r') as f:
        skeletonDataList = json.load(f)

    base_coordinate, base_origin = get_human_base_link(skeletonDataList[:100])
    single_robot = SingleRobot(simulation_manager, client, 
                            base_coordinate = base_coordinate,
                            base_origin = base_origin, 
                            save_joint_limits = False)
    single_robot.setPosture()
    time.sleep(5.0)

    # skeletonDataList = skeletonDataList[100:600]
    skeletonDataList = skeletonDataList#[100:600]

    speed_percent = 1.0

    # list for storing chosen pose
    # each element in this list is a tuple joint_name_list and joint_angle_list

    chosen_pose_list = []

    # plot skeleton
    for i, skeletonData in enumerate(skeletonDataList):

        # TMP: convert data
        for key, data in skeletonData.items():
            if key == "time": 
                continue
            # before data transformation
            # skeletonData[key] = np.array([-data[2], -data[0], data[1]+1.0])

            # after data transformation
            skeletonData[key] = np.array([data[0], data[1], data[2]+1.0])

        # store this pose
        current_joint_name_list = []
        current_joint_angle_list = []


        headPosition = np.array(skeletonData["Head"])
        hipPosition = np.array(skeletonData["HipCenter"])

        # print("Prev Head position:", headPosition)
        headPosition = hipPosition + 0.4*(headPosition - hipPosition)
        skeletonData["Head"] = list(headPosition)
        # print("Hip position:", hipPosition)
        # print("New Head position:", headPosition)

        key_points_pos_dict = extract_human_keypoints(skeletonData)
        angle_dict = single_robot.get_angles_from_keypoints(key_points_pos_dict)

        angle_list = testDataBase.direction_match("LShoulder", angle_dict["LShoulder"])
        single_robot.joint_control( ['LShoulderPitch', 'LShoulderRoll'], 
                                    angle_list, [speed_percent, speed_percent])
        LShoulderStr = "LShoulder: " + str(angle_list[0]) + ", " + str(angle_list[1])
        # p.addUserDebugText(LShoulderStr, [1.0, 0, 1.2], [0, 0, 0], 1, 0.5)
        print(LShoulderStr)

        # store Lshoulder
        current_joint_name_list.append('LShoulderPitch')
        current_joint_name_list.append('LShoulderRoll')
        current_joint_angle_list.append(angle_list[0])
        current_joint_angle_list.append(angle_list[1])
        
        angle_list = testDataBase.direction_match("LElbow", angle_dict["LElbow"])
        single_robot.joint_control( ['LElbowYaw', 'LElbowRoll'], 
                                    angle_list, [speed_percent, speed_percent])
        LElbowStr = "LElbow: " + str(angle_list[0]) + ", " + str(angle_list[1])
        # p.addUserDebugText(LElbowStr, [1.0, 0, 1.0], [0, 0, 0], 1, 0.5)
        print(LElbowStr)

        # store Lelbow
        current_joint_name_list.append('LElbowYaw')
        current_joint_name_list.append('LElbowRoll')
        current_joint_angle_list.append(angle_list[0])
        current_joint_angle_list.append(angle_list[1])

        
        angle_list = testDataBase.direction_match("RShoulder", angle_dict["RShoulder"])
        single_robot.joint_control( ['RShoulderPitch', 'RShoulderRoll'], 
                                    angle_list, [speed_percent, speed_percent])
        RShoulderStr = "RShoulder: " + str(angle_list[0]) + ", " + str(angle_list[1])
        # p.addUserDebugText(RShoulderStr, [1.0, 0, 0.8], [0, 0, 0], 1, 0.5)
        print(RShoulderStr)

        # store RShoulder
        current_joint_name_list.append('RShoulderPitch')
        current_joint_name_list.append('RShoulderRoll')
        current_joint_angle_list.append(angle_list[0])
        current_joint_angle_list.append(angle_list[1])

        
        angle_list = testDataBase.direction_match("RElbow", angle_dict["RElbow"])
        single_robot.joint_control( ['RElbowYaw', 'RElbowRoll'], 
                                    angle_list, [speed_percent, speed_percent])
        RElbowStr = "RElbow: " + str(angle_list[0]) + ", " + str(angle_list[1])
        # p.addUserDebugText(RElbowStr, [1.0, 0, 0.6], [0, 0, 0], 1, 0.5)
        print(RElbowStr)

        # store RElbow
        current_joint_name_list.append('RElbowYaw')
        current_joint_name_list.append('RElbowRoll')
        current_joint_angle_list.append(angle_list[0])
        current_joint_angle_list.append(angle_list[1])

        angle_list = testDataBase.direction_match("Hip",angle_dict["Hip"])
        single_robot.joint_control(['HipPitch', 'HipRoll'],
                                    angle_list, [1.0, 1.0])
        HipStr = "Hip: " + str(angle_list[0]) + ", " + str(angle_list[1])
        print(HipStr)

        # store RElbow
        current_joint_name_list.append('HipPitch')
        current_joint_name_list.append('HipRoll')
        current_joint_angle_list.append(angle_list[0])
        current_joint_angle_list.append(angle_list[1])

        single_robot.get_move_command(key_points_pos_dict)

        '''
        save_command = input("Save current pose?")
        if save_command == 'y':
            temp_dict = {}
            temp_dict['name_list'] = current_joint_name_list
            temp_dict['angle_list'] = current_joint_angle_list
            chosen_pose_list.append(temp_dict)
        
        elif save_command == 's':
            break
        else:
            pass
        '''

        temp_dict = {}
        temp_dict['name_list'] = current_joint_name_list
        temp_dict['angle_list'] = current_joint_angle_list
        chosen_pose_list.append(temp_dict)

        #input()

        # time.sleep(0.1)
    
    single_robot.debug_keypoint_trajectory(['RWrist'])
    input()


    num_points = len(chosen_pose_list)
    joint_value_matrix = [x["angle_list"] for x in chosen_pose_list]
    joint_value_matrix = np.array(joint_value_matrix)
    assert(num_points == joint_value_matrix.shape[0])
    print("joint_value_matrix shape:", joint_value_matrix.shape)

    idx_arr = np.arange(num_points)

    # Idea 1: apply polynomial curve fit and then take derivative
    #       points with 0 derivative will be included

    '''
    poly_list = []
    selected_roots = []
    for i in range(10):
        coeff = polyfit(idx_arr, joint_value_matrix[:, i], 5)
        poly = np.poly1d(coeff)
        poly_diff = np.polyder(poly)

        roots = np.roots(poly_diff)
        for root in roots:
            if np.isreal(root) and root > 0 and root < num_points:
                selected_roots.append(root)
        poly_list.append(poly)
    

    selected_roots = sorted(np.real(selected_roots))
    print("selected roots:", selected_roots)
    selected_poses = [ np.array(chosen_pose_list[0]["angle_list"]) ]
    prev_root = -100
    for root in selected_roots:
        # if root-prev_root < 10:
        #     continue
        prev_root = root

        # convert root to real value
        joint_value_list = np.real([poly(root) for poly in poly_list])

        prev_joint_value_list = selected_poses[-1]
        unmoved_joints = np.abs(joint_value_list - prev_joint_value_list) < 0.1

        if not unmoved_joints.all():
            joint_value_list[unmoved_joints] = prev_joint_value_list[unmoved_joints]
            selected_poses.append(joint_value_list)
            print(root)
    '''

    # Idea 2: apply 5-point midpoint formula to numerically calculate the derivative 
    #         of each joint angle

    # ignore derivative of first and last 2 points

    h = 1
    # derivative of the 3rd term to last 3rd term
    derivative = 1 / (12 * h) * (joint_value_matrix[0:-4,:] 
                                 - 8 * joint_value_matrix[1:-3,:] 
                                 + 8 * joint_value_matrix[3:-1,:]
                                 - joint_value_matrix[4:,:])

    # choose terms
    selected_poses = [ np.array(chosen_pose_list[0]["angle_list"]) ]
    
    print(derivative)

    for ii in range(0,derivative.shape[0]):

        joint_value_list = joint_value_matrix[ii + 2, :]
        prev_joint_value_list = selected_poses[-1]
        unmoved_joints = np.abs(joint_value_list - prev_joint_value_list) < 0.5

        if max(np.abs(derivative[ii,:])) > 0.4 or not unmoved_joints.all():
            joint_value_list[unmoved_joints] = prev_joint_value_list[unmoved_joints]
            selected_poses.append(joint_value_list)



    print("selected_poses:", len(selected_poses))

    '''
    joint_value_list = [x["angle_list"][0] for x in chosen_pose_list]

    idx_list = np.arange(len(chosen_pose_list))
    cs = CubicSpline(idx_list, joint_value_list)

    z = polyfit(idx_list, joint_value_list, 5)

    p = np.poly1d(z)
    p2 = np.polyder(p)
    roots = np.roots(p2)
    print("roots:", roots)
    for root in roots:
        if np.isreal(root) and root > 0 and root < len(idx_list):
            selected_roots.append(root)
    print("selected roots:", selected_roots)
    '''
    '''
    joint_name = "LShoulder"
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.plot(joint_value_matrix[:, 0], label=joint_name)
    ax.plot(idx_arr, poly_list[0](idx_arr), label="degree=20 fit")
    # ax.plot(xs, cs(xs), label="iterpolation spline")
    plt.legend()
    plt.show()
    '''





    chosen_pose_list = [
        {
            "name_list": [ "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
                           "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
                           "HipPitch", "HipRoll"
            ], "angle_list": list(x)
        } for x in selected_poses
    ]

    filename = "robot_motion"+datetime.now().strftime("%b-%d-%H-%M") + '.json'

    with open('kinectData/'+filename, 'w') as f:
        json.dump(chosen_pose_list, f, indent=2)
        print("saved robot data")
