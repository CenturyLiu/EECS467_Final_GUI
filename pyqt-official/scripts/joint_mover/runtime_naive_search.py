import pathlib
print(pathlib.Path(__file__).parent.absolute())

import sys
sys.path.append("D:\\UM Class\\EECS467_Final_project\\")

import time
import json
import numpy as np
import pybullet as p
from qibullet import SimulationManager
from naive_search_joint_angle import jointAngleDataBase
from single_robot import SingleRobot, extract_human_keypoints
from calibrate_utils import get_human_base_link
import winsound
# from kinectData.kinectServer import kinectServer
from kinectData.kinectUDPServer import kinectServer

def runtime_search():

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

    dataCollector = kinectServer()
    time.sleep(5.0)
    dataCollector.dataLock.acquire()
    base_coordinate, base_origin = get_human_base_link(dataCollector.skeletonDataList)
    dataCollector.dataLock.release()

    single_robot = SingleRobot(simulation_manager, client, 
                            base_coordinate = base_coordinate,
                            base_origin = base_origin, 
                            save_joint_limits = False)
    single_robot.setPosture()
    time.sleep(5.0)
    
    collection_length = 500
    frame_count = 0

    joint_name_list = [ "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
                        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
                        "HipPitch", "HipRoll" ]
    raw_pose_list = []

    # plot skeleton
    while True:

        # temporary code
        # exit by timing
        if frame_count >= collection_length:
            break

        frame_count += 1

        current_angle_list = []
        
        skeletonData = dataCollector.readLastData()
        if skeletonData is None:
            print("no last data")
            time.sleep(0.5)
            continue 

        # TMP: convert data
        for key, data in skeletonData.items():
            if key == "time": 
                continue
            # before data transformation
            # skeletonData[key] = np.array([-data[2], -data[0], data[1]+1.0])

            # after data transformation
            skeletonData[key] = np.array([data[0], data[1], data[2]+1.0])

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
                                    angle_list, [1.0, 1.0])
        LShoulderStr = "LShoulder: " + str(angle_list[0]) + ", " + str(angle_list[1])
        current_angle_list.extend(angle_list)
        # p.addUserDebugText(LShoulderStr, [1.0, 0, 1.2], [0, 0, 0], 1, 0.5)

        # if angle_list[0] > 0.5 and angle_list[0] < 1.0:
        #     filename = '../sounds/c2.wav'
        #     winsound.PlaySound(filename, winsound.SND_FILENAME | winsound.SND_ASYNC)
        #     print("play one sound")
        
        # if angle_list[1] > 0.5 and angle_list[1] < 1.0:
        #     filename = '../sounds/g-piano6.wav'
        #     winsound.PlaySound(filename, winsound.SND_FILENAME | winsound.SND_ASYNC)
        #     print("play another sound")
        
        angle_list = testDataBase.direction_match("LElbow", angle_dict["LElbow"])
        single_robot.joint_control( ['LElbowYaw', 'LElbowRoll'], 
                                    angle_list, [1.0, 1.0])
        LElbowStr = "LElbow: " + str(angle_list[0]) + ", " + str(angle_list[1])
        # p.addUserDebugText(LElbowStr, [1.0, 0, 1.0], [0, 0, 0], 1, 0.5)
        current_angle_list.extend(angle_list)
        
        angle_list = testDataBase.direction_match("RShoulder", angle_dict["RShoulder"])
        single_robot.joint_control( ['RShoulderPitch', 'RShoulderRoll'], 
                                    angle_list, [1.0, 1.0])
        RShoulderStr = "RShoulder: " + str(angle_list[0]) + ", " + str(angle_list[1])
        # p.addUserDebugText(RShoulderStr, [1.0, 0, 0.8], [0, 0, 0], 1, 0.5)
        current_angle_list.extend(angle_list)
        
        angle_list = testDataBase.direction_match("RElbow", angle_dict["RElbow"])
        single_robot.joint_control( ['RElbowYaw', 'RElbowRoll'], 
                                    angle_list, [1.0, 1.0])
        RElbowStr = "RElbow: " + str(angle_list[0]) + ", " + str(angle_list[1])
        # p.addUserDebugText(RElbowStr, [1.0, 0, 0.6], [0, 0, 0], 1, 0.5)
        current_angle_list.extend(angle_list)

        angle_list = testDataBase.direction_match("Hip",angle_dict["Hip"])
        single_robot.joint_control(['HipPitch', 'HipRoll'],
                                    angle_list, [1.0,1.0])
        HipStr = "Hip: " + str(angle_list[0]) + ", " + str(angle_list[1])
        current_angle_list.extend(angle_list)

        single_robot.get_move_command(key_points_pos_dict)

        # time.sleep(0.1)

        print("name_list:", joint_name_list)
        print("current_angle_list:", current_angle_list)
        raw_pose_list.append(current_angle_list)

    dataCollector.stopServer()
    np.savetxt('robot_motion.out', np.array(raw_pose_list))


if __name__ == "__main__":
    action_history = runtime_search()