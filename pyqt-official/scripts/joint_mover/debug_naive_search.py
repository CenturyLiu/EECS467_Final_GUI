import time
import json
from itertools import product
import numpy as np
from qibullet import SimulationManager
from naive_search_joint_angle import jointAngleDataBase
from single_robot import SingleRobot, extract_human_keypoints, fake_human_keypoints


start_time = time.time()
testDataBase = jointAngleDataBase()
end_time = time.time()
print("load data time:", end_time-start_time)

start_time = time.time()
for i in range(1000):
    direction = np.random.rand(3)
    
end_time = time.time()
print("search time:", end_time-start_time)

simulation_manager = SimulationManager()
client = simulation_manager.launchSimulation(gui=True) 

single_robot = SingleRobot(simulation_manager, client, save_joint_limits = False)

joint_name = single_robot.get_joint_names()
joint_limits = single_robot.get_joint_upper_lower_limit()

joint_name_list = ['LShoulderPitch', 'LShoulderRoll',
                'LElbowYaw', 'LElbowRoll', 
                'RShoulderPitch', 'RShoulderRoll', 
                'RElbowYaw', 'RElbowRoll']
points_per_dim = 20
    
single_robot.setPosture()

time.sleep(5.0)

joint_range_dict = {
    joint_name: np.linspace(float(joint_limits[joint_name]['lower_limit']), 
                            float(joint_limits[joint_name]['upper_limit']), points_per_dim)
    for joint_name in joint_name_list
}

# store LShoulder data
angleRange = product(joint_range_dict["RShoulderPitch"], joint_range_dict["RShoulderRoll"])


for angle1, angle2 in angleRange:

    print("input angles:", angle1, angle2)
    # set robot joint
    single_robot.joint_control(["RShoulderPitch", "RShoulderRoll"],
                                [angle1, angle2], [1.0, 1.0])
    time.sleep(3)

    # debug robot joints
    joint_name_pos_dict = single_robot.get_robot_joint_world_pose()
    # single_robot.debug_robot_joints()

    # calculate and debug local coordinate systems
    key_points_pos_dict = fake_human_keypoints(joint_name_pos_dict)
    angle_dict = single_robot.get_angles_from_keypoints(key_points_pos_dict)

    print("direction:", angle_dict["RShoulder"])
    angle_list = testDataBase.direction_match("RShoulder", angle_dict["RShoulder"])
    print("search result:", angle_list)
    print()
    input()

    # add new row to database
    time.sleep(0.2)

# reset robot pose
single_robot.setPosture()

time.sleep(5.0)

