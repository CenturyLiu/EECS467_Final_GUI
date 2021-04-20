#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  8 09:02:11 2021

@author: ShaoxiongYao
"""

import time
import math
import json
import pybullet as p
from datetime import datetime
from single_robot import SingleRobot
from qibullet import SimulationManager

# create environment for single robot simulation
simulation_manager = SimulationManager()
client = simulation_manager.launchSimulation(gui=True) 
robot_choice = 'pepper'

single_robot = SingleRobot(simulation_manager, client, robot_choice = robot_choice)

joint_limits = single_robot.get_joint_upper_lower_limit()

bodyUniqueId = single_robot.robot_virtual.getRobotModel()
endEffectorLinkIndex = single_robot.robot_virtual.link_dict["r_hand"].getIndex()

print("body id", bodyUniqueId)
print("end effector id", endEffectorLinkIndex)

ls = p.getLinkState(bodyUniqueId, endEffectorLinkIndex)
print("link state", ls[4])
input()

p.setGravity(0, 0, 0)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0

ikSolver = 0
useSimulation = 1
useRealTimeSimulation = 0
p.setRealTimeSimulation(useRealTimeSimulation)
trailDuration = 15

pos = [-0.2, -0.2, 1.0]

while 1:

    if (useSimulation and useRealTimeSimulation == 0):
        p.stepSimulation()

    jointPoses = p.calculateInverseKinematics(bodyUniqueId,
                                              endEffectorLinkIndex,
                                              pos,
                                              solver=ikSolver)    

    for i in range(len(jointPoses)):
        p.setJointMotorControl2(bodyIndex=bodyUniqueId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                targetVelocity=0,
                                force=500,
                                positionGain=0.03,
                                velocityGain=1)

    ls = p.getLinkState(bodyUniqueId, endEffectorLinkIndex)
    print("link state", ls[4])
    if (hasPrevPose):
        p.addUserDebugLine(prevPose, pos, [0, 0, 1], 1, trailDuration)
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1

# disconnect simulation engine
p.disconnect()

