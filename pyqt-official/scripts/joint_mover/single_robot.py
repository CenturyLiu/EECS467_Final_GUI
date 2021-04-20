#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  2 16:43:02 2021

@author: shijiliu
"""


import sys
from qibullet import SimulationManager
from configobj import ConfigObj
import numpy as np
import time
import json
import matplotlib.pyplot as plt
import pybullet as p
from scipy.spatial.transform import Rotation
from numpy.linalg import norm
from math import atan2
from collections import deque
from motion_filter import butter_bandpass_filter

def get_homogeneous_matrix(rotation, transformation):
    '''
    

    Parameters
    ----------
    rotation : np.array
        numpy 3*3 array, representing rotation (can be extented to shear, reflection ...)
    transformation : np.array
        transformation, 1*3.

    Returns
    -------
    H : np.array
        homogeneous matrix with H[3,0:3] = 0, H[3][3] = 1
    '''
    H = np.zeros((4,4))
    H[0:3,0:3] = rotation
    
    H[0:3,3] = transformation.T
    H[3][3] = 1.0
    
    return H

class SingleRobot(object):
    def __init__(self, simulation_manager, client, base_coordinate = np.eye(3), base_origin = np.array([0,0,0]),  
                 robot_choice = 'pepper', save_joint_limits = False):
        '''
        

        Parameters
        ----------
        robot_choice : string, optional
            which robot to choose, valid values are 'pepper' and 'nao'. The default is 'pepper'.
        save_joint_limits : bool, optional
            whether the joint limits of a robot needs to be saved. The default is False.

        Returns
        -------
        None.

        '''
        self.save_joint_limits = save_joint_limits
        self.joint_limits = ConfigObj()
        self.robot_choice = robot_choice
        
        self.simulation_manager = simulation_manager
        self.client = client

        if robot_choice == 'pepper':
            self.robot_virtual = self.simulation_manager.spawnPepper(self.client,spawn_ground_plane = True)
            self.del_virtual = self.simulation_manager.removePepper
            
        elif robot_choice == 'nao':
            self.robot_virtual = self.simulation_manager.spawnNao(self.client,spawn_ground_plane = True)
            self.del_virtual = self.simulation_manager.removeNao

        else:
            print("Invalid robot_choice")
            print("Default to pepper")
            self.robot_virtual = self.simulation_manager.spawnPepper(self.client,spawn_ground_plane = True)
            self.del_virtual = self.simulation_manager.removePepper
            
        # get unique id
        self.bodyUniqueId = self.robot_virtual.getRobotModel()
        
        self.get_link_names()
        self.get_joint_names()
        self.base_origin = base_origin
        self.base_coodinate = base_coordinate
        self.angle_history_dict = { joint_name: [] 
                                    for joint_name in self.joint_names }
        
        # debug items
        self.debug_id = []
        

    def __del__(self):
        self.del_virtual(self.robot_virtual)
        self.simulation_manager.stopSimulation(self.client)
        
        
    def get_joint_names(self):
        self.joint_names = []
        for name in self.robot_virtual.joint_dict.keys():
            self.joint_names.append(name)
        
        return self.joint_names
    
    def get_link_names(self):
        self.link_names = []
        for name in self.robot_virtual.link_dict.keys():
            self.link_names.append(name)
            
        if self.save_joint_limits:
            with open(self.robot_choice + '.txt', 'w') as f:
                for name in self.link_names:
                    f.write("%s\n" % (name))
            
        return self.link_names
    
    def get_joint_upper_lower_limit(self):
        # return upper and lower limit of available joints
        
        for name in self.joint_names:
            # create storage
            self.joint_limits[name] = {}
            
            # get the joint
            joint = self.robot_virtual.getJoint(name)
            
            # get joint limit
            self.joint_limits[name]['lower_limit'] = joint.getLowerLimit()
            self.joint_limits[name]['upper_limit'] = joint.getUpperLimit()
            
        if self.save_joint_limits:
            self.joint_limits.filename = self.robot_choice + '_joint_limits'
            self.joint_limits.write()
        #else:
        #    print(self.joint_limits)
        return self.joint_limits
    
    def get_joint_angles(self, joint_name_list):
        joint_angle_list = self.robot_virtual.getAnglesPosition(joint_name_list)
        return joint_angle_list

    def joint_control(self, joint_name_list, joint_value_list, speed_percentage_list):
        # receive joint name, value and velocity percentage
        # check whether joint names are valid
        # control valid joints
        
        valid_joint_name_list = []
        valid_joint_value_list = []
        valid_speed_percentage_list = []
        for ii, name in enumerate(joint_name_list):
            if name in self.joint_names:
                valid_joint_name_list.append(name)
                valid_joint_value_list.append(joint_value_list[ii])
                valid_speed_percentage_list.append(speed_percentage_list[ii])
        
        filtered_valid_value_list = []
        for joint_name, joint_value in zip(valid_joint_name_list, valid_joint_value_list):
            self.angle_history_dict[joint_name].append(joint_value)

            # print(joint_name, self.angle_history_dict[joint_name])

            if len(self.angle_history_dict[joint_name]) >= 2:
                filtered_value = butter_bandpass_filter(
                    np.array(self.angle_history_dict[joint_name])
                )
                filtered_valid_value_list.append(filtered_value[-1])
                # print("filter value:", filtered_value)
            else:
                filtered_valid_value_list.append(joint_value)
        
        if valid_joint_name_list:
            self.robot_virtual.setAngles(valid_joint_name_list, valid_joint_value_list, valid_speed_percentage_list)
        
    def setPosture(self,posture="Stand"):
        # set virtual robot to a posture
        # valid values: "Crouch", "Stand", "StandZero"
        self.robot_virtual.goToPosture(posture,1.0)
        
    def get_joint_world_pose(self, joint_name, verbose=False):
        # get the [x,y,z] pose of a joint in world frame
        
        # check whether the joint exists
        if not joint_name in self.joint_names:
            print("Invalid joint name")
            return None 
       
        # get joint index
        joint = self.robot_virtual.getJoint(joint_name)
        index = joint.getIndex()
        
        # use pybullet to get the JointAxis, ParentFramePos, ParentFrameOrientation, parent_id
        
        joint_info = p.getJointInfo(self.bodyUniqueId, index)
        
        joint_axis = joint_info[13]
        ParentFramePos = joint_info[14]
        ParentFrameOrientation = joint_info[15]
        parent_id = joint_info[16]
        
        '''
        print("\n---")
        print("joint_axis: " ,joint_axis)
        print("ParentFramePos: " ,ParentFramePos)
        print("ParentFrameOrientation: ", ParentFrameOrientation)
        print("parent_id: ", parent_id)
        '''
        
        # get the parent link state: linkWorldPosition, linkWorldOrientation, worldLinkFramePosition, worldLinkFrameOrientation
        # see P23, 24 https://usermanual.wiki/Document/pybullet20quickstart20guide.479068914.pdf
        
        parent_link_info = p.getLinkState(self.bodyUniqueId, parent_id)
        #print("parent_link_info",parent_link_info)
        
        # use the worldLinkFramePosition and  worldLinkFrameOrientation
        parent_worldLinkFramePosition = parent_link_info[4]
        parent_worldLinkFrameOrientation = parent_link_info[5]
        
        '''
        print("parent_worldLinkFramePosition: ",parent_worldLinkFramePosition)
        print("parent_worldLinkFrameOrientation: ", parent_worldLinkFrameOrientation)
        '''
        
        # apply transformations we get above to calculate [x,y,z] pose of a joint in world frame
        
        Rot_world_to_parent = Rotation.from_quat([parent_worldLinkFrameOrientation[0],
                                                  parent_worldLinkFrameOrientation[1],
                                                  parent_worldLinkFrameOrientation[2],
                                                  parent_worldLinkFrameOrientation[3]]).as_matrix()
        H_world_to_parent = get_homogeneous_matrix(Rot_world_to_parent,np.array([parent_worldLinkFramePosition[0],parent_worldLinkFramePosition[1],parent_worldLinkFramePosition[2]]))
        
        
        Rot_parent_to_joint = Rotation.from_quat([ParentFrameOrientation[0],
                                                  ParentFrameOrientation[1],
                                                  ParentFrameOrientation[2],
                                                  ParentFrameOrientation[3]]).as_matrix()
        H_parent_to_joint = get_homogeneous_matrix(Rot_parent_to_joint, np.array([ParentFramePos[0],ParentFramePos[1],ParentFramePos[2]]))
        
        #H_joint_in_world = np.dot(H_parent_to_joint, np.dot(H_world_to_parent, np.array([0,0,0,1]).T ) )
        
        # apply local transformation matrix first, then apply global transformation
        H_joint_in_world = np.dot(H_world_to_parent, np.dot(H_parent_to_joint, np.array([0,0,0,1]).T ) )
        
        joint_world_xyz = H_joint_in_world[0:3] / H_joint_in_world[3]
        
        if verbose:
            print("joint %s: x: %f, y: %f, z: %f" % (joint_name, joint_world_xyz[0],joint_world_xyz[1],joint_world_xyz[2]))
        return joint_world_xyz
        
        
    def get_robot_joint_world_pose(self, joints_name_list = ['KneePitch', 'HipPitch', 'HipRoll', 'HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll','LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']):
        
        # get joint x-y-z, debug those joints
        #joints_name_list = ['KneePitch', 'HipPitch', 'HipRoll', 'HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll','LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        
        joint_name_pos_dict = {}
        
        for name in joints_name_list:
            joint_world_xyz = self.get_joint_world_pose(name)
            if len(joint_world_xyz) != 0:
                # have got valid xyz
                # debug
                
                # lift all joints up by 1.0 meter, to clearly see joints
                #joint_world_xyz[2] += 1.0
                
                # lift all joints to the right by 1.0 m
                #joint_world_xyz[1] += 1.0
                
                # lift all joints to back by 1.0 m
                joint_world_xyz[0] += -1.0
                
                #p.addUserDebugText(name,joint_world_xyz,[1,0,0]) # show joint name at joint pos with red color
                joint_name_pos_dict[name] = joint_world_xyz
                
        self.joint_name_pos_dict = joint_name_pos_dict
        
        return joint_name_pos_dict
        
        
    def debug_robot_joints(self):
        '''
        if self.debug_id:
            # clean all previous drawing, draw again
            for item in self.debug_id:
                p.removeUserDebugItem(item)
            
            self.debug_id = []
        '''
        p.removeAllUserDebugItems()
        
        joint_name_pos_dict = self.joint_name_pos_dict
        
        # draw head
        p.addUserDebugLine(joint_name_pos_dict['HeadPitch'],joint_name_pos_dict['HeadYaw'], [0,1,0], 3.0)
        
        # draw body
        p.addUserDebugLine(joint_name_pos_dict['HeadYaw'],joint_name_pos_dict['HipRoll'], [0,1,0], 3.0)
        p.addUserDebugLine(joint_name_pos_dict['HipRoll'],joint_name_pos_dict['HipPitch'], [0,1,0], 3.0)
        p.addUserDebugLine(joint_name_pos_dict['HipPitch'],joint_name_pos_dict['KneePitch'], [0,1,0], 3.0)
        
        
        # draw left arm
        p.addUserDebugLine(joint_name_pos_dict['HeadYaw'],joint_name_pos_dict['LShoulderRoll'], [1,0,0], 3.0)
        
        #p.addUserDebugLine(joint_name_pos_dict['LShoulderRoll'],joint_name_pos_dict['LShoulderPitch'], [1,0,0], 3.0)
        #p.addUserDebugLine(joint_name_pos_dict['LShoulderPitch'],joint_name_pos_dict['LElbowYaw'], [1,0,0], 3.0)
        
        p.addUserDebugLine(joint_name_pos_dict['LShoulderRoll'],joint_name_pos_dict['LElbowYaw'], [1,0,0], 3.0)
        p.addUserDebugLine(joint_name_pos_dict['LElbowYaw'],joint_name_pos_dict['LElbowRoll'], [1,0,0], 3.0)
        p.addUserDebugLine(joint_name_pos_dict['LElbowRoll'],joint_name_pos_dict['LWristYaw'], [1,0,0], 3.0)
        
        
        # draw right arm
        p.addUserDebugLine(joint_name_pos_dict['HeadYaw'],joint_name_pos_dict['RShoulderRoll'], [1,0,0], 3.0)
        
        #p.addUserDebugLine(joint_name_pos_dict['RShoulderRoll'],joint_name_pos_dict['RShoulderPitch'], [1,0,0], 3.0)
        #p.addUserDebugLine(joint_name_pos_dict['RShoulderPitch'],joint_name_pos_dict['RElbowYaw'], [1,0,0], 3.0)
        
        p.addUserDebugLine(joint_name_pos_dict['RShoulderRoll'],joint_name_pos_dict['RElbowYaw'], [1,0,0], 3.0)
        p.addUserDebugLine(joint_name_pos_dict['RElbowYaw'],joint_name_pos_dict['RElbowRoll'], [1,0,0], 3.0)
        p.addUserDebugLine(joint_name_pos_dict['RElbowRoll'],joint_name_pos_dict['RWristYaw'], [1,0,0], 3.0)
        
    def get_angles_from_keypoints(self, key_points_pos_dict):
        '''
        calculate human joint angles based on detected keypoints
        the return value of this function cannot be used to control robot directly
        instead, we need to fit the relationship between human joint angles and the robot control angles

        Parameters
        ----------
        key_points_pos_dict : python dictionary
            store keypoint_name and keypoint_world_pose pair. The keypoint_name will be used to index the dictionary, and get the corresponding world [x,y,z] of the key point
            For example: hip_world_pose = key_points_pos_dict['Hip']
            
            The dictionary must include the following entries:
                'Hip'
                'Neck'
                'LShoulder'
                'LElbow'
                'LWrist'
                'RShoulder'
                'RElbow'
                'RWrist'

        Returns
        -------
        angle_dict : python dictionary
            store joint name and next joint coordinate in local frame (unit vector: [x, y, z]) in terms of local coordinate system of joints
            The following key will be available for indexing this dictionary:
                'LShoulder'
                'LElbow'
                'RShoulder'
                'RElbow'
                'Hip'
                'BaseRotate'
        '''
        
        # debug keypoints
        self.debug_keypoints(key_points_pos_dict)

        angle_dict = {}
        
        # get local coordinate systems for joints
        # i.e. coordinate system for 'LShoulder', 'LElbow', 'RShoulder', 'RElbow'
        
        # coordinate system for LShoulder:
        # unit y-axis in the same direction as the vector from neck to shoulder
        # unit x-axis is the unit vector in the direction of the cross product
        #             between unit y-axis and the vector from hip to neck
        # unit z-axis is the cross product of x-axis and y-axis
        
        Shoulder_reference_vec = key_points_pos_dict['Neck'] - key_points_pos_dict['Hip']
        LShoulder_y_direction_vec = key_points_pos_dict['LShoulder'] - key_points_pos_dict['Neck']
        
        LShoulder_coordinate = self.get_local_coordinate(LShoulder_y_direction_vec,Shoulder_reference_vec)
        
        # debug the LShoulder_coordinate
        self.debug_local_coordinate(key_points_pos_dict['LShoulder'], LShoulder_coordinate)

        LElbow_LShoulder_diff_T = (key_points_pos_dict['LElbow'] - key_points_pos_dict['LShoulder']).reshape(-1, 1)

        angle_dict["LShoulder"] = LShoulder_coordinate @ LElbow_LShoulder_diff_T

        angle_dict["LShoulder"] = angle_dict["LShoulder"].reshape(-1)/norm(angle_dict["LShoulder"])
        # self.debug_relative_orientation(np.array([0.5, 0, 1]), angle_dict["LShoulder"])
        
        # coordinate system for RShoulder:
        # unit y-axis in the same direction as the vector from neck to shoulder
        # unit x-axis is the unit vector in the direction of the cross product
        #             between unit y-axis and the vector from hip to neck
        # unit z-axis is the cross product of x-axis and y-axis
        
        RShoulder_y_direction_vec = key_points_pos_dict['RShoulder'] - key_points_pos_dict['Neck']
        
        RShoulder_coordinate = self.get_local_coordinate(RShoulder_y_direction_vec,Shoulder_reference_vec)
        
        # debug the LShoulder_coordinate
        self.debug_local_coordinate(key_points_pos_dict['RShoulder'], RShoulder_coordinate)

        RElbow_RShoulder_diff_T = (key_points_pos_dict['RElbow'] - key_points_pos_dict['RShoulder']).reshape(-1, 1)

        angle_dict["RShoulder"] = RShoulder_coordinate @ RElbow_RShoulder_diff_T
        angle_dict["RShoulder"] = angle_dict["RShoulder"].reshape(-1)/norm(angle_dict["RShoulder"])
        # self.debug_relative_orientation(np.array([0.5, 0, 1]), angle_dict["LShoulder"])
        
        # coordinate system for LElbow
        # unit y-axis in the same direction as the vector from neck to Lshoulder
        # unit x-axis is the unit vector in the direction of the cross product
        #             between unit y-axis and the vector from neck to LShoulder
        # unit z-axis is the cross product of x-axis and y-axis
        
        LElbow_reference_vec = key_points_pos_dict['LShoulder'] - key_points_pos_dict['Neck']
        LElbow_y_direction_vec = key_points_pos_dict['LElbow'] - key_points_pos_dict['LShoulder']
        
        LElbow_coordinate = self.get_local_coordinate(LElbow_y_direction_vec, LElbow_reference_vec)
        
        # debug the LElbow_coordinate
        self.debug_local_coordinate(key_points_pos_dict['LElbow'], LElbow_coordinate)

        LWrist_LElbow_diff_T = (key_points_pos_dict['LWrist'] - key_points_pos_dict['LElbow']).reshape(-1, 1)

        angle_dict["LElbow"] = LElbow_coordinate @ LWrist_LElbow_diff_T
        angle_dict["LElbow"] = angle_dict["LElbow"].reshape(-1)/norm(angle_dict["LElbow"])
        # self.debug_relative_orientation(np.array([0.5, 0, 1]), angle_dict["LElbow"])
        
        # coordinate system for RElbow
        # unit y-axis in the same direction as the vector from neck to Rshoulder
        # unit x-axis is the unit vector in the direction of the cross product
        #             between unit y-axis and the vector from neck to RShoulder
        # unit z-axis is the cross product of x-axis and y-axis
        
        RElbow_reference_vec = key_points_pos_dict['RShoulder'] - key_points_pos_dict['Neck']
        RElbow_y_direction_vec = key_points_pos_dict['RElbow'] - key_points_pos_dict['RShoulder']
        
        RElbow_coordinate = self.get_local_coordinate(RElbow_y_direction_vec, RElbow_reference_vec)
        
        # debug the RElbow_coordinate
        self.debug_local_coordinate(key_points_pos_dict['RElbow'], RElbow_coordinate)

        RWrist_RElbow_diff_T = (key_points_pos_dict['RWrist'] - key_points_pos_dict['RElbow']).reshape(-1, 1)

        angle_dict["RElbow"] = RElbow_coordinate @ RWrist_RElbow_diff_T
        angle_dict["RElbow"] = angle_dict["RElbow"].reshape(-1)/norm(angle_dict["RElbow"])
        # self.debug_relative_orientation(np.array([0.5, 0, 1]), angle_dict["RElbow"])

        Hip_Head_vec = (key_points_pos_dict["Neck"] - key_points_pos_dict["Hip"]).reshape(-1, 1)
        angle_dict["Hip"] = self.base_coodinate @ Hip_Head_vec
        angle_dict["Hip"] = angle_dict["Hip"].reshape(-1)/norm(angle_dict["Hip"])
        self.debug_relative_orientation(np.array([0.5, 0, 1]), angle_dict["Hip"])
        
        return angle_dict
    
    def get_move_command(self, key_points_pos_dict):

        # calculate relative direction of vector from right shoulder to left shoulder in base link
        LRShoulder_vec = (key_points_pos_dict["LShoulder"] - key_points_pos_dict["RShoulder"]).reshape(-1, 1)
        direction = self.base_coodinate @ LRShoulder_vec
        direction = direction.reshape(-1)/norm(direction)

        theta = atan2(direction[1], direction[0]) - np.pi/2

        translation = (key_points_pos_dict["Hip"] - self.base_origin).reshape(-1, 1)
        translation_vec = self.base_coodinate @ translation

        x, y = translation_vec[0][0], translation_vec[1][0]

        # self.debug_relative_orientation(np.array([0.5, 0, 1]), angle_dict["BaseRotate"])

        print("moveto command:", x, y, theta)
        self.robot_virtual.moveTo(x, y, theta, frame=self.robot_virtual.FRAME_WORLD, _async=True)

        return [x, y, theta]

    
    def debug_keypoints(self, key_points_pos_dict):
        '''
        

        Parameters
        ----------
        key_points_pos_dict : python dictionary
            store keypoint_name and keypoint_world_pose pair. The keypoint_name will be used to index the dictionary, and get the corresponding world [x,y,z] of the key point
            For example: hip_world_pose = key_points_pos_dict['Hip']
            
            The dictionary must include the following entries:
                'Hip'
                'Neck'
                'LShoulder'
                'LElbow'
                'LWrist'
                'RShoulder'
                'RElbow'
                'RWrist'

        Returns
        -------
        None.

        '''
        p.removeAllUserDebugItems()
        # hip to neck
        p.addUserDebugLine(key_points_pos_dict['Hip'],key_points_pos_dict['Neck'], [1,0,0], 3.0)
        
        # neck to arms
        p.addUserDebugLine(key_points_pos_dict['Neck'],key_points_pos_dict['LShoulder'], [1,0,0], 3.0)
        p.addUserDebugLine(key_points_pos_dict['LShoulder'],key_points_pos_dict['LElbow'], [1,0,0], 3.0)
        p.addUserDebugLine(key_points_pos_dict['LElbow'],key_points_pos_dict['LWrist'], [1,0,0], 3.0)
        p.addUserDebugLine(key_points_pos_dict['Neck'],key_points_pos_dict['RShoulder'], [1,0,0], 3.0)
        p.addUserDebugLine(key_points_pos_dict['RShoulder'],key_points_pos_dict['RElbow'], [1,0,0], 3.0)
        p.addUserDebugLine(key_points_pos_dict['RElbow'],key_points_pos_dict['RWrist'], [1,0,0], 3.0)
        
        pass
    
    def debug_local_coordinate(self, origin, Coordinate_local):
        '''
        

        Parameters
        ----------
        origin : np.array(), 1*3
            world pose of the origin of the local coordinate system.
        Coordinate_local : np.array(), 3*3
            the local coordinate vectors in world coordinate
            the row vectors for Coordinate_local are unit_x = Coordinate_local[0,:], unit_y = Coordinate_local[1,:], unit_z = Coordinate_local[2,:]
.

        Returns
        -------
        None.

        '''
        unit_x = Coordinate_local[0,:]
        unit_y = Coordinate_local[1,:]
        unit_z = Coordinate_local[2,:]
        
        factor = 0.1
        
        # debug local x, in red
        p.addUserDebugLine(origin, origin + unit_x * factor, [1,0,0], 6.0)
        
        # debug local y, in green
        p.addUserDebugLine(origin, origin + unit_y * factor, [0,1,0], 6.0)
        
        # debug local z, in blue
        p.addUserDebugLine(origin, origin + unit_z * factor, [0,0,1], 6.0)
    
    def debug_relative_orientation(self, origin, direction):
        '''
        

        Parameters
        ----------
        origin: np.array(), 1*3
            origin of the relative orientation
        direction : np.array(), 1*3
            the relative orientation of the next link
.

        Returns
        -------
        None.

        '''

        unit_x = np.array([1, 0, 0])
        unit_y = np.array([0, 1, 0])
        unit_z = np.array([0, 0, 1])
        
        factor = 0.1
        
        # debug x axis
        p.addUserDebugLine(origin, origin + unit_x * factor, [1,0,0], 6.0)
        
        # debug y axis
        p.addUserDebugLine(origin, origin + unit_y * factor, [0,1,0], 6.0)
        
        # debug z axis
        p.addUserDebugLine(origin, origin + unit_z * factor, [0,0,1], 6.0)

        # debug relative orientation
        p.addUserDebugLine(origin, origin + direction * 2 * factor, [0,1,1], 6.0)
    
    def get_local_coordinate(self, y_direction_vec, reference_vec):
        '''
        

        Parameters
        ----------
        y_direction_vec : np.array()
            the vector in the same direction as y-axis, not necessarily unit vector.
        reference_vec : np.array()
            the vector used to calculate x-axis by np.cross(y,unit_reference_vec).
            Special case: cross product got 0 vector, 
            solution: add small variance to unit_reference_vec and calculate again

        Returns
        -------
        Coordinate_local : np.array(), 3*3
            the local coordinate vectors in world coordinate
            the row vectors for Coordinate_local are unit_x = Coordinate_local[0,:], unit_y = Coordinate_local[1,:], unit_z = Coordinate_local[2,:]

        '''
        # create return value
        Coordinate_local = np.eye(3)
        
        # get unit_y
        unit_y = y_direction_vec / np.linalg.norm(y_direction_vec) # normalize y
        
        # get x_axis by cross product
        unit_reference_vec = reference_vec / np.linalg.norm(reference_vec)
        unit_x = np.cross(unit_y, unit_reference_vec)
        
        # avoid special case, i.e. cross product get zero vector
        if unit_x[0] == 0.0 and unit_x[1] == 0.0 and unit_x[2] == 0.0:
            random_value = np.random.rand(3)
            unit_y += random_value / np.linalg.norm(random_value) * 0.01
            unit_x = np.cross(unit_y, unit_reference_vec)
        
        # normalize unit_x
        unit_x = unit_x / np.linalg.norm(unit_x)
        
        # get unit_z
        unit_z = np.cross(unit_x, unit_y)
        
        Coordinate_local[0,:] = unit_x
        Coordinate_local[1,:] = unit_y
        Coordinate_local[2,:] = unit_z
        
        return Coordinate_local
    
def fake_human_keypoints(joint_name_pos_dict):
    '''
    get fake human keypoints from robot joint pose

    Parameters
    ----------
    joint_name_pos_dict : python dictionary
        store world pose of robot joints. The following keys should be available
            'KneePitch', 
            'HipPitch', 
            'HipRoll', 
            'HeadYaw', 
            'HeadPitch', 
            'LShoulderPitch', 
            'LShoulderRoll',
            'LElbowYaw', 
            'LElbowRoll', 
            'LWristYaw', 
            'RShoulderPitch', 
            'RShoulderRoll', 
            'RElbowYaw', 
            'RElbowRoll', 
            'RWristYaw'

    Returns
    -------
    key_points_pos_dict : python dictionary
            store keypoint_name and keypoint_world_pose pair. The keypoint_name will be used to index the dictionary, and get the corresponding world [x,y,z] of the key point
            For example: hip_world_pose = key_points_pos_dict['Hip']
            
            The dictionary must include the following entries:
                'Hip'
                'Neck'
                'LShoulder'
                'LElbow'
                'LWrist'
                'RShoulder'
                'RElbow'
                'RWrist'

    '''
    
    # create return value
    key_points_pos_dict = {}
    
    # make up human skeleton
    key_points_pos_dict['Hip'] = joint_name_pos_dict['HipRoll']
    key_points_pos_dict['Neck'] = joint_name_pos_dict['HeadYaw']
    key_points_pos_dict['LShoulder'] = joint_name_pos_dict['LShoulderRoll']
    key_points_pos_dict['LElbow'] = joint_name_pos_dict['LElbowRoll']
    key_points_pos_dict['LWrist'] = joint_name_pos_dict['LWristYaw']
    key_points_pos_dict['RShoulder'] = joint_name_pos_dict['RShoulderRoll']
    key_points_pos_dict['RElbow'] = joint_name_pos_dict['RElbowRoll']
    key_points_pos_dict['RWrist'] = joint_name_pos_dict['RWristYaw']
    
    
    return key_points_pos_dict

def extract_human_keypoints(joint_name_pos_dict):
    '''
    extra human keypoints from full keypoints

    Parameters
    ----------
    joint_name_pos_dict : python dictionary
        store world pose of robot joints. The following keys should be available
            "HipCenter", "Spine", "ShoulderCenter", "Head", 
            "LShoulder", "LElbow", "LWrist", "LHand", 
            "RShoulder", "RElbow", "RWrist", "RHand", 
            "LHip", "LKnee", "LAnkle", "LFoot", 
            "RHip", "RKnee", "RAnkle", "RFoot"

    Returns
    -------
    key_points_pos_dict : python dictionary
            store keypoint_name and keypoint_world_pose pair. The keypoint_name will be used to index the dictionary, and get the corresponding world [x,y,z] of the key point
            For example: hip_world_pose = key_points_pos_dict['Hip']
            
            The dictionary must include the following entries:
                'Hip'
                'Neck'
                'LShoulder'
                'LElbow'
                'LWrist'
                'RShoulder'
                'RElbow'
                'RWrist'
                'LHip'
                'RHip'
                'LKnee'
                'RKnee'
                'LAnkle'
                'RAnkle'
    '''
    
    # create return value
    key_points_pos_dict = {}
    
    # make up human skeleton
    key_points_pos_dict['Hip']          = joint_name_pos_dict['HipCenter']
    key_points_pos_dict['Neck']         = joint_name_pos_dict['Head']
    key_points_pos_dict['LShoulder']    = joint_name_pos_dict['LShoulder']
    key_points_pos_dict['LElbow']       = joint_name_pos_dict['LElbow']
    key_points_pos_dict['LWrist']       = joint_name_pos_dict['LWrist']
    key_points_pos_dict['RShoulder']    = joint_name_pos_dict['RShoulder']
    key_points_pos_dict['RElbow']       = joint_name_pos_dict['RElbow']
    key_points_pos_dict['RWrist']       = joint_name_pos_dict['RWrist']

    key_points_pos_dict['LHip']     = joint_name_pos_dict['LHip']
    key_points_pos_dict['RHip']     = joint_name_pos_dict['RHip']
    key_points_pos_dict['LKnee']    = joint_name_pos_dict['LKnee']
    key_points_pos_dict['RKnee']    = joint_name_pos_dict['RKnee']
    key_points_pos_dict['LAnkle']   = joint_name_pos_dict['LAnkle']
    key_points_pos_dict['RAnkle']   = joint_name_pos_dict['RAnkle']
    
    
    return key_points_pos_dict
        
if __name__ == "__main__":
    
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True) 
    
    single_robot = SingleRobot(simulation_manager, client, save_joint_limits = False)

    with open('kinectData/skeletonTest.json', 'r') as f:
        skeletonDataList = json.load(f)

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
        
        #joint_name = ['KneePitch', 'HipPitch', 'HipRoll', 'HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll','LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        
        # joint_name = ['LShoulderPitch', 'LShoulderRoll','LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        
        joint_name = ['HipPitch', 'HipRoll']

        # joint_name = ['LShoulderPitch', 'LShoulderRoll',
        #               'LElbowYaw', 'LElbowRoll', 
        #               'RShoulderPitch', 'RShoulderRoll', 
        #               'RElbowYaw', 'RElbowRoll']

        joint_name_list = []
        joint_value_list = []
        speed_percentage_list = []
        
        for name in joint_name:
            joint_name_list.append(name)
            joint_value_list.append( float(joint_limits[name]['lower_limit']) )
            speed_percentage_list.append(1.0)
            
        single_robot.setPosture()
        
        time.sleep(5.0)
        
        while True:
            #single_robot.joint_control(joint_name_list,joint_value_list,speed_percentage_list)
            
            #single_robot.get_joint_world_pose('LShoulderRoll')
            
            # control joints one by one 
            
            # debug robot joints
            joint_name_pos_dict = single_robot.get_robot_joint_world_pose()
            # if skeletonDataList != []:
            #     joint_name_pos_dict = skeletonDataList.pop(0)
            # else:
            #     print("No skeleton data, exit.")
            #     break
            
            #single_robot.debug_robot_joints()
            
            # calculate and debug local coordinate systems
            key_points_pos_dict = fake_human_keypoints(joint_name_pos_dict)
            angle_dict = single_robot.get_angles_from_keypoints(key_points_pos_dict)
            # print("angle_dict", angle_dict)
            
            for name in joint_name:
                
                joint_values = np.linspace(float(joint_limits[name]['lower_limit']), 
                                           float(joint_limits[name]['upper_limit']), 20)
                print("---")
                print("Joint: %s"%(name))
                for value in joint_values:
                    print(value)
                    single_robot.joint_control([name],[value],[1.0])
                    
                    # debug robot joints
                    joint_name_pos_dict = single_robot.get_robot_joint_world_pose()
                    # single_robot.debug_robot_joints()
                    
                    # calculate and debug local coordinate systems
                    key_points_pos_dict = fake_human_keypoints(joint_name_pos_dict)
                    single_robot.get_angles_from_keypoints(key_points_pos_dict)
                    
                    time.sleep(0.1)
                
                # reset robot pose to stand
                single_robot.setPosture()
                
                # debug robot joints
                joint_name_pos_dict = single_robot.get_robot_joint_world_pose()
                #single_robot.debug_robot_joints()
                
                # calculate and debug local coordinate systems
                key_points_pos_dict = fake_human_keypoints(joint_name_pos_dict)
                single_robot.get_angles_from_keypoints(key_points_pos_dict)
                
                time.sleep(2.0)

            
            pass
    except KeyboardInterrupt:
        simulation_manager.stopSimulation(client)
        pass
    finally:
        simulation_manager.stopSimulation(client)