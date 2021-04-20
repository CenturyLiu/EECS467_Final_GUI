#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  2 15:37:40 2021

@author: shijiliu
"""


import sys
from qibullet import SimulationManager
from configobj import ConfigObj

class Robot_joint_test(object):
    def __init__(self, simulation_manager, client, robot_choice = 'pepper', save_joint_limits = False):
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
        
        self.get_joint_names()
        

    def __del__(self):
        self.del_virtual(self.robot_virtual)
        self.simulation_manager.stopSimulation(self.client)
        
        
    def get_joint_names(self):
        self.joint_names = []
        for name in self.robot_virtual.joint_dict.keys():
            self.joint_names.append(name)
        
        return self.joint_names
    
    def get_joint_upper_lower_limit(self):
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
        else:
            print(self.joint_limits)
        
        
if __name__ == "__main__":
    
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True) 
    
    robot_joint_test = Robot_joint_test(simulation_manager, client,robot_choice = 'nao', save_joint_limits = True)
    
    try:
        robot_joint_test.get_joint_upper_lower_limit()
            
    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)