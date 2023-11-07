#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 11 14:58:17 2023

@author: root
"""
import numpy as np

# !meshcat-server
from tools import setupwithmeshcat
robot, cube, viz = setupwithmeshcat(url="tcp://127.0.0.1:6000")

## 1: Print setup parameters (tutorial)
from config import MESHCAT_URL
print(MESHCAT_URL)
print(robot.model)
from tools import collision
collision(robot, robot.q0)

## 2: Configuration functions (tutorial)
from config import LEFT_HAND, RIGHT_HAND
print ("Left hand joint name: ", LEFT_HAND)
import pinocchio as pin
q = robot.q0.copy()

#update the frame positions in robot.data given q
pin.framesForwardKinematics(robot.model,robot.data,q)

#now let's print the placement attached to the right hand
print ("Left hand joint placement: ")
pin.computeJointJacobians(robot.model,robot.data,q)
frameid = robot.model.getFrameId(LEFT_HAND)
oMframe = robot.data.oMf[frameid] 
print(oMframe)


# print ("Left hand joint placement: ")
# frameid = robot.model.getFrameId(LEFT_HAND)
# oMframe = robot.data.oMf[frameid] 
# print(oMframe)




## 3: Helper functions (tutorial)
from config import LEFT_HOOK, RIGHT_HOOK, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
from tools import getcubeplacement, setcubeplacement
from setup_meshcat import updatevisuals
from inverse_geometry import computeqgrasppose

#We can access the current cube position using
oMcube  = getcubeplacement(cube) #origin of the cube
oMcubeL = getcubeplacement(cube, LEFT_HOOK) #placement of the left hand hook
oMcubeR = getcubeplacement(cube, RIGHT_HOOK) #placement of the right hand hook



#the cube position is updated using the following function:
setcubeplacement(robot, cube, CUBE_PLACEMENT)
#to update the frames for both the robot and the cube you can call
updatevisuals(viz, robot, cube, q)

q_opt = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET, viz=True)

updatevisuals(viz, robot, cube, q_opt)

from tools import collision

print("Collision check: " + str(collision(robot,q_opt)))
