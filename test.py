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

q_start = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
q_final = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET, viz)

# from tools import collision,jointlimitsviolated
# # print("Collision check: " + str(collision(robot,q_opt)))
# # print("Joint Violation check: " + str(jointlimitsviolated(robot,q_opt)))

# max_x = np.asarray(CUBE_PLACEMENT_TARGET)[0,3] + 0.05
# max_y = np.asarray(CUBE_PLACEMENT_TARGET)[1,3] + 0.05
# min_x = np.asarray(CUBE_PLACEMENT)[0,3] - 0.05
# min_y = np.asarray(CUBE_PLACEMENT)[1,3] - 0.05
# max_z = 1.2
# min_z = 0.93

# steps = 300
# configs = []
# discretisationsteps_newconf = 10 #To tweak later on
# discretisationsteps_validedge = 10 #To tweak later on
# delta = 300 #To tweak later on

# def Rand_Cube_Conf():
#     from tools import collision,jointlimitsviolated,distanceToObstacle
#     import time
#     while True:
#         # generate randome cube xyz within limits
#         x = (np.random.default_rng().random()* (max_x-min_x)) + min_x
#         y = (np.random.default_rng().random()* (max_y-min_y)) + min_y
#         z = (np.random.default_rng().random()* (max_z-min_z)) + min_z
#         rand_cube_pos = np.array([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])
#         # calculate inverse geometry
#         q1 = computeqgrasppose(robot, q, cube, rand_cube_pos, viz)
#         # if collision/jointlimit/obstacle are not violated, return config
#         if not (collision(robot, q1) or jointlimitsviolated(robot, q1) or distanceToObstacle(robot, q1)<0.01): 
#             time.sleep(0.000001)
#             updatevisuals(viz, robot, cube, q1)
#             return q1

# def lerp(q0,q1,t):    
#     return q0 * (1 - t) + q1 * t

# def distance(q1,q2):    
#     '''Return the euclidian distance between two configurations'''
#     return np.linalg.norm(q2-q1)
        
# def NEAREST_VERTEX(G,q_rand):
#     '''returns the index of the Node of G with the configuration closest to q_rand  '''
#     min_dist = 10e4
#     idx=-1
#     for (i,node) in enumerate(G):
#         dist = distance(node[1],q_rand) 
#         if dist < min_dist:
#             min_dist = dist
#             idx = i
#     return idx

# def NEW_CONF(q_near,q_rand,discretisationsteps, delta_q = None):
#     '''Return the closest configuration q_new such that the path q_near => q_new is the longest
#     along the linear interpolation (q_near,q_rand) that is collision free and of length <  delta_q'''
#     q_end = q_rand.copy()
#     dist = distance(q_near, q_rand)
#     if delta_q is not None and dist > delta_q:
#         #compute the configuration that corresponds to a path of length delta_q
#         q_end = lerp(q_near,q_rand,delta_q/dist)
#         dist = delta_q
#     dt = dist / discretisationsteps
#     for i in range(1,discretisationsteps):
#         q = lerp(q_near,q_end,dt*i)
#         if collision(robot,q):
#             return lerp(q_near,q_end,dt*(i-1))
#     return q_end


# def VALID_EDGE(q_new,q_goal,discretisationsteps):
#     return np.linalg.norm(q_goal -NEW_CONF(q_new, q_goal,discretisationsteps)) < 0.0001
# def ADD_EDGE_AND_VERTEX(G,parent,q):
#     G += [(parent,q)]

# def rrt(q_init, q_goal, steps, delta_q):
#     G = [(None,q_init)]
#     for _ in range(steps):
#         print('step ' + str(_))
#         q_rand = Rand_Cube_Conf()
#         q_near_index = NEAREST_VERTEX(G,q_rand)
#         q_near = G[q_near_index][1]        
#         q_new = NEW_CONF(q_near,q_rand,discretisationsteps_newconf, delta_q = None)    
#         ADD_EDGE_AND_VERTEX(G,q_near_index,q_new)
#         if VALID_EDGE(q_new,q_goal,discretisationsteps_validedge):
#             print ("Path found!")
#             ADD_EDGE_AND_VERTEX(G,len(G)-1,q_goal)
#             return G, True
#     print("path not found")
#     return G, False

# def getpath(G):
#     path = []
#     node = G[-1]
#     while node[0] is not None:
#         path = [node[1]] + path
#         node = G[node[0]]
#     path = [G[0][1]] + path
#     return path

# from math import ceil
# from time import sleep

# def displayedge(q0,q1,vel=2.): #vel in sec.    
#     '''Display the path obtained by linear interpolation of q0 to q1 at constant velocity vel'''
#     dist = distance(q0,q1)
#     duration = dist / vel    
#     nframes = ceil(48. * duration)
#     f = 1./48.
#     for i in range(nframes-1):
#         viz.display(lerp(q0,q1,float(i)/nframes))
#         sleep(f)
#     viz.display(q1)
#     sleep(f)
    
# def displaypath(path):
#     for q0, q1 in zip(path[:-1],path[1:]):
#         displayedge(q0,q1)
    
# G, foundpath = rrt(q_start, q_final, steps, delta)

from path import computepath

qinit, qgoal, G, foundpath = computepath(robot,cube,viz, q_start,q_final,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

# path = foundpath and getpath(G) or [] 
# displaypath(path)

# import time
# for n in range(len(G)-1):
#     updatevisuals(viz, robot, cube, G[n][1])
#     time.sleep(0.5)