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

from path import computepath

G = computepath(robot,cube, q_start[0],q_final[0],CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET,viz)
G = G[0]

def Pos_Interp(a,b,x):
    amp = b-a
    ext = 1.0
    pos1 = (amp/(ext*ext))*2 * x * x
    pos2 = amp - ((amp/(ext*ext))*2 * (x-ext) * (x-ext))
    vel1 = 4 * amp * x / (ext * ext)
    vel2 = 4 * amp * (ext - x) / (ext * ext)
    acc1 = 4 * amp / (ext * ext)
    acc2 = -4 * amp / (ext * ext)
    if x < 0.5:
        return pos1, vel1, acc1
    else:
        return pos2, vel2, acc2

def Traj_Interp1(G, t, t_total):
    # G = G[0]
    traj_sec_time = t_total/(len(G)-1)
    n = int(np.floor(t/traj_sec_time))
    # print('section ' + str(n))
    sec_start_time = n*traj_sec_time
    sec_end_time = (n+1)*traj_sec_time
    # print('starts at ' + str(sec_start_time) + ', ends at ' + str(sec_end_time))
    sec_start_config = G[n]
    try:
        sec_end_config = G[n+1]
    except IndexError:
        config_pos, config_vel, config_acc = G[n]*0,G[n]*0,G[n]*0
        return config_pos, config_vel, config_acc
        
    # print('start config = ' + str(sec_start_config))
    # print('end config = ' + str(sec_end_config))
    config_pos=[]
    config_vel=[]
    config_acc=[]
    step_time = t - (n*traj_sec_time)
    step_time_frac = step_time/traj_sec_time

    for i in range(len(G[n])):
        # print(i)
        config_pos.append(Pos_Interp(G[n][i], G[n+1][i], step_time_frac)[0])
        config_vel.append(Pos_Interp(G[n][i], G[n+1][i], step_time_frac)[1]) 
        config_acc.append(Pos_Interp(G[n][i], G[n+1][i], step_time_frac)[2]) 
    return config_pos, config_vel, config_acc

import numpy as np
import matplotlib.pyplot as plot
x = np.arange(0, 10, 0.01);
amplitude   = np.sin(x)

pos1_1 = []
vel1_1 = []
acc1_1 = []
totaltime = 10
traj_sec_time = totaltime/(len(G)-1)

joint = 5
pos1_1 = []
vel1_1 = []
acc1_1 = []

for i in x:
    n = int(np.floor(i/traj_sec_time))
    if n == len(G):
        n = len(G)-1

    pos1_1.append(G[n][joint] + Traj_Interp1(G,i,10)[0][joint])
    vel1_1.append(Traj_Interp1(G,i,10)[1][joint])
    acc1_1.append(Traj_Interp1(G,i,10)[2][joint])

plot.plot(x, pos1_1)
plot.plot(x, vel1_1)
plot.plot(x, acc1_1)
plot.xlabel('Time')
plot.ylabel('Position, Velocity, Acceleration')
plot.title('Position, Velocity, and Acceleration vs Time (Joint ' + str(joint) + ')')

plot.show()
