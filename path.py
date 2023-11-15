#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
from numpy.linalg import pinv

from config import LEFT_HAND, RIGHT_HAND
import time

#returns a collision free path from qinit to qgoal under grasping constraints
#the path is expressed as a list of configurations
def computepath(robot,cube,qinit,qgoal,cubeplacementq0, cubeplacementqgoal, viz):
    from tools import collision,jointlimitsviolated
    from setup_meshcat import updatevisuals

    max_x = np.asarray(cubeplacementqgoal)[0,3] + 0.05
    max_y = np.asarray(cubeplacementqgoal)[1,3] + 0.05
    min_x = np.asarray(cubeplacementq0)[0,3] - 0.05
    min_y = np.asarray(cubeplacementq0)[1,3] - 0.05
    max_z = 1.5
    min_z = 0.93

    steps = 500
    configs = []
    discretisationsteps_newconf = 10 #To tweak later on
    discretisationsteps_validedge = 10 #To tweak later on
    delta = 300 #To tweak later on

    def Rand_Cube_Conf():
        from tools import collision,jointlimitsviolated,distanceToObstacle
        from inverse_geometry import computeqgrasppose
        import time
        while True:
            restart_loop = False
            # generate random cube xyz within limits
            x = (np.random.default_rng().random()* (max_x-min_x)) + min_x
            y = (np.random.default_rng().random()* (max_y-min_y)) + min_y
            z = (np.random.default_rng().random()* (max_z-min_z)) + min_z
            if (x-0.33)<0.1 and (y+0.3)<0.3 and z<1.3:
                restart_loop = True
            rand_cube_pos = np.array([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])
            # calculate inverse geometry
            q1,successinit = computeqgrasppose(robot, qinit, cube, rand_cube_pos, True)
            # if collision/jointlimit/obstacle are not violated, return config
            if restart_loop == True:
                continue
            elif (collision(robot, q1) or jointlimitsviolated(robot, q1) or distanceToObstacle(robot, q1)<0.01): 
                time.sleep(0.000001)
                if viz != None:
                    updatevisuals(viz, robot, cube, q1)
                return q1
            else:
                continue

    def lerp(q0,q1,t):    
        return q0 * (1 - t) + q1 * t

    def distance(q1,q2):    
        '''Return the euclidian distance between two configurations'''
        return np.linalg.norm(q2-q1)
            
    def NEAREST_VERTEX(G,q_rand):
        '''returns the index of the Node of G with the configuration closest to q_rand  '''
        min_dist = 10e4
        idx=-1
        for (i,node) in enumerate(G):
            dist = distance(node[1],q_rand) 
            if dist < min_dist:
                min_dist = dist
                idx = i
        return idx

    def NEW_CONF(q_near,q_rand,discretisationsteps, delta_q = None):
        '''Return the closest configuration q_new such that the path q_near => q_new is the longest
        along the linear interpolation (q_near,q_rand) that is collision free and of length <  delta_q'''
        q_end = q_rand.copy()
        dist = distance(q_near, q_rand)
        if delta_q is not None and dist > delta_q:
            #compute the configuration that corresponds to a path of length delta_q
            q_end = lerp(q_near,q_rand,delta_q/dist)
            dist = delta_q
        dt = dist / discretisationsteps
        for i in range(1,discretisationsteps):
            q = lerp(q_near,q_end,dt*i)
            if collision(robot,q):
                return lerp(q_near,q_end,dt*(i-1))
        return q_end


    def VALID_EDGE(q_new,q_goal,discretisationsteps):
        return np.linalg.norm(q_goal -NEW_CONF(q_new, q_goal,discretisationsteps)) < 0.001
    def ADD_EDGE_AND_VERTEX(G,parent,q):
        G += [(parent,q)]

    def rrt(q_init, q_goal, steps, delta_q):
        G = [(None,q_init)]
        for _ in range(steps):
            print('step ' + str(_))
            q_rand = Rand_Cube_Conf()
            q_near_index = NEAREST_VERTEX(G,q_rand)
            q_near = G[q_near_index][1]        
            q_new = NEW_CONF(q_near,q_rand,discretisationsteps_newconf, delta_q = None)    
            ADD_EDGE_AND_VERTEX(G,q_near_index,q_new)
            if VALID_EDGE(q_new,q_goal,discretisationsteps_validedge):
                print ("Path found!")
                ADD_EDGE_AND_VERTEX(G,len(G)-1,q_goal)
                return G, True
        print("path not found")
        return G, False

    def getpath(G):
        path = []
        node = G[-1]
        while node[0] is not None:
            path = [node[1]] + path
            node = G[node[0]]
        path = [G[0][1]] + path
        return path

    from math import ceil
    from time import sleep

    def displayedge(q0,q1,vel=2.): #vel in sec.    
        '''Display the path obtained by linear interpolation of q0 to q1 at constant velocity vel'''
        dist = distance(q0,q1)
        duration = dist / vel    
        nframes = ceil(48. * duration)
        f = 1./48.
        for i in range(nframes-1):
            viz.display(lerp(q0,q1,float(i)/nframes))
            sleep(f)
        viz.display(q1)
        sleep(f)
        
    def displaypath(path):
        for q0, q1 in zip(path[:-1],path[1:]):
            displayedge(q0,q1)
        
    G, foundpath = rrt(qinit, qgoal, steps, delta)
    
    path = foundpath and getpath(G) or [] 
    if viz != None:
        displaypath(path)   
    return [getpath(G)], foundpath
    pass


def displaypath(robot,path,dt,viz):
    for q in path:
        viz.display(q)
        time.sleep(dt)


if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import cubeplacementq0, cubeplacementqgoal
    from inverse_geometry import computeqgrasppose
    
    robot, cube, viz = setupwithmeshcat()
    
    
    q = robot.q0.copy()
    q0,successinit = computeqgrasppose(robot, q, cube, cubeplacementq0, viz)
    qe,successend = computeqgrasppose(robot, q, cube, cubeplacementqgoal,  viz)
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path = computepath(q0,qe,cubeplacementq0, cubeplacementqgoal)
    
    displaypath(robot,path,dt=0.5,viz=viz) #you ll probably want to lower dt
    
