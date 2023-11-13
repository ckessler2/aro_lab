#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import pinocchio as pin 
import numpy as np
from numpy.linalg import pinv,inv,norm,svd,eig
from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits
from config import LEFT_HOOK, RIGHT_HOOK, LEFT_HAND, RIGHT_HAND, EPSILON
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET

from tools import setcubeplacement

from scipy.optimize import fmin_bfgs



def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
    # Import bfgs
    from scipy.optimize import fmin_bfgs
    
    # Define left/right hand targets based on cubetarget
    target_L = cubetarget + np.array([[0,0,0,0],[0,0,0,0.05],[0,0,0,0],[0,0,0,0]])
    target_R = cubetarget - np.array([[2,0,0,0],[0,2,0,0.05],[0,0,0,0],[0,0,0,0]])
    
    # Cost function, summing squares of both euclidian distances (hand to target)
    def cost(q,robot,target_R,target_L):
        pin.framesForwardKinematics(robot.model,robot.data,q)
        eff = np.asarray(robot.data.oMf[robot.model.getFrameId(RIGHT_HAND)])
        eff2 = np.asarray(robot.data.oMf[robot.model.getFrameId(LEFT_HAND)])
        return np.linalg.norm(eff - target_R)**2 + np.linalg.norm(eff2 - target_L)**2

    # Solve using BFGS
    qopt_bfgs = fmin_bfgs(cost, qcurrent, args=(robot,target_R,target_L),gtol=0.002)
    # print('\n *** Optimal configuration from BFGS = %s \n\n\n\n' % qopt_bfgs)
    
    if viz is not None:
        from setup_meshcat import updatevisuals
        # updatevisuals(viz, robot, cube, qopt_bfgs)
    return qopt_bfgs, True
          

    
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from setup_meshcat import updatevisuals
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()
    
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    updatevisuals(viz, robot, cube, q0)
    
print('success')