#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np

from bezier import Bezier
    
# in my solution these gains were good enough for all joints but you might want to tune this.
Kp =200.               # proportional gain (P of PD)
Kv = 0.6 * np.sqrt(Kp)   # derivative gain (D of PD)

def controllaw(sim, robot, trajs, tcurrent, cube):
    # trajs = path
    q, vq = sim.getpybulletstate()
    traj_sec_time = 4/(len(trajs)-1)
    n = int(np.floor(tcurrent/traj_sec_time))
    if n == len(path):
        n = len(path)-1
    # pos1_1.append(path[n][joint] + Traj_Interp1(path,i,total_time)[0][joint])
    #TODO 
    # torques = [0.0 for _ in sim.bulletCtrlJointsInPinOrder]
    # expected_torques = [10*(Traj_Interp1(trajs,tcurrent,10)[2][joint]) for joint in range(15)]
    expected_positions = [(Traj_Interp1(trajs,tcurrent,4)[0][joint]) for joint in range(15)] + trajs[n]
    expected_velocities = [(Traj_Interp1(trajs,tcurrent,4)[1][joint]) for joint in range(15)]
    # expected_positions.reverse()
    # print('position error (joint 1): ' + str(abs(q[0]-expected_positions[11])))
    torques = -Kp * (q-expected_positions)
    torques += -Kv * (vq)
    # torques *= [0,0,0,0,0,0,0,0,0,0,0,1,0,0,0]
    # torques += [0,0,0,0,0,0,0,0,1,0,0,0,0,0,1] * 0.1
    # torques.append(0.)
    sim.step(torques)

if __name__ == "__main__":
        
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT
    
    robot, sim, cube = setupwithpybullet()
    
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
    from inverse_geometry import computeqgrasppose
    from path import computepath
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    path = computepath(robot,cube,q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, None)
    # from test import G
    path = path[0]

    
    #setting initial configuration
    sim.setqsim(q0)
    
    
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
    
    def maketraj(q0, q1, t, T):
        # q_of_t = Bezier([q0,q0,q1,q1],t_max=T)
        # vq_of_t = q_of_t.derivative(1)
        # vvq_of_t = vq_of_t.derivative(1)
        q_of_t, vq_of_t, vvq_of_t = Traj_Interp1(path, t, T)
        
        return q_of_t, vq_of_t, vvq_of_t
    
    total_time=4.
    
    tcur = 0.
    trajs, vels, accs = maketraj(q0, qe, tcur, total_time)   
    # trajs = path
    pos1_actual = []
    joint = 11
    
    while tcur < total_time:
        rununtil(controllaw, DT, sim, robot, path, tcur, cube)
        pos1_actual.append(sim.getpybulletstate()[0])
        tcur += DT
    for joint in range(15):
        import numpy as np
        import matplotlib.pyplot as plot
        x = np.arange(0, total_time + 0.0001, 0.001);
        traj_sec_time = total_time/(len(path)-1)
        pos1_1=[]
        for i in x:
            n = int(np.floor(i/traj_sec_time))
            if n == len(path):
                n = len(path)-1
            pos1_1.append(path[n][joint] + Traj_Interp1(path,i,total_time)[0][joint])
        plot.plot(x, pos1_1)
        plot.plot(x, np.asarray(pos1_actual)[:,joint])
        plot.xlabel('Time')
        plot.ylabel('Position')
        plot.title('Position, Desired Position vs Time (Joint ' + str(joint) + ')')
    
        plot.show()