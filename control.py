#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np

from bezier import Bezier

# PART 2: From path create time-parameterised trajectory, and implement control

# GIVEN GAINS

# Kp =200.               # proportional gain (P of PD)
# Kv = 0.6 * np.sqrt(200)   # derivative gain (D of PD)
# Kp = -1 *[650,20,0.1,1500,400,300,1000,1000,50,1500,400,300,1000,1000,50]

# This function applies the control law (PD torque controller, given trajectory and current time)
def controllaw(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
    traj_sec_time = 5/(len(trajs)-1)
    n = int(np.floor(tcurrent/traj_sec_time))
    if n == len(path):
        n = len(path)-1
    expected_positions = [(Traj_Interp1(trajs,tcurrent,5)[0][joint]) for joint in range(15)] + trajs[n]
    expected_velocities = [(Traj_Interp1(trajs,tcurrent,5)[1][joint]) for joint in range(15)]
    # HARDCODED STUFF
    # First function adds extra rotation to the base, to keep the arms away from the obstacle
    # Second adds extra bending to the elbow joints, keeping the hands above the obstacle
    # Second doesnt work, so am leaving it commented for now 
    expected_positions[0] += -0.5*np.sin((2*np.pi/5)*tcurrent)
    expected_positions += [0,0,0,0,0,0.2,0,0,0,0,0,0.2,0,0,0] * np.sin(np.pi * tcurrent / 5)
    print('Time = ' + str(tcurrent))
    # Position error
    torques = (q-expected_positions)
    # JONAH's PD GAINS
    Kp = np.array([650,20,0.1,1500,400,300,1000,1000,50,1500,400,300,1000,1000,50])
    Kv = np.array([50,1,0.05,50,24,15,15,15,0.5,50,24,15,15,15,0.5])
    # Torque command = position error * -Gains
    torques *= -Kp
    torques += -Kv * (vq)
    # This attempts to apply constant hand torques to hold the cube (doesn't work)
    # torques += [0,0,0,0,0,0,0,0,-5,0,0,0,0,0,5]
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
    path, foundpath = computepath(robot,cube,q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, None)
    path = path[0]

    #setting initial configuration
    sim.setqsim(q0)
    
    # This function is called for each joint, for each timestep
    # Given an initial/end joint angle (a,b) and a 0-1 normalised time (x)
    # This function returns an intermediate position (pos1), and velocity/accelerations
    # Motion is assumed to be “bang-bang” (constant acceleration followed by constant deceleration)
    # Since acceleration is constant, velocity is linear, and displacement is quadratic
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
    
    # This function is called once for each timestep
    # G is the path (list of configs)
    # Given the current time and total trajectory time, it calls Pos_interp
    # to figure out the desired configuration at current time
    def Traj_Interp1(G, t, t_total):
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
    
    # This function simply calls Traj_interp - should be removed, not deemed worth the effort
    def maketraj(q0, q1, t, T):
        q_of_t, vq_of_t, vvq_of_t = Traj_Interp1(path, t, T)
        
        return q_of_t, vq_of_t, vvq_of_t
    
    # Set the total trajectory time and current time
    total_time=5.
    tcur = 0.
    trajs, vels, accs = maketraj(q0, qe, tcur, total_time)   
    pos1_actual = []
    joint = 11
    
    # If path is found, run the simulation
    if foundpath == True:
        while tcur < total_time:
            rununtil(controllaw, DT, sim, robot, path, tcur, cube)
            pos1_actual.append(sim.getpybulletstate()[0])
            tcur += DT
        # For each joint, plot desired and actual positions over the entire trajectory
        for joint in range(15):
            import numpy as np
            import matplotlib.pyplot as plot
            x = np.arange(0, total_time, 0.001);
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
    else:
        print("Path not found, exiting...")