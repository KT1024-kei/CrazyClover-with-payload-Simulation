#!/user/bin/env python
# coding: UTF-8

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
from collections import deque
from tools.Mathfunction import Mathfunction as MF

class pl():
    def __init__(self,ax = None):
        if ax is None:
            fig = plt.figure()
            ax = Axes3D.Axes3D(fig)
            ax.set_xlim((-2,2))
            ax.set_ylim((-2,2))
            ax.set_zlim((0,2))
            ax.set_xlabel(" x[m] ")
            ax.set_ylabel(" y[m] ")
            ax.set_zlabel(" z[m] ")
        ax.plot([], [], [], '-', c='red',zorder = 10)
        ax.plot([], [], [], '-', c='blue',zorder = 10)
        ax.plot([], [], [], '-', c='green', marker='o', markevery=2,zorder = 10)
        ax.plot([], [], [], '-', c='red',zorder = 10)
        ax.plot([], [], [], '-', c='green', marker='o', markevery=2,zorder = 10)
        ax.plot([], [], [], '.', c='green', markersize=2,zorder = 10)
        
        self.lines = ax.get_lines()[-6:]
        self.pos_history = deque(maxlen=100)

    def update_plot(self,frame, P, L, q):

        # * plot quadrotor bodies in frame
        lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]], np.array([[P[0], P[0]+q[0]],[P[1], P[1]+q[1]],[P[2], P[2]+q[2]]]), np.array([L[0], L[1], L[2]])]

        for line, line_data in zip(self.lines[:5], lines_data):
            x, y, z = line_data
            line.set_data(x, y)
            line.set_3d_properties(z)

        self.pos_history.append(np.array([L[0], L[1], L[2]]))
        history = np.array(self.pos_history)
        self.lines[-1].set_data(history[:,0], history[:,1])
        self.lines[-1].set_3d_properties(history[:,-1])


Arm_length = 0.15
Height = 0.05

drone_nom = pd.read_csv('/home/kato/lab_exp_desktop_crazyswarm/Simulation/CrazyClover-with-payload-Simulation/demo_Experiment/drone_nom.csv')
drone_state = pd.read_csv('/home/kato/lab_exp_desktop_crazyswarm/Simulation/CrazyClover-with-payload-Simulation/demo_Experiment/drone_state.csv')
load_nom = pd.read_csv('/home/kato/lab_exp_desktop_crazyswarm/Simulation/CrazyClover-with-payload-Simulation/demo_Experiment/load_nom.csv')
load_state = pd.read_csv('/home/kato/lab_exp_desktop_crazyswarm/Simulation/CrazyClover-with-payload-Simulation/demo_Experiment/load_state.csv')

T = drone_nom["T"]

Qp = np.vstack((np.array(drone_state["Px"]), np.vstack((np.array(drone_state["Py"]), np.array(drone_state["Pz"]))))).T
Qv = np.vstack((np.array(drone_state["Vx"]), np.vstack((np.array(drone_state["Vy"]), np.array(drone_state["Vz"]))))).T
Qe = np.vstack((np.array(drone_state["Roll"]), np.vstack((np.array(drone_state["Pitch"]), np.array(drone_state["Yaw"]))))).T

Lp = np.vstack((np.array(load_state["Lx"]), np.vstack((np.array(load_state["Ly"]), np.array(load_state["Lz"]))))).T
# LV = np.vstack((np.array(load_state["VLx"]), np.vstack((np.array(load_state["VLy"]), np.array(load_state["VLz"]))))).T

Lq = np.vstack((np.array(load_state["qx"]), np.vstack((np.array(load_state["qy"]), np.array(load_state["qz"]))))).T
Ldq = np.vstack((np.array(load_state["dqx"]), np.vstack((np.array(load_state["dqy"]), np.array(load_state["dqz"]))))).T

body_frame = np.array([(Arm_length, 0, 0, 1),
                       (0, Arm_length, 0, 1),
                       (-Arm_length, 0, 0, 1),
                       (0, -Arm_length, 0, 1),
                       (0, 0, 0, 1),
                       (0, 0, Height, 1),
                       (0, 0, 0.0, 1.0),
                       (0, 0, 0.0, 1)])

exp_plot = pl()

for i in range(1000,len(T)-1):

    P = Qp[i, :]
    q = Lq[i, :]
    L = Lp[i, :]
    Euler = Qe[i, :]
    R = MF().Euler2Rot(Euler)
    wHb = np.r_[np.c_[R,P], np.array([[0, 0, 0, 1]])]
    quadWorldFrame = wHb.dot(body_frame.T)
    world_frame = quadWorldFrame[0:3]

    exp_plot.update_plot(world_frame, P, L, 0.5*q)
    plt.pause(0.001)
