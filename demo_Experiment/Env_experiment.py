#-*- coding: utf-8 -*-
"""
tips 

自作モジュールを読み込みたい時はsettingsのところでpython.analysis.extraPathsにパスを登録する

"""
import sys
sys.path.append('../')
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
from collections import deque

from tools.Decorator import run_once
from tools.Mathfunction import Mathfunction, Integration
from tools.Log import Log_data
from Drone.Drone_with_Load_model import Drone_with_cable_suspended as QCS
from Controller.Controllers import Controllers

class Env_Experiment(Mathfunction):
    def __init__(self, Texp, Tsam, num):

        # Experiment Parametor
        self.Tend = Texp
        self.dt = Tsam
        self.num = num        
        
        self.log = Log_data(num)
        self.model = QCS(self.dt)
        self.l = self.model.l

    def init_state(self, drone = 0, 
                                P=np.array([0.0, 0.0, 1.0]),   
                                V=np.array([0.0, 0.0, 0.0]), 
                                R=np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), 
                                Euler=np.array([0.0, 0.0, 0.0]), 
                                Wb=np.array([0.0, 0.0, 0.0]), 
                                Euler_rate=np.array([0.0, 0.0, 0.0]),
                                q = np.array([0.0, 0.0, -1.0]),
                                dq = np.array([0.0, 0.0, 0.0])):

        R = self.Euler2Rot(Euler)
        print("set Initial state")
        self.P          = P
        self.V          = V
        self.Euler      = Euler
        self.R          = R
        self.Wb         = Wb
        self.Euler_rate = Euler_rate
        self.L          = P + self.l*q
        self.dL         = V + self.l*dq
        self.q          = q
        self.dq         = dq
        self.M = drone.M

        drone.set_initial_state(P, V, R, Euler, Wb, Euler_rate, self.L, self.dL, self.q, self.dq, self.dt)

    def init_plot(self,ax = None):
        if ax is None:
            fig = plt.figure()
            ax = Axes3D.Axes3D(fig)
            ax.set_xlim((-2,2))
            ax.set_ylim((-2,2))
            ax.set_zlim((0,2))
        ax.plot([], [], [], '-', c='red',zorder = 10)
        ax.plot([], [], [], '-', c='blue',zorder = 10)
        ax.plot([], [], [], '-', c='green', marker='o', markevery=2,zorder = 10)
        ax.plot([], [], [], '-', c='red',zorder = 10)
        ax.plot([], [], [], '-', c='green', marker='o', markevery=2,zorder = 10)
        ax.plot([], [], [], '.', c='green', markersize=2,zorder = 10)
        
        self.lines = ax.get_lines()[-6:]
        self.pos_history = deque(maxlen=100)

    def update_plot(self,frame):
        
        lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]], np.array([[self.P[0], self.L[0]],[self.P[1], self.L[1]],[self.P[2], self.L[2]]]), np.array([self.L[0], self.L[1], self.L[2]])]

        for line, line_data in zip(self.lines[:5], lines_data):
            x, y, z = line_data
            line.set_data(x, y)
            line.set_3d_properties(z)

        self.pos_history.append(np.array([self.L[0], self.L[1], self.L[2]]))
        history = np.array(self.pos_history)
        self.lines[-1].set_data(history[:,0], history[:,1])
        self.lines[-1].set_3d_properties(history[:,-1])

# ------------------------------- ここまで　初期化関数 ---------------------
    def set_reference(self, controller,  
                            P=np.array([-1.0, 0.0, 0.0]),   
                            V=np.array([0.0, 0.0, 0.0]), 
                            R=np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), 
                            Euler=np.array([0.0, 0.0, 0.0]), 
                            Wb=np.array([0.0, 0.0, 0.0]), 
                            Euler_rate=np.array([0.0, 0.0, 0.0]),
                            traj="circle",
                            controller_type="payload",
                            command = "hovering",
                            init_controller=True):
        if init_controller:
            controller.select_controller()
        if controller_type == "pid":
            if command =="hovering":
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type)    
            elif command == "land":
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type) 
            else:
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type)
        elif controller_type == "payload":
            controller.set_reference(traj)

        
    def set_dt(self, dt):
        self.dt = dt
        
    def update_state(self, drone):
        
        self.P          =     drone.P.now
        self.V          =     drone.V.now
        self.Euler      =     drone.Euler.now
        self.R          =     drone.R.now
        self.Wb         =     drone.Wb.now
        self.Euler_rate =     drone.Euler_rate.now
        drone.Euler_rate.now[1] =  -drone.Euler_rate.now[1]
        self.L = drone.L.now
        self.dL = drone.dL.now
        self.q = drone.q.now
        self.dq = drone.dq.now
        self.M = drone.M

    def take_log(self, t, ctrl):
        self.log.write_state(t, self.P, self.V, self.R, self.Euler, np.zeros(3), np.zeros(3), self.M, self.L, self.q, self.dq)
        ctrl.log(self.log, t)
        
    def save_log(self):
      self.log.close_file()

    def time_check(self, t, Tint, Tend):
        if t > Tend:
            return True
        return False

    @run_once
    def land(self, controller, controller_type="pid"):
        controller.switch_controller("pid")
        self.set_reference(controller=controller, command="land", init_controller=True, P=self.land_P, controller_type="pid")
        
    @run_once
    def hovering(self, controller, P=np.array([0.0, 0.0, 1.0]), controller_type="payload"):
        self.set_reference(controller=controller, command="hovering", P=P, controller_type=controller_type)
        self.land_P = np.array([0.0, 0.0, 0.1])

    def track_circle(self, controller, flag=False):
        self.set_reference(controller=controller, traj="circle", controller_type="payload", init_controller=flag)
    
    def track_straight(self, controller, flag):
        self.set_reference(controller=controller, traj="straight", controller_type="payload", init_controller=flag)

    def stop_track(self, controller):
        self.set_reference(controller=controller, traj="stop", controller_type="payload", init_controller=False)
        self.land_P[0:2] = self.P[0:2]
    
    def track_hover_payload(self, controller, flag=False):
        self.set_reference(controller=controller, traj="hover", controller_type="payload", init_controller=flag)