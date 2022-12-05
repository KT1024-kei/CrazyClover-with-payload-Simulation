#-*- coding: utf-8 -*-
"""
There are usefull tools to do Experiment

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

# * ========================= Initialize part ============================ * 
class Env_Experiment(Mathfunction):
    def __init__(self, Texp, Tsam, num):

        # * Experiment parametor
        self.Tend = Texp
        self.dt = Tsam
        self.num = num
        self.t = 0     
        
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

    def update_plot(self,frame):

        # * plot quadrotor bodies in frame
        lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]], np.array([[self.P[0], self.L[0]],[self.P[1], self.L[1]],[self.P[2], self.L[2]]]), np.array([self.L[0], self.L[1], self.L[2]])]

        for line, line_data in zip(self.lines[:5], lines_data):
            x, y, z = line_data
            line.set_data(x, y)
            line.set_3d_properties(z)

        self.pos_history.append(np.array([self.L[0], self.L[1], self.L[2]]))
        history = np.array(self.pos_history)
        self.lines[-1].set_data(history[:,0], history[:,1])
        self.lines[-1].set_3d_properties(history[:,-1])

# * ==================================================================== * 

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
                            init_controller=True,
                            tmp_P = np.zeros(3)):
        if init_controller:
            controller.select_controller()
        if controller_type == "pid":
            if command =="hovering":
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type)    
            elif command == "land":
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type) 
            else:
                controller.set_reference(P, V, R, Euler, Wb, Euler_rate, controller_type)
        elif controller_type == "mellinger":
            controller.set_reference(traj, self.t, tmp_P)

        elif controller_type == "QCSL":
            controller.set_reference(traj, self.t)

    def set_clock(self, t):
        self.t = t

    def set_dt(self, dt):
        self.dt = dt
        
    def update_state(self, drone):
        
        self.P          =     drone.P.now
        self.V          =     drone.V.now
        self.Euler      =     drone.Euler.now
        self.R          =     drone.R.now
        self.Wb         =     drone.Wb.now
        self.Euler_rate =     drone.Euler_rate.now
        drone.Euler_rate.now[1] =  -drone.Euler_rate.now[1] # * inverse the pitch rate same as crazyflie
        self.L = drone.L.now
        self.dL = drone.dL.now
        self.q = drone.q.now
        self.dq = drone.dq.now
        self.M = drone.M

    def take_log(self, ctrl):
        self.log.write_state(self.t, self.P, self.V, self.R, self.Euler, np.zeros(3), np.zeros(3), self.M, self.L, self.q, self.dq)
        ctrl.log(self.log, self.t)
        
    def save_log(self):
      self.log.close_file()

    def time_check(self,  Tint, Tend):
        if self.t > Tend:
            return True
        return False

# * ======================== controll commands ================================ *  

    @run_once
    def land(self, controller, controller_type="pid"):
        controller.switch_controller("pid")
        self.set_reference(controller=controller, command="land", init_controller=True, P=self.land_P, controller_type="pid")
        
    @run_once
    def hovering(self, controller, P=np.array([0.0, 0.0, 1.0]), controller_type="payload"):
        self.set_reference(controller=controller, command="hovering", P=P, controller_type=controller_type)
        self.land_P = np.array([0.0, 0.0, 0.1])

    def quad_takeoff(self, controller, controller_type="mellinger", Pinit=np.array([0.0, 0.0, 0.0])):
        controller.switch_controller(controller_type)
        self.set_reference(controller=controller, traj="takeoff", controller_type=controller_type, tmp_P=Pinit)
        self.land_P = np.array([0.0, 0.0, 0.1])
    
    def quad_land(self, controller, controller_type="mellinger"):
        controller.switch_controller(controller_type)
        self.set_reference(controller=controller, traj="land", controller_type=controller_type, init_controller=False, tmp_P=np.array([self.P[0], self.P[1], 0.0]))

    def quad_tack_circle(self, controller, controller_type="mellinger", flag=False):
        controller.switch_controller(controller_type)
        self.set_reference(controller=controller, traj="circle", controller_type=controller_type, init_controller=flag)

    def quad_tack_straight(self, controller, flag=False):
        self.set_reference(controller=controller, traj="straight", controller_type="mellinger", init_controller=flag)
    
    def quad_stop_track(self, controller, controller_type="mellinger"):
        controller.switch_controller(controller_type)
        self.set_reference(controller=controller, traj="stop", controller_type=controller_type, init_controller=False, tmp_P=np.array([self.P[0], self.P[1], 1.0]))
        self.land_P[0:2] = self.P[0:2]
        
    def payload_track_circle(self, controller, controller_type="QCSL", flag=False):
        controller.switch_controller(controller_type)
        self.set_reference(controller=controller, traj="circle", controller_type=controller_type, init_controller=flag)
    
    def payload_track_straight(self, controller, controller_type="QCSL", flag=False):
        controller.switch_controller(controller_type)
        self.set_reference(controller=controller, traj="straight", controller_type=controller_type, init_controller=flag)

    def payload_stop_track(self, controller, controller_type="QCSL"):
        controller.switch_controller(controller_type)
        self.set_reference(controller=controller, traj="stop", controller_type=controller_type, init_controller=False)
        self.land_P[0:2] = self.P[0:2]
    
    def paylaod_track_hover_payload(self, controller, controller_type="QCSL", flag=False):
        controller.switch_controller(controller_type)
        self.set_reference(controller=controller, traj="hover", controller_type=controller_type, init_controller=flag)