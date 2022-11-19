import sys


import numpy as np
from numpy import linalg as LA
from Controller.Payload_Trajectory import Payload_Trajectory
from tools.Mathfunction import Mathfunction
from Drone.Drone_with_Load_model import Drone_with_cable_suspended as DCS
from tools.pid import PIDVEC

class Quad_with_Cable_Suspended(Mathfunction):
  def __init__(self, dt):
    self.dt = dt

  def set_dt(self, dt):
    self.dt = dt
    
  def qcsl_init(self):
    print("Init QCSL Controller")

    # set physical parametor
    self.e3 = np.array([0.0, 0.0, 1.0])
    self.g = 9.81
    self.ge3 = self.g * self.e3
    model = DCS(self.dt)
    self.mQ = model.mQ
    self.mL = model.mL
    self.I = model.I
    self.l = model.l

    # init trajectory
    self.kp = np.array([0.7, 0.7, 4.0])*np.array([1, 1, 1])
    self.kd = np.array([6, 6, 3])*0.6
    self.ki = np.array([0.0, 0.0, 1.0])
    self.Lpid = PIDVEC(self.kp, self.ki, self.kd, self.dt)
    self.kpL = np.array([3, 3, 1])*1.3
    self.kpdL = np.array([6, 6, 5])*0.8
    self.kR = np.array([20, 20,0.5])

    self.Euler_nom = np.array([0.0, 0.0, 0.0])
    self.Euler_rate_nom = np.array([0.0, 0.0, 0.0])
    self.traj_W = np.zeros(3)
    self.traj_L = np.zeros(3);self.traj_q = np.zeros(3);self.traj_dq = np.zeros(3)

    self.input_acc = 0.0
    self.input_Wb = np.zeros(3)

    self.trajectory = Payload_Trajectory()

  def set_reference(self, traj_plan):
    self.trajectory.set_traj_plan(traj_plan)
    self.rad2deg = 180.0/np.pi

  def set_state(self, P, V, R, Euler, L, dL, q, dq):

    self.P = P
    self.V = V
    self.R = R
    self.Euler = Euler

    self.L = L
    self.dL = dL
    self.q = q
    self.dq = dq

  def Payload_Position_controller(self):

    self.traj_L = self.trajectory.traj_L
    self.traj_dL = self.trajectory.traj_dL
    self.traj_ddL = self.trajectory.traj_ddL
    self.traj_dddL = self.trajectory.traj_dddL
    
    self.Lpid.Err = self.L - self.traj_L
    self.Lpid.Err_div = self.dL - self.traj_dL
    self.Lpid.runpid3()

    self.A = -self.Lpid.output + (self.mQ + self.mL) * (self.traj_ddL + self.ge3) + self.mQ*self.l*np.dot(self.dq, self.dq)*self.q
    self.traj_q = -self.A/LA.norm(self.A)
    
  def Payload_Attitude_controller(self):
    # self.traj_q = self.trajectory.traj_q
    self.traj_dq = self.trajectory.traj_dq
    self.traj_ddq = self.trajectory.traj_ddq
    self.traj_dddq = self.trajectory.traj_dddq

    eq = np.matmul(self.Vee(self.q), self.Vee(self.q))@self.traj_q
    edq = self.dq - np.cross(np.cross(self.traj_q, self.traj_dq), self.q)

    self.F_pd = -self.kpL*eq - self.kpdL*edq
    self.F_ff = self.mQ*self.l*(np.dot(self.q, np.cross(self.traj_q, self.traj_dq))*np.cross(self.q, self.dq) + np.cross(np.cross(self.traj_q, self.traj_ddq), self.q))
    self.F_n = np.dot(self.A, self.q)*self.q

    self.F = self.F_n-self.F_ff-self.F_pd
    # print(abs(self.F_n/self.F))
    self.input_acc = max(9.8/3, np.dot(self.F, self.R@self.e3))/(self.mQ + self.mL)

  def Quadrotor_Attitude_controller(self):

    # set trajectory of each state
    traj_acc = self.mQ*(self.traj_ddL - self.l*self.traj_ddq + self.ge3) + self.mL*(self.traj_ddL+ self.ge3)
    traj_jer = self.mQ*(self.traj_dddL - self.l*self.traj_dddq) + self.mL*self.traj_dddL
    traj_yaw = self.trajectory.traj_Qyaw
    traj_yaw_rate = self.trajectory.traj_Qyaw_rate

    # calculate nominal Rotation matrics
    traj_R = np.zeros((3, 3))
    traj_Rxc = np.array([np.cos(traj_yaw), np.sin(traj_yaw), 0.0])
    traj_Ryc = np.array([-np.sin(traj_yaw), np.cos(traj_yaw), 0.0])
    traj_Rz = self.F/np.linalg.norm(self.F)

    traj_Rx = np.cross(traj_Ryc, traj_Rz)/np.linalg.norm(np.cross(traj_Ryc, traj_Rz))
    traj_Ry = np.cross(traj_Rz, traj_Rx)
    # traj_Ry = np.cross(traj_Rz, traj_Rxc)
    # traj_Rx = np.cross(traj_Ry, traj_Rz)

    traj_R[:, 0] = traj_Rx
    traj_R[:, 1] = traj_Ry
    traj_R[:, 2] = traj_Rz

    # calculate nominal Angular velocity
    traj_wy =  np.dot(traj_Rx, traj_jer) / np.dot(traj_Rz, self.F)
    traj_wx = -np.dot(traj_Ry, traj_jer) / np.dot(traj_Rz, self.F)
    traj_wz = (traj_yaw_rate * np.dot(traj_Rxc, traj_Rx) + traj_wy * np.dot(traj_Ryc, traj_Rz))/np.linalg.norm(np.cross(traj_Ryc, traj_Rz))
    self.traj_W[0] = traj_wx
    self.traj_W[1] = traj_wy
    self.traj_W[2] = traj_wz
    
    
    # calculate input Body angular velocity
    eR = self.Wedge((np.matmul(traj_R.T, self.R) - np.matmul(self.R.T, traj_R))/2.0)
    self.input_Wb = self.R.T@traj_R@(self.traj_W - self.kR*eR)
    
    # calculate nominal Euler angle and Euler angle rate
    self.Euler_nom[1] =  np.arctan( ( traj_acc[0]*np.cos(traj_yaw) + traj_acc[1]*np.sin(traj_yaw) ) / (traj_acc[2]))                                                        
    self.Euler_nom[0] = np.arctan( ( traj_acc[0]*np.sin(traj_yaw) - traj_acc[1]*np.cos(traj_yaw) ) / np.sqrt( (traj_acc[2])**2 + ( traj_acc[0]*np.cos(traj_yaw) + traj_acc[2]*np.sin(traj_yaw) )**2));  
    self.Euler_nom[2] = traj_yaw

    self.input_Euler_rate = self.BAV2EAR(self.Euler_nom, self.input_Wb)
    self.Euler_rate_nom = self.BAV2EAR(self.Euler_nom, self.traj_W)

  def qcsl_ctrl(self, t):
    self.trajectory.set_clock(t)
    self.trajectory.set_traj()
    self.Payload_Position_controller()
    self.Payload_Attitude_controller()
    self.Quadrotor_Attitude_controller()

  def stop_tracking(self):
    self.set_reference("stop")

  def log_nom(self, log, t):

    log.write_nom(t=t, input_acc=self.input_acc, input_Wb=self.input_Wb, P=self.trajectory.traj_L, V=self.trajectory.traj_dL, Euler=self.Euler_nom, Wb=self.traj_W, Euler_rate=self.Euler_rate_nom, L=self.traj_L, q=self.traj_q, dq=self.traj_dq)

    











    




    