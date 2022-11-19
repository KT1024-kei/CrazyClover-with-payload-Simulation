import sys
import numpy as np
from numpy import linalg as LA
import pandas as pd
sys.path.append('../')
from tools.Mathfunction import Mathfunction as MF
from tools.Decorator import run_once

class Payload_Trajectory():

  def __init__(self):
    print("Init trajectory planning")
    self.traj_L = np.zeros(3)
    self.traj_dL = np.zeros(3)
    self.traj_ddL = np.zeros(3)
    self.traj_dddL = np.zeros(3)
    self.traj_ddddL = np.zeros(3)
    self.traj_dddddL = np.zeros(3)
    self.traj_Qyaw = 0.0
    self.traj_Qyaw_rate = 0.0

    self.e3 = np.array([0.0, 0.0, 1.0])
    self.g = 9.81

  def set_clock(self, t):
    self.t = t

  @run_once
  def poly_traj_init(self, trajectory_plan):
    # polynominal trajectory planning
    if trajectory_plan == "straight":
      self.traj = pd.read_csv('/home/kato/lab_exp_desktop_crazyswarm/Simulation/CrazyClover/Controller/Trajectory segment parametors/traj_straight_4s.csv')
    
    self.len_seg = self.traj["N_segment"][0]
    self.segs_T = self.traj["Tseg"][0:self.len_seg]
    self.Xcoeffs = self.traj["Xcoeff"]
    self.Ycoeffs = self.traj["Ycoeff"]
    self.Zcoeffs = self.traj["Zcoeff"]
    self.Order = self.traj["Order"][0]

    self.seg_now = 0
    self.T = 0
    self.Toffset = self.t
    # print(self.Order)

  def set_traj_plan(self, trajectory_plan):
    self.trajectory_plan = trajectory_plan
    print(trajectory_plan)
    
  def poly_traj(self):
    t = self.t
    # print(sum(self.segs_T) + self.Toffset, t)
    if sum(self.segs_T) + self.Toffset < t:
      return 0
    if sum(self.segs_T[:self.seg_now+1])+self.Toffset < t:
      self.T += self.segs_T[self.seg_now]
      self.seg_now += 1
    t -= (self.T + self.Toffset)
    # print(t)
    
    Xcoeff = self.Xcoeffs[self.seg_now*self.Order:(self.seg_now+1)*self.Order]
    Ycoeff = self.Ycoeffs[self.seg_now*self.Order:(self.seg_now+1)*self.Order]
    Zcoeff = self.Zcoeffs[self.seg_now*self.Order:(self.seg_now+1)*self.Order]
    
    poly_T0 = MF().time_polyder(t, 0, self.Order)
    poly_T1 = MF().time_polyder(t, 1, self.Order)
    poly_T2 = MF().time_polyder(t, 2, self.Order)
    poly_T3 = MF().time_polyder(t, 3, self.Order)
    poly_T4 = MF().time_polyder(t, 4, self.Order)
    poly_T5 = MF().time_polyder(t, 5, self.Order)

    self.traj_L = np.array([np.dot(Xcoeff, poly_T0), np.dot(Ycoeff, poly_T0), np.dot(Zcoeff, poly_T0)])
    self.traj_dL = np.array([np.dot(Xcoeff, poly_T1), np.dot(Ycoeff, poly_T1), np.dot(Zcoeff, poly_T1)])
    self.traj_ddL = np.array([np.dot(Xcoeff, poly_T2), np.dot(Ycoeff, poly_T2), np.dot(Zcoeff, poly_T2)])
    self.traj_dddL = np.array([np.dot(Xcoeff, poly_T3), np.dot(Ycoeff, poly_T3), np.dot(Zcoeff, poly_T3)])
    self.traj_ddddL = np.array([np.dot(Xcoeff, poly_T4), np.dot(Ycoeff, poly_T4), np.dot(Zcoeff, poly_T4)])
    self.traj_dddddL = np.array([np.dot(Xcoeff, poly_T5), np.dot(Ycoeff, poly_T5), np.dot(Zcoeff, poly_T5)])
    self.traj_ddddddL = np.array([np.dot(Xcoeff, poly_T3), np.dot(Ycoeff, poly_T3), np.dot(Zcoeff, poly_T3)])

    self.traj_Qyaw = 0.0
    self.traj_Qyaw_rate = 0.0

  def traj_circle(self):
    T = 12
    A = 1.0
    w = 2*np.pi/T
    self.traj_L[0] =  A*np.cos(w*self.t);      self.traj_L[1] =  A*np.sin(w*self.t);      self.traj_L[2] = 0.5
    self.traj_dL[0] = -A*w*np.sin(w*self.t);    self.traj_dL[1] =  A*w*np.cos(w*self.t);    self.traj_dL[2] = 0.0
    self.traj_ddL[0] = -A*w**2*np.cos(w*self.t); self.traj_ddL[1] = -A*w**2*np.sin(w*self.t); self.traj_ddL[2] = 0.0
    self.traj_dddL[0] =  A*w**3*np.sin(w*self.t); self.traj_dddL[1] = -A*w**3*np.cos(w*self.t); self.traj_dddL[2] = 0.0
    self.traj_ddddL[0] =  A*w**4*np.cos(w*self.t); self.traj_ddddL[1] = A*w**4*np.sin(w*self.t); self.traj_ddddL[2] = 0.0
    self.traj_dddddL[0] =  -A*w**5*np.sin(w*self.t); self.traj_dddddL[1] = A*w**5*np.cos(w*self.t); self.traj_dddddL[2] = 0.0
    # print(self.traj_acc)

    self.traj_Qyaw = 0.0
    self.traj_Qyaw_rate = 0.0

  def traj_hover(self):
    T = 10.0
    A = 1.0
    w = 2*np.pi/T
    self.traj_L[0:2] = 0.0; self.traj_L[2] =  0.5#1+A*np.sin(w*self.t)     
    self.traj_dL[0:2] = 0.0; self.traj_dL[2] =  0.0#A*w*np.cos(w*self.t)*0
    self.traj_ddL[0:2] = 0.0; self.traj_ddL[2] = -A*w**2*np.sin(w*self.t)*0
    self.traj_dddL[0:2] =  0.0; self.traj_dddL[2] = -A*w**3*np.cos(w*self.t) *0
    self.traj_ddddL[0:2] =  0.0; self.traj_ddddL[2] = A*w**4*np.sin(w*self.t)*0
    self.traj_dddddL[0:2] =  0.0; self.traj_dddddL[2] = A*w**5*np.cos(w*self.t)*0
    # print(self.traj_acc)

    self.traj_Qyaw = 0.0
    self.traj_Qyaw_rate = 0.0


  def Cable_vector_traj(self):
    
    ge3 = self.g*self.e3
    Lacc = self.traj_ddL  # 2nd der
    Ljer = self.traj_dddL  # 3d der
    Lsnap = self.traj_ddddL  # 4th der
    Lcrack = self.traj_dddddL  # 5th der
    
    P_c =-(Lacc + ge3)/LA.norm(Lacc + ge3)

    P_cd = - Ljer/LA.norm(Lacc + ge3) - P_c*np.dot(Ljer, P_c)/LA.norm(Lacc + ge3)

    P_cdd = (-Lsnap + -2*P_cd*np.dot(Ljer, P_c) + P_c*(np.dot(Lsnap, P_c) + np.dot(Ljer, P_cd)) )/LA.norm(Lacc + ge3)

    P_cddd = ( -Lcrack + 3*P_cdd *np.dot(Ljer, P_c) + 3*P_cd*(np.dot(Lsnap, P_c) + np.dot(Ljer, P_cd)) + P_c*(np.dot(Lcrack, P_c) + 2*np.dot(Lsnap, P_c) + 2*np.dot(Lsnap, P_cd) + np.dot(Ljer, P_cdd) + 2*np.dot(Ljer, P_cd) ))/LA.norm(Lacc + ge3)

    self.traj_q = P_c
    self.traj_dq = P_cd
    self.traj_ddq = P_cdd
    self.traj_dddq = P_cddd

  
  def stop_track(self):

    self.traj_L[0] = 0.0; self.traj_L[1] = 0.0;  self.traj_L[2] = 0.0
    self.traj_dL[0] = 0.0; self.traj_dL[1] = 0.0;  self.traj_dL[2] = 0.0
    self.traj_ddL[0] = 0.0; self.traj_ddL[1] = 0.0;  self.traj_ddL[2] = 0.0
    self.traj_dddL[0] = 0.0; self.traj_dddL[1] = 0.0;  self.traj_dddL[2] = 0.0
    self.traj_ddddL[0] = 0.0; self.traj_ddddL[1] = 0.0;  self.traj_ddddL[2] = 0.0
    # print(self.traj_acc)

    self.traj_yaw = 0.0
    self.traj_yaw_rate = 0.0
  
  def set_traj(self):
    
    if self.trajectory_plan == "circle":
      self.traj_circle()
      self.Cable_vector_traj()
    elif self.trajectory_plan == "hover":
      self.traj_hover()
      self.Cable_vector_traj()
    
    elif self.trajectory_plan == "stop":
      self.stop_track()

    elif self.trajectory_plan == "straight":
      
      self.poly_traj_init("straight")
      self.poly_traj()
      self.Cable_vector_traj()