import sys
import numpy as np
import pandas as pd
sys.path.append('../')
from tools.Mathfunction import Mathfunction as MF
from tools.Decorator import run_once

class Trajectory():

  def __init__(self):
    print("Init trajectory planning")

    # * initialize trajectory states
    self.traj_pos = np.zeros(3)
    self.traj_vel = np.zeros(3)
    self.traj_acc = np.zeros(3)
    self.traj_jer = np.zeros(3)
    self.traj_yaw = 0.0
    self.traj_yaw_rate = 0.0

  # ! set trajectory plan 
  def set_traj_plan(self, trajectory_plan):
    self.trajectory_plan = trajectory_plan
    self.set_poly_traj(trajectory_plan)
    print(trajectory_plan)
    
  
  def set_clock(self, t):
    self.t = t

  # ! polynominal trajectory planning
  def poly_traj_init(self, trajectory_plan):

    if trajectory_plan == "straight":
      self.traj = pd.read_csv('/home/kato/lab_exp_desktop_crazyswarm/Simulation/CrazyClover/Controller/Trajectory segment parametors/traj_straight_4s.csv')
    
    elif trajectory_plan == "takeoff":
      self.traj = pd.read_csv('/home/kato/lab_exp_desktop_crazyswarm/Simulation/CrazyClover/Controller/Trajectory segment parametors/traj_takeoff.csv')
    
    elif trajectory_plan == "land":
      self.traj = pd.read_csv('/home/kato/lab_exp_desktop_crazyswarm/Simulation/CrazyClover/Controller/Trajectory segment parametors/traj_land.csv')
    
    else:
      return 0

    # * set trajectory palametors 
    self.len_seg = self.traj["N_segment"][0]
    self.segs_T = self.traj["Tseg"][0:self.len_seg]
    self.Xcoeffs = self.traj["Xcoeff"]
    self.Ycoeffs = self.traj["Ycoeff"]
    self.Zcoeffs = self.traj["Zcoeff"]
    self.Order = self.traj["Order"][0]

    self.seg_now = 0
    self.T = 0
    self.Toffset = self.t

  # ! periodic trajectory tracking 
  def poly_traj_periodic(self):

    # * manage time to calculate polynominal trajectory
    t = self.t
    if sum(self.segs_T) + self.Toffset < t:
      self.Toffset += sum(self.segs_T)
      self.seg_now = 0
      self.T = 0
    if sum(self.segs_T[:self.seg_now+1])+self.Toffset < t:
      self.T += self.segs_T[self.seg_now]
      self.seg_now += 1
    t -= (self.T + self.Toffset)

    # * set polynominal coeffiients
    Xcoeff = self.Xcoeffs[self.seg_now*self.Order:(self.seg_now+1)*self.Order]
    Ycoeff = self.Ycoeffs[self.seg_now*self.Order:(self.seg_now+1)*self.Order]
    Zcoeff = self.Zcoeffs[self.seg_now*self.Order:(self.seg_now+1)*self.Order]

    # * calculate states of trajectory (position, velocity, ...)
    poly_T0 = MF().time_polyder(t, 0, self.Order)
    poly_T1 = MF().time_polyder(t, 1, self.Order)
    poly_T2 = MF().time_polyder(t, 2, self.Order)
    poly_T3 = MF().time_polyder(t, 3, self.Order)

    self.traj_pos = np.array([np.dot(Xcoeff, poly_T0), np.dot(Ycoeff, poly_T0), np.dot(Zcoeff, poly_T0)])
    self.traj_vel = np.array([np.dot(Xcoeff, poly_T1), np.dot(Ycoeff, poly_T1), np.dot(Zcoeff, poly_T1)])
    self.traj_acc = np.array([np.dot(Xcoeff, poly_T2), np.dot(Ycoeff, poly_T2), np.dot(Zcoeff, poly_T2)]) + np.array([0.0 ,0.0, 9.8])
    self.traj_jer = np.array([np.dot(Xcoeff, poly_T3), np.dot(Ycoeff, poly_T3), np.dot(Zcoeff, poly_T3)])

    self.traj_yaw = 0.0
    self.traj_yaw_rate = 0.0

  # ! non periodic trajectory tracking
  def poly_traj_non_periodic(self):

    # * manage time to calculate polynominal trajectory
    t = self.t
    if sum(self.segs_T) + self.Toffset < t:
      return 0
    if sum(self.segs_T[:self.seg_now+1])+self.Toffset < t:
      self.T += self.segs_T[self.seg_now]
      self.seg_now += 1
    t -= (self.T + self.Toffset)

    # * set polynominal coeffiients
    Xcoeff = self.Xcoeffs[self.seg_now*self.Order:(self.seg_now+1)*self.Order]
    Ycoeff = self.Ycoeffs[self.seg_now*self.Order:(self.seg_now+1)*self.Order]
    Zcoeff = self.Zcoeffs[self.seg_now*self.Order:(self.seg_now+1)*self.Order]

    # * calculate states of trajectory (position, velocity, ...)
    poly_T0 = MF().time_polyder(t, 0, self.Order)
    poly_T1 = MF().time_polyder(t, 1, self.Order)
    poly_T2 = MF().time_polyder(t, 2, self.Order)
    poly_T3 = MF().time_polyder(t, 3, self.Order)

    self.traj_pos = np.array([np.dot(Xcoeff, poly_T0), np.dot(Ycoeff, poly_T0), np.dot(Zcoeff, poly_T0)])
    self.traj_vel = np.array([np.dot(Xcoeff, poly_T1), np.dot(Ycoeff, poly_T1), np.dot(Zcoeff, poly_T1)])
    self.traj_acc = np.array([np.dot(Xcoeff, poly_T2), np.dot(Ycoeff, poly_T2), np.dot(Zcoeff, poly_T2)]) + np.array([0.0 ,0.0, 9.8])
    self.traj_jer = np.array([np.dot(Xcoeff, poly_T3), np.dot(Ycoeff, poly_T3), np.dot(Zcoeff, poly_T3)])

    self.traj_yaw = 0.0
    self.traj_yaw_rate = 0.0

  def traj_circle(self):
    T = 7.0
    A = 1.0
    w = 2*np.pi/T
    self.traj_pos[0] =  A*np.cos(w*self.t);      self.traj_pos[1] =  A*np.sin(w*self.t);      self.traj_pos[2] = 1.0
    self.traj_vel[0] = -A*w*np.sin(w*self.t);    self.traj_vel[1] =  A*w*np.cos(w*self.t);    self.traj_vel[2] = 0.0
    self.traj_acc[0] = -A*w**2*np.cos(w*self.t); self.traj_acc[1] = -A*w**2*np.sin(w*self.t); self.traj_acc[2] = 9.8
    self.traj_jer[0] =  A*w**3*np.sin(w*self.t); self.traj_jer[1] = -A*w**3*np.cos(w*self.t); self.traj_jer[2] = 0.0

    self.traj_yaw = 0.0
    self.traj_yaw_rate = 0.0

  # ! stop trajectory update (only set gravity cancel term)  
  def stop_track(self):

    self.traj_pos[0] = 0.0; self.traj_pos[1] = 0.0;  self.traj_pos[2] = 0.0
    self.traj_vel[0] = 0.0; self.traj_vel[1] = 0.0;  self.traj_vel[2] = 0.0
    self.traj_acc[0] = 0.0; self.traj_acc[1] = 0.0;  self.traj_acc[2] = 9.8
    self.traj_jer[0] = 0.0; self.traj_jer[1] = 0.0;  self.traj_jer[2] = 0.0

    self.traj_yaw = 0.0
    self.traj_yaw_rate = 0.0

  # ! select trajectory
  def set_traj(self):
    
    if self.trajectory_plan == "circle":
      self.traj_circle()
    
    elif self.trajectory_plan == "stop":
      self.stop_track()

    elif self.trajectory_plan == "straight":
      self.poly_traj_non_periodic()
    
    elif self.trajectory_plan == "takeoff":
      self.poly_traj_non_periodic()
    
    elif self.trajectory_plan == "land":
      self.poly_traj_non_periodic()

  # ! initialize polynominal trajectory 
  def set_poly_traj(self, poly_traj):
    self.poly_traj_init(poly_traj)
    



    

