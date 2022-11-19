import sys
sys.path.append('../')

import numpy as np

from tools.pid import PID
from tools.Mathfunction import Mathfunction

class Pid_Controller(Mathfunction):

  def __init__(self, dt, mode):
      self.mode = mode
      self.dt = dt
      self.input_thrust_gf = 0.0
      self.input_M_gf = np.array([0.0, 0.0, 0.0])
      
      self.input_thrust_pwm = 0.0
      self.input_M_pwm = np.array([0.0, 0.0, 0.0])

      self.gravity_calcel = 9.8
      self.rad2deg = 180/np.pi

      self.input_acc = 0.0
      self.input_Wb = np.zeros(3)
  
  def set_dt(self, dt):
    self.dt = dt

  def pid_init(self):
      self.R_pid = PID(10.0, 2.5, 0.0, self.dt)
      self.P_pid = PID(10.0, 2.5, 0.0, self.dt)
      self.Y_pid = PID(10.0, 1.5, 0.0, self.dt)

      self.Vx_pid = PID(1.0, 0.0, 0.0, self.dt)
      self.Vy_pid = PID(1.0, 0.0, 0.0, self.dt)
      self.Vz_pid = PID(4.0, 0.0, 0.0, self.dt)

      self.Px_pid = PID(1.0, 0.0, 0.0, self.dt)
      self.Py_pid = PID(1.0, 0.0, 0.0, self.dt)
      self.Pz_pid = PID(3.0, 1., 0.0, self.dt)

  def set_state(self, P, V, R, Euler):
      
    # print("set references")
    self.P = P
    self.V = V
    self.R = R
    self.Euler = Euler

    self.Px_pid.state = P[0]
    self.Py_pid.state = P[1]
    self.Pz_pid.state = P[2]


    self.Vx_pid.state = V[0]
    self.Vy_pid.state = V[1]
    self.Vz_pid.state = V[2]

    self.R_pid.state = Euler[0]
    self.P_pid.state = Euler[1]
    self.Y_pid.state = Euler[2]

    # if d gain == 0
    
    # self.R_pid.dt = self.dt 
    # self.P_pid.dt = self.dt 
    # self.Y_pid.dt = self.dt 
    # self.Vx_pid.dt = self.dt
    # self.Vy_pid.dt = self.dt
    # self.Vz_pid.dt = self.dt
    # self.Px_pid.dt = self.dt
    # self.Py_pid.dt = self.dt
    # self.Pz_pid.dt = self.dt


  def set_reference(self, P, V, R, Euler, Wb, Euler_rate, mode):
      
    # print("set references")
    self.Pref = P
    self.Vref = V
    self.Rref = R
    self.Eulerref = Euler
    self.Wbref = Wb
    self.Euler_rateref = Euler_rate

  def controller_attitude_pid(self):
    # print("PID Attitude Controller")

    if self.mode == "attitude":
      self.R_pid.desired = self.Eulerref[0]
      self.P_pid.desired = self.Eulerref[1]
      self.Y_pid.desired = self.Eulerref[2]


    self.R_pid.runpid()
    self.P_pid.runpid()

    # update PID for yaw axis
    Yaw_Error = self.Y_pid.desired - self.Euler[2]
    if Yaw_Error > np.pi:
      Yaw_Error -= 2*np.pi
    elif Yaw_Error < -np.pi:
      Yaw_Error += 2*np.pi
    self.Y_pid.Err = Yaw_Error
    self.Y_pid.runpid()

    input_Euler_rate = np.array([self.R_pid.output, self.P_pid.output, self.Y_pid.output])
    self.input_Wb = self.EAR2BAV(self.Euler, input_Euler_rate)
    self.input_acc = max(-9.8/2.0, self.input_thrust_gf) + self.gravity_calcel

  def controller_velocity_pid(self):
    # print("PID Velocity controller")
    if self.mode == "velocity":
      self.Vx_pid.desired = self.Vref[0]
      self.Vy_pid.desired = self.Vref[1]
      self.Vz_pid.desired = self.Vref[2]

    cosY = np.cos(self.Euler[2]); sinY = np.sin(self.Euler[2])

    self.Vx_pid.runpid()
    self.Vy_pid.runpid()
    self.Vz_pid.runpid()

    self.R_pid.desired = -(self.Vy_pid.output * cosY) + (self.Vx_pid.output  * sinY)
    self.P_pid.desired = -(-(self.Vx_pid.output * cosY) - (self.Vy_pid.output * sinY))

    self.input_thrust_gf = self.Vz_pid.output
    
    # print(self.Vz_pid.output)
    # print(self.P_pid.desired)
    self.controller_attitude_pid()
    

  def controller_position_pid(self, t):
    # print("PID Position Controller")
    if self.mode == "position":
      self.Px_pid.desired = self.Pref[0]
      self.Py_pid.desired = self.Pref[1]
      self.Pz_pid.desired = self.Pref[2]
    cosY = np.cos(self.Euler[2]); sinY = np.sin(self.Euler[2])

    # calcurate desired body velocity
    self.Px_pid.runpid()
    self.Py_pid.runpid()
    self.Pz_pid.runpid()

    self.Vx_pid.desired = self.Px_pid.output
    self.Vy_pid.desired = self.Py_pid.output
    self.Vz_pid.desired = self.Pz_pid.output

    self.controller_velocity_pid()

  def log_nom(self, log, t):
    log.write_nom(t=t, input_acc=self.input_acc, input_Wb=self.input_Wb, P=self.Pref, 
                        V=np.array([self.Vx_pid.desired, self.Vy_pid.desired, self.Vz_pid.desired]), 
                        Euler=np.array([self.R_pid.desired, self.P_pid.desired, self.Y_pid.desired]), 
                        Wb=self.Wbref, Euler_rate=self.Euler_rateref)