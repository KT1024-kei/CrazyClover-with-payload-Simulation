import sys
sys.path.append('../')

import numpy as np

from tools.Mathfunction import Mathfunction
from Drone.State import State
from Drone.Inner_controller import Controller_attituede_rate

class Drone_with_cable_suspended(Mathfunction):
  def __init__(self, dt):
    print("Initial DRONE model")

    self.set_parametor(dt)
    self.inner_controller = Controller_attituede_rate(dt, self.mQ+self.mL, self.I)
    
  def set_parametor(self, dt):

    # Physical Parametor
    self.g = 9.81
    self.e3 = np.array([0, 0, 1.0])
    self.dt = dt
    
    self.mQ = 0.558
    self.I = np.array([[10**(-1) , 0.0, 0.0],[0.0, 10**(-1), 0.0], [0.0, 0.0, 10**(-1)]])
    self.Arm_length = 0.15
    self.Hegiht = 0.05
    
    self.mL = 0.05
    self.l = 0.5

    self.CM_MP2FM  = np.array([[1.0, 1.0, 1.0, 1.0], [-1.0, -1.0, 1.0, 1.0], [-1.0, 1.0, 1.0, -1.0], [1.0, -1.0, 1.0, -1.0]])
    self.M = np.zeros(4)
    self.body_frame = np.array([(self.Arm_length, 0, 0, 1),
                       (0, self.Arm_length, 0, 1),
                       (-self.Arm_length, 0, 0, 1),
                       (0, -self.Arm_length, 0, 1),
                       (0, 0, 0, 1),
                       (0, 0, self.Hegiht, 1),
                       (0, 0, 0.0, 1.0),
                       (0, 0, 0.0, 1)])
 
  def set_initial_state(self, P, V, R, Euler, Wb, Euler_rate, L, dL, q, dq,  dt):
    # quadrotor state
    self.P = State(dt, P)
    self.V = State(dt, V)
    self.R = State(dt, R)
    self.Euler = State(dt, Euler)
    self.Wb = State(dt, Wb) # Omega
    self.Euler_rate = State(dt, Euler_rate)

    # Load state
    self.L = State(dt, L)
    self.dL = State(dt, dL)
    self.q = State(dt, q)
    self.dq = State(dt, dq)

  
  def update_state(self, acc, Omega_acc, Lacc, ddq):
    # quadrotor states
    self.V.integration(acc)
    self.Wb.integration(Omega_acc)
    self.Euler_rate.update((self.BAV2EAR(self.Euler.now, self.Wb.pre)))
    self.P.integration(self.V.pre)
    self.Euler_rate.pre[1] = -self.Euler_rate.pre[1]
    self.Euler.integration(self.Euler_rate.pre)
    self.R.integration(np.matmul(self.R.now, self.Vee(self.Wb.pre)))

    wHb = np.r_[np.c_[self.R.now,self.P.now], np.array([[0, 0, 0, 1]])]
    self.quadWorldFrame = wHb.dot(self.body_frame.T)
    self.world_frame = self.quadWorldFrame[0:3]

    # Load states
    self.dL.integration(Lacc)
    self.L.integration(self.dL.pre)
    self.dq.integration(ddq)
    self.q.integration(self.dq.pre)

    self.inner_controller.set_state(self.Wb.now, self.Euler_rate.now)
    
  def MP2FM(self, Moter_Power):

    return np.matmul(self.CM_MP2FM, Moter_Power)

  def get_input_acc_and_Wb(self, acc, Wb):

    self.inner_controller.inner_controller2(acc, Wb)
    return self.inner_controller.MP_pwm

  def Power_destribution_stock(self, T, Eulerrate):

    Moter_Power = np.zeros(4)
    r = Eulerrate[0]/2.0
    p = Eulerrate[1]/2.0
    y = Eulerrate[2]

    Moter_Power[0] = self.saturation(T - r + p + y, 35000.0, 0.0)
    Moter_Power[1] = self.saturation(T - r - p - y, 35000.0, 0.0)
    Moter_Power[2] = self.saturation(T + r - p + y, 35000.0, 0.0)
    Moter_Power[3] = self.saturation(T + r + p - y, 35000.0, 0.0)

    return Moter_Power

  def Power_destribution_stock2(self, IN_Moter_Power):

      Moter_Power = np.zeros(4)

      Moter_Power[0] = self.saturation(IN_Moter_Power[0], 40000.0, 0.0)
      Moter_Power[1] = self.saturation(IN_Moter_Power[1], 40000.0, 0.0)
      Moter_Power[2] = self.saturation(IN_Moter_Power[2], 40000.0, 0.0)
      Moter_Power[3] = self.saturation(IN_Moter_Power[3], 40000.0, 0.0)

      return Moter_Power

  def Drone_with_Load_Dynamics(self, F, M):

    acc = (np.dot(self.q.now, F*self.R.now@self.e3)*self.q.now*(1 - (self.mQ+self.mL)/self.mQ) +  \
          self.mL*self.l*np.dot(self.dq.now, self.dq.now)*self.q.now + ((self.mQ+self.mL)/self.mQ)*np.dot(self.q.now, self.q.now)*F*self.R.now@self.e3)/(self.mQ + self.mL) - self.g*self.e3
    Omega_acc = np.matmul(np.linalg.inv(self.I), (M - np.cross(self.Wb.now, np.matmul(self.I, self.Wb.now))))

    Lacc = (np.dot(self.q.now,F*self.R.now@self.e3) - self.mQ*self.l*(np.dot(self.dq.now,self.dq.now)))*self.q.now/(self.mQ + self.mL) - self.g*self.e3
    qdd = np.cross(self.q.now, np.cross(self.q.now, F*self.R.now@self.e3))/(self.mQ*self.l) - np.dot(self.dq.now, self.dq.now)*self.q.now

    return acc, Omega_acc, Lacc, qdd

  def MM_pwm2gf(self, moter):

    In_m1 = moter[0]
    In_m2 = moter[1]
    In_m3 = moter[2]
    In_m4 = moter[3]

    Out_m = np.zeros(4)

    m1_map = np.array([2.1866e-09, 2.5864e-05, -0.0699]) # np.array([2.077e-07, 0.0021, 0.0])
    m2_map = np.array([1.9461e-09, 2.5622e-05, -0.0648]) # np.array([2.1910e-07, 0.0022, 0.0])
    m3_map = np.array([2.0772e-09, 2.3301e-05, -0.0495]) # np.array([2.1161e-07, 0.0024, 0.0])
    m4_map = np.array([1.8948e-09, 3.1570e-05, -0.0759]) #  np.array([2.0210e-07, 0.0024, 0.0])

    m1_map_torque = np.array([-7.4901e-21, -9.0801e-17, 4.6733e-11, 1.0349e-07, -9.8509e-06])
    m2_map_torque = np.array([1.3154e-20, -1.5278e-15, 7.4698e-11, -1.4693e-07, 1.3121e-04])
    m3_map_torque = np.array([-1.2578e-20, 4.5518e-16, 2.5502e-11, 3.6675e-07, 2.8010e-04])
    m4_map_torque = np.array([-1.0431e-21, -6.6071e-16, 6.1008e-11, 1.7505e-08, 1.9959e-05])
    
    sign_m1 = np.sign(In_m1)
    Out_m[0] = sign_m1 * np.dot(m1_map, np.array([In_m1**2, sign_m1*np.abs(In_m1), 1.0])) + sign_m1 * np.dot(m1_map_torque, np.array([In_m1**4, sign_m1*np.abs(In_m1)**3, In_m1**2, sign_m1*np.abs(In_m1), 1.0]))

    sign_m2 = np.sign(In_m2)
    Out_m[1] = sign_m2 * np.dot(m2_map, np.array([In_m2**2, sign_m2*np.abs(In_m2), 1.0])) + sign_m2 * np.dot(m2_map_torque, np.array([In_m2**4, sign_m1*np.abs(In_m2)**3,In_m2**2, sign_m2*np.abs(In_m2), 1.0]))

    sign_m3 = np.sign(In_m3)
    Out_m[2] = sign_m3 * np.dot(m3_map, np.array([In_m3**2, sign_m3*np.abs(In_m3), 1.0])) + sign_m3 * np.dot(m3_map_torque, np.array([In_m3**4, sign_m1*np.abs(In_m3)**3,In_m3**2, sign_m3*np.abs(In_m3), 1.0]))

    sign_m4 = np.sign(In_m4)
    Out_m[3] = sign_m4 * np.dot(m4_map, np.array([In_m4**2, sign_m4*np.abs(In_m4), 1.0])) + sign_m4 * np.dot(m4_map_torque, np.array([In_m4**4, sign_m1*np.abs(In_m4)**3,In_m4**2, sign_m4*np.abs(In_m4), 1.0]))

    return Out_m

  def main(self, acc, Wb):

    self.M = self.get_input_acc_and_Wb(acc, Wb)

    M_pwm = self.Power_destribution_stock2(self.M)
    M_gf = self.MM_pwm2gf(M_pwm)
    self.F_and_M = self.MP2FM(M_gf)
    tmp = np.array([self.I[0, 0]*self.inner_controller.M_gf[0], self.I[1, 1]*self.inner_controller.M_gf[1], self.I[2, 2]*self.inner_controller.M_gf[2]])
    tmp = np.array([self.F_and_M[1], self.F_and_M[2],self.F_and_M[2]])

    acc, Wb_acc, Lacc, ddq  = self.Drone_with_Load_Dynamics(acc*(self.mQ+self.mL), tmp)
    self.update_state(acc,  Wb_acc, Lacc, ddq)

    return acc, Wb_acc