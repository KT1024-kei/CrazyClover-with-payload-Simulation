import sys
sys.path.append('../')

import numpy as np
from tools.pid import PID
# from Drone.Drone_model import Drone
from tools.Mathfunction import Mathfunction

class Controller_attituede_rate(Mathfunction):

  def __init__(self, dt, mass, I):
    print("initialize Attitude rate controller")

    self.mQ =mass
    self.I = I
    self.g = 9.8
    # Euler angle rate PID
    self.R_rate_pid = PID(5.0, 2, 0.0, dt, 1.5)
    self.P_rate_pid = PID(5.0, 2, 0.0, dt, 1.5)
    self.Y_rate_pid = PID(15.0, 5, 0.0, dt, 1.5)
    
    # Body angle velocity PID
    self.Wx_pid = PID(30., 0.0, 0.0, dt, 1.5)
    self.Wy_pid = PID(30., 0.0, 0.0, dt, 1.5)
    self.Wz_pid = PID(1, 0, 0.0, dt)
    
    self.M_gf = np.array([0.0, 0.0, 0.0])
    self.FM_pwm = np.array([0.0, 0.0, 0.0, 0.0])

    self.MP_gf = np.array([0.0, 0.0, 0.0, 0.0])
    self.MP_pwm = np.array([0.0, 0.0, 0.0, 0.0])

    self.rad2deg = 180.0/np.pi
    self.deg2rad = np.pi/180.0

    # Controll Matrix
    KT = self.mQ/4
    KR = np.sqrt(2)*I[1, 1]/0.17/4.0
    KR = I[0, 0]/4.0
    KP = np.sqrt(2)*I[0, 0]/0.17/4.0
    KP = I[1, 1]/4.0
    KY = I[2, 2]/4
    # KY = 1/4.0
    self.FM2MP_map  = np.array([[KT, KT, KT, KT], [-KR, -KR, KR, KR], [-KP, KP, KP, -KP], [-KY, KY, -KY, KY]]).T
    self.MP2FM_map = np.array([[1.0, 1.0, 1.0, 1.0], [-1.0, -1.0, 1.0, 1.0], [-1.0, 1.0, 1.0, -1.0], [-1.0, 1.0, -1.0, 1.0]])

  def set_state(self, Wb, Euler_rate):
      
    # print("set references")
    self.Wb = Wb
    self.Euler_rate = Euler_rate

    self.Wx_pid.state = Wb[0]
    self.Wy_pid.state = Wb[1]
    self.Wz_pid.state = Wb[2]

    self.R_rate_pid.state = Euler_rate[0] * self.rad2deg
    self.P_rate_pid.state = -Euler_rate[1] * self.rad2deg
    self.Y_rate_pid.state = Euler_rate[2] * self.rad2deg
  
  def inner_controller(self, F, Euler_rate_input):
    
    self.R_rate_pid.desired = Euler_rate_input[0]
    self.P_rate_pid.desired = Euler_rate_input[1]
    self.Y_rate_pid.desired = Euler_rate_input[2]

    self.controll_attitude_rate()
    self.FM2MP(F)
    self.MM_gf2pwm()
    self.MM_gf2pwm2(F)
    # print("C",self.MP_gf)
    # print(self.M_gf)

  def inner_controller2(self, acc, Wb):
    
    # set desired angular velocity
    self.Wb = Wb
    self.Wx_pid.desired = Wb[0]
    self.Wy_pid.desired = Wb[1]
    self.Wz_pid.desired = Wb[2]

    self.controll_attitude_rate2()
    self.FM2MP(acc)
    self.MM_gf2pwm()
    # self.MM_gf2pwm2(F)
    # print("C",self.MP_gf)
    # print(self.M_gf)

  def controll_attitude_rate(self):

    self.R_rate_pid.runpid()
    self.P_rate_pid.runpid()
    self.Y_rate_pid.runpid()
    
    self.M_gf[0] = self.R_rate_pid.output*self.deg2rad
    self.M_gf[1] = self.P_rate_pid.output*self.deg2rad
    self.M_gf[2] = self.Y_rate_pid.output*self.deg2rad

  def controll_attitude_rate2(self):

    # Wb = self.EAR2BAV(Euler, Euler_rate)

    self.Wx_pid.runpid()
    self.Wy_pid.runpid()
    self.Wz_pid.runpid()
    
    self.M_gf[0] = self.Wx_pid.output
    self.M_gf[1] = self.Wy_pid.output
    self.M_gf[2] = self.Wz_pid.output

    self.M_gf += np.cross(self.Wb, self.I@self.Wb)

  def FM2MP(self, acc):

    self.MP_gf_TRP = np.matmul(self.FM2MP_map, np.array([acc, self.M_gf[0], self.M_gf[1], 0.0])) # Thrust, Roll, Pitch
    self.MP_gf_Y = np.matmul(self.FM2MP_map, np.array([0.0, 0.0, 0.0, self.M_gf[2]])) # Yaw

  def MM_gf2pwm(self):
    # print("Moter map: gF -> PWM ")

    In_m1 = self.MP_gf_TRP[0]
    In_m2 = self.MP_gf_TRP[1]
    In_m3 = self.MP_gf_TRP[2]
    In_m4 = self.MP_gf_TRP[3]
    
    sign_m1 = np.sign(In_m1)
    self.MP_pwm[0] = sign_m1 * (np.sqrt(sign_m1*In_m1*8.746454211812262e-9+1.280608521387491e-9)*2.28664090792239e+8-5.914182129481692e+3)
    # self.MP_pwm[0] = sign_m1 * (-1.0785e-5 * In_m1**4 + sign_m1* 0.0080 * In_m1**3 - 2.0815 * In_m1**2 +  306.7172*In_m1 + 1.7019e+03)

    sign_m2 = np.sign(In_m2)
    self.MP_pwm[1] = sign_m2 * (np.sqrt(sign_m2*In_m2*7.784387419895479e-9+1.160848849564822e-9)*2.56924519826488e+8-6.582814614736632e+3)
    # self.MP_pwm[1] = sign_m2 * (-8.3705e-06 * In_m2**4 + sign_m2* 0.0066 * In_m2**3 - 1.8432 * In_m2**2 +  288.9068*In_m2 + 1.8032e+03)

    sign_m3 = np.sign(In_m3)
    self.MP_pwm[2] = sign_m3 * (np.sqrt(sign_m3*In_m3*8.308971069381441e-9+9.542541377640133e-10)*2.407036904208272e+8-5.608689869160311e+3)
    # self.MP_pwm[2] = sign_m3 * (-8.6533e-06 * In_m3**4 + sign_m3*  0.0068 * In_m3**3 -1.8811 * In_m3**2 + 290.2357*In_m3 + 1.8179e+03)

    sign_m4 = np.sign(In_m4)
    self.MP_pwm[3] = sign_m4 * (np.sqrt(sign_m4*In_m4*7.579233605794426e-9+1.571593599291622e-9)*2.638789228598223e+8-8.330685304890129e+3)
    # self.MP_pwm[3] = sign_m4 * (-9.6405e-06 * In_m4**4 + sign_m4* 0.0073 * In_m4**3 -1.9698 * In_m4**2 +  298.3715*In_m4 + 1.6579e+03)

    In_m1_torque = -min(0.04, self.MP_gf_Y[2])
    In_m2_torque = min(0.04,  self.MP_gf_Y[2])
    In_m3_torque = -min(0.04, self.MP_gf_Y[2])
    In_m4_torque = min(0.04,  self.MP_gf_Y[2])

    sign_m1 = np.sign(In_m1_torque)
    # poly 4 map
    self.MP_pwm[0] += sign_m1 * (sign_m1*In_m1_torque*1.972462809575412e+6-In_m1_torque**2.0*8.540142042526409e+7+sign_m1*In_m1_torque**3.0*2.160322149978045e+9-In_m1_torque**4.0*1.934880389335135e+10+1.18794316827178e+3)

    # route map
    # self.MP_pwm[0] += sign_m1 * (np.sqrt(sign_m1 *In_m1_torque*1.006001536082039e-10+1.587299594622495e-13)*1.988068534953908e+10-7.920649662566478e+3)


    sign_m2 = np.sign(In_m2_torque)
    # poly 4 map
    self.MP_pwm[1] += sign_m2 * (sign_m2*In_m2_torque*2.258409150995067e+6-In_m2_torque**2.0*1.194174502034637e+8+sign_m2*In_m2_torque**3.0*3.622150201229846e+9-In_m2_torque**4.0*3.829605650204248e+10+1.505118375565417e+3)
    
    # route map
    # self.MP_pwm[1] += sign_m2 * (np.sqrt(sign_m2 *In_m2_torque*9.66189557532568e-11+1.932237720338033e-13)*2.069987182543716e+10-9.09908944642907e+3)


    sign_m3 = np.sign(In_m3_torque)
    # poly 4 map
    self.MP_pwm[2] += sign_m3 * (sign_m3*In_m3_torque*1.900760294497556e+6-In_m3_torque**2.0*7.296413958824399e+7+sign_m3*In_m3_torque**3.0*1.75802477736957e+9-In_m3_torque**4.0*1.537286308440307e+10+1.140378949262509e+2)

    # route map
    # self.MP_pwm[2] += sign_m3 * (np.sqrt(sign_m3 *In_m3_torque*1.12726263933792e-10+1.24935566032467e-13)*1.774209425741873e+10-6.271160652750821e+3)



    sign_m4 = np.sign(In_m4_torque)
    # poly 4 map
    self.MP_pwm[3] += sign_m4 * (sign_m4*In_m4_torque*1.924196710241383e+6-In_m4_torque**2.0*8.409577635107751e+7+sign_m4*In_m4_torque**3.0*2.163985380337182e+9-In_m4_torque**4.0*1.955878508632528e+10+1.330285503005834e+3)

    # route map
    # self.MP_pwm[3] += sign_m4 * (np.sqrt(sign_m4 *In_m4_torque*8.969381778123772e-11+1.667909502882046e-13)*2.229808084296265e+10-9.106546870860521e+3)


    



    


