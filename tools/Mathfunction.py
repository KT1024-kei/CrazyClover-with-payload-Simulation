import numpy as np

class Mathfunction():
  # --------------- function related to derivative polynominal --------------
  def time_polyder(self, t, k, order):
      terms = np.zeros(order)
      coeffs = np.polyder([1]*order,k)[::-1]
      pows = t**np.arange(0,order-k,1)
      terms[k:] = coeffs*pows
      return terms
  # --------------- Pysical math function ------------------
  def Vee(self, Vector):
    return np.array([[0, -Vector[2], Vector[1]], [Vector[2], 0, -Vector[0]], [-Vector[1], Vector[0], 0]])

  def Wedge(self, Mat):
    return np.array([Mat[2, 1], -Mat[2, 0], Mat[1, 0]])

  def saturation(self, param, UP_limit, LOW_limit):

    return max(min(param, UP_limit), LOW_limit)
  
  # Body angular velocity to Euler angle rate
  def BAV2EAR(self, Euler, Wb): 
    r = Euler[0]
    p = Euler[1]
    y = Euler[2]

    cosR = np.cos(r);sinR = np.sin(r)
    cosP = np.cos(p);sinP = np.sin(p)

    Euler_angle_rate = np.matmul(np.linalg.inv(np.array([[1.0, 0.0, -sinP],
                                        [0.0, cosR,  cosP * sinR],
                                        [0.0, -sinR, cosP * cosR]])) , Wb)
    # Euler_angle_rate = np.matmul(np.array([[cosR, sinR, 0.0],
    #                                       [-cosP*sinR, cosP*cosR, 0.0],
    #                                       [sinP*cosR, sinP*sinR, cosP]])/cosP, Wb)
  
    return Euler_angle_rate

  def EAR2BAV(self, Euler, Euler_rate):
    r = Euler[0]
    p = Euler[1]
    y = Euler[2]

    cosR = np.cos(r);sinR = np.sin(r)
    cosP = np.cos(p);sinP = np.sin(p)

    Wb = np.matmul((np.array([[1.0, 0.0, -sinP],
                                        [0.0, cosR,  cosP * sinR],
                                        [0.0, -sinR, cosP * cosR]])) , Euler_rate)
    
    return Wb
  
  def Euler2Rot(self, Euler):
    print(Euler)
    r = Euler[0]; p = Euler[1]; y = Euler[2]
    cosR = np.cos(r); sinR = np.sin(r)
    cosP = np.cos(p); sinP = np.sin(p)
    cosY = np.cos(y); sinY = np.sin(y)

    R1 = np.matrix(([cosR,-sinR,0],
                    [sinR,cosR,0],
                    [0,0,1]))
    
    R2 = np.matrix(([cosP,0,sinP],
                    [0,1,0],
                    [-sinP,0,cosP]))
    
    R3 = np.matrix(([cosY,-sinY,0],
                    [sinY,cosY,0],
                    [0,0,1]))
    
    return np.array(R3*R2*R1)

class RowPath_Filter():
  
  def Init_LowPass2D(self, fc): # cut off frecency, sampling rate
    
    self.fc = fc
    self.r0 = np.zeros(3)
    self.r1 = np.zeros(3)
    self.r2 = np.zeros(3)

  def LowPass2D(self, V0, dt):
    fr = 1.0/(self.fc * dt)     
    omega = np.tan(np.pi/fr)

    c = 1.0 + 2.0/np.sqrt(2)*omega + omega**2
    b0 = omega**2/c
    b1 = 2.0*self.b0
    b2 = self.b0

    a1 = 2*(omega**2 - 1)/c
    a2 = (1.0 - 2.0/np.sqrt(2)*omega + omega**2)/c
    
    self.r0 = V0 - a1*self.r1 - a2*self.r2
    Vout = b0*self.r0 + b1*self.r1 + b2*self.r2

    self.r2 = self.r1
    self.r1 = self.r0

    return Vout
    
class Integration():
  
  def __init__(self, dt, param):
    self.dt = dt
    self.integral = param

  def integration(self, param):
    self.integral += param*self.dt    
    return self.integral