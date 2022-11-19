import numpy as np

class State():
  def __init__(self, dt, initial):
    print("Initialize State")
    self.dt = float(dt)
    self.pre = initial
    self.now = initial
    self.pre = 0.0

    self.deriv = 0.0
  
  def init_drone_state(self, t=0, P=np.array([0.0, 0.0, 0.0]),   
                                V=np.array([0.0, 0.0, 0.0]), 
                                R=np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), 
                                Euler=np.array([0.0, 0.0, 0.0]), 
                                Wb=np.array([0.0, 0.0, 0.0]), 
                                Euler_rate=np.array([0.0, 0.0, 0.0]),
                                Moter = np.array([0.0 ,0.0, 0.0, 0.0])):
    self.t = t
    self.P = P
    self.V = V
    self.Euler = Euler
    self.R = R
    self.Wb = Wb
    self.Euler_rate = Euler_rate
    self.Moter = Moter

  
  def set_drone_state(self, t, P, V, R, Euler, Wb, Euler_rate, Moter):
    self.t = t
    self.P = P
    self.V = V
    self.R = R
    self.Euler = Euler
    self.Wb = Wb
    self.Euler_rate = Euler_rate
    self.Moter = Moter

  def integration(self, d_state):
    self.pre = self.now
    self.now += d_state*self.dt    
  
  def derivertive(self):
    self.deriv = (self.now - self.pre)/self.dt
  
  def update(self, new):
    self.pre = self.now
    self.now = new

  def log_state(self, log):
    log.write_state(t=self.t, P=self.P, V=self.V, Euler=self.Euler, Wb=self.Wb, Euler_rate=self.Euler_rate, M=self.M)