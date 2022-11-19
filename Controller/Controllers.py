import sys
sys.path.append('../')

import numpy as np


from Controller.Pid_Controller import Pid_Controller
from Controller.QCSL_controller import Quad_with_Cable_Suspended as QCSL

class Controllers():

  def __init__(self, dt, controller_mode, mode="position", traj="circle"):
    print("initialize Controller")
    

    self.dt = dt
    self.mode = mode
    self.traj = traj
    self.controller_mode = controller_mode
    self.pid_controller = Pid_Controller(self.dt, self.mode)
    self.QCSL_controller = QCSL(self.dt)


    self.input_thrust_gf = 0.0
    self.input_M_gf = np.array([0.0, 0.0, 0.0])
    
    self.input_thrust_pwm = 0.0
    self.input_M_pwm = np.array([0.0, 0.0, 0.0])

    self.gravity_calcel = 9.81
    self.rad2deg = 180/np.pi

  def select_controller(self):

    if self.controller_mode == "pid":
      self.controller = self.pid_controller
      self.set_reference = self.pid_controller.set_reference
      self.cal_output = self.pid_controller.controller_position_pid
      self.set_state = self.pid_controller.set_state
      self.init_controller = self.pid_controller.pid_init
      self.log = self.pid_controller.log_nom
      self.set_dt = self.pid_controller.set_dt

    elif self.controller_mode == "QCSL":
      self.QCSL_controller = QCSL(self.dt)
      self.controller = self.QCSL_controller
      self.cal_output = self.QCSL_controller.qcsl_ctrl
      self.set_state = self.QCSL_controller.set_state
      self.init_controller = self.QCSL_controller.qcsl_init
      self.set_reference = self.QCSL_controller.set_reference
      self.stop_tracking = self.QCSL_controller.stop_tracking
      self.log = self.QCSL_controller.log_nom
      self.set_dt = self.QCSL_controller.set_dt
    self.init_controller()
  
  def switch_controller(self, controller_type):

    if controller_type == "pid":
      self.pid_controller = Pid_Controller(self.dt, self.mode)
      self.controller = self.pid_controller
      self.set_reference = self.pid_controller.set_reference
      self.cal_output = self.pid_controller.controller_position_pid
      self.set_state = self.pid_controller.set_state
      self.init_controller = self.pid_controller.pid_init
      self.log = self.pid_controller.log_nom
      self.set_dt = self.pid_controller.set_dt

    
    elif controller_type == "QCSL":
      self.QCSL_controller = QCSL(self.dt)
      self.controller = self.QCSL_controller
      self.cal_output = self.QCSL_controller.qcsl_ctrl
      self.set_state = self.QCSL_controller.set_state
      self.init_controller = self.QCSL_controller.qcsl_init
      self.set_reference = self.QCSL_controller.set_reference
      self.stop_tracking = self.QCSL_controller.stop_tracking
      self.log = self.QCSL_controller.log_nom
      self.set_dt = self.QCSL_controller.set_dt


    self.init_controller()
  
  def get_output(self, t):
    self.cal_output(t)
    self.input_acc = self.controller.input_acc
    self.input_Wb = self.controller.input_Wb

  def controller_trajectory_tracking(self, refs):
    print("Trajectory tracking controller")

  