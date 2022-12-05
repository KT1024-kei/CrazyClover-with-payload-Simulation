"""
manage controllers
  - mellinger
  - PID

controller functions 
  - controller : have all controller functions
  - init_controller : initialize controller
  - set_reference : set reference commands
  - cal_output : calculate control input 
  - set_state : set states
  - log : take desired state and control input log
  - set_dt : set minute time

"""
import sys
sys.path.append('../')

import numpy as np

from Controller.Pid_Controller import Pid_Controller
from Controller.QCSL_controller import Quad_with_Cable_Suspended as QCSL
from Controller.Mellinger_controller import Mellinger

class Controllers():

  def __init__(self, dt, controller_mode, pid_mode="position", traj="circle"):

    # * set parametor
    self.dt = dt
    self.controller_mode = controller_mode
    self.mode = pid_mode

    # * initialize each controller
    self.pid_controller = Pid_Controller(self.dt, self.mode)
    self.mellinger_controller = Mellinger(self.dt)

  # ! select first controller 
  def select_controller(self):

    if self.controller_mode == "pid":
      self.controller = self.pid_controller
      self.set_reference = self.pid_controller.set_reference
      self.cal_output = self.pid_controller.controller_position_pid
      self.set_state = self.pid_controller.set_state
      self.init_controller = self.pid_controller.pid_init
      self.log = self.pid_controller.log_nom
      self.set_dt = self.pid_controller.set_dt

    elif self.controller_mode == "mellinger":
      self.controller = self.mellinger_controller
      self.cal_output = self.mellinger_controller.mellinger_ctrl
      self.set_state = self.mellinger_controller.set_state
      self.init_controller = self.mellinger_controller.mellinger_init
      self.set_reference = self.mellinger_controller.set_reference
      self.log = self.mellinger_controller.log_nom
      self.set_dt = self.mellinger_controller.set_dt

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
      self.controller_mode = "pid"
      self.pid_controller = Pid_Controller(self.dt, self.mode)
      self.controller = self.pid_controller
      self.set_reference = self.pid_controller.set_reference
      self.cal_output = self.pid_controller.controller_position_pid
      self.set_state = self.pid_controller.set_state
      self.init_controller = self.pid_controller.pid_init
      self.log = self.pid_controller.log_nom
      self.set_dt = self.pid_controller.set_dt

    elif controller_type == "mellinger":
      self.controller_mode = "mellinger"
      self.mellinger_controller = Mellinger(self.dt)
      self.controller = self.mellinger_controller
      self.cal_output = self.mellinger_controller.mellinger_ctrl
      self.set_state = self.mellinger_controller.set_state
      self.init_controller = self.mellinger_controller.mellinger_init
      self.set_reference = self.mellinger_controller.set_reference
      self.log = self.mellinger_controller.log_nom
      self.set_dt = self.mellinger_controller.set_dt

    elif controller_type == "QCSL":
      self.controller_mode = "QCSL"
      self.QCSL_controller = QCSL(self.dt)
      self.controller = self.QCSL_controller
      self.cal_output = self.QCSL_controller.qcsl_ctrl
      self.set_state = self.QCSL_controller.set_state
      self.init_controller = self.QCSL_controller.qcsl_init
      self.set_reference = self.QCSL_controller.set_reference
      self.stop_tracking = self.QCSL_controller.stop_tracking
      self.log = self.QCSL_controller.log_nom
      self.set_dt = self.QCSL_controller.set_dt

    else:
      print("{} not defined".format(controller_type))
      exit()

    self.init_controller()

  # ! calculate registered controller input 
  def get_output(self, t):
    self.cal_output(t)
    self.input_acc = self.controller.input_acc
    self.input_Wb = self.controller.input_Wb


  