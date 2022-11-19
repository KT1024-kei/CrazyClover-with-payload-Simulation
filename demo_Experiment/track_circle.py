#!/user/bin/env python
# coding: UTF-8

import sys
sys.path.append('../')
import time
import numpy as np
import matplotlib.pyplot as plt

from demo_Experiment.Env_experiment import Env_Experiment
from Controller.Controllers import Controllers
from Drone.Drone_with_Load_model import Drone_with_cable_suspended

def Experiment(Texp, Tsam, num_drone):

    Env = Env_Experiment(Texp, Tsam, 0)
    Drone_env = [0]*num_drone
    Drone_ctrl = [0]*num_drone
    Drone_log = [0]*num_drone
    # swarm = Crazyswarm()
    # timeHelper = swarm.timeHelper
    # cf = swarm.allcfs.crazyflies


    zero = np.zeros(3)
    for i in range(num_drone):
        Drone_env[i] = Env_Experiment(Texp, Tsam, i)
        Drone_ctrl[i] = Controllers(Tsam, "QCSL")
        Drone_env[i].track_hover_payload(Drone_ctrl[i], True)

    cf = [0]*num_drone
    for i in range(num_drone):
      cf[i] = Drone_with_cable_suspended(Tsam)
      Drone_env[i].init_state(cf[i], P=np.array([0.1, 0.0, 0.0]))
      Drone_env[i].init_plot(None)

    circle_flag = True
    t = 0
    cnt = 0
    while True:

        for i in range(num_drone):
            Drone_env[i].take_log(t, Drone_ctrl[i])

        for i in range(num_drone):
            Drone_env[i].update_state(cf[i])

        if t > 3.0:
            for i in range(num_drone):
                if circle_flag:
                    Drone_ctrl[i].switch_controller("QCSL")
                    Drone_env[i].track_circle(Drone_ctrl[i])
                    circle_flag = False
        
        for i in range(num_drone):
            Drone_ctrl[i].set_state(Drone_env[i].P, Drone_env[i].V, Drone_env[i].R, Drone_env[i].Euler, Drone_env[i].L, Drone_env[i].dL, Drone_env[i].q, Drone_env[i].dq)
            Drone_ctrl[i].get_output(t)

        
        for i in range(num_drone):
            cf[i].main(Drone_ctrl[i].input_acc, Drone_ctrl[i].input_Wb)


        for i in range(num_drone):
            if cnt/100 == 1:
                Drone_env[i].update_plot(cf[i].world_frame)
                plt.pause(Tsam*100)
                cnt = 0
            cnt += 1

        if Env.time_check(t, Tsam, Texp): break
        t += Tsam
    
    for i in range(num_drone):
        Drone_env[i].save_log()
if __name__ == "__main__":
  Experiment(50, 0.001, 1)




