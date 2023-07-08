import torch
import numpy as np
import time

from UW_mppi.MPPI import MPPI
from UW_mppi.Dynamics.SimpleCarDynamics import SimpleCarDynamics
from UW_mppi.Costs.SimpleCarCost import SimpleCarCost

class mppi_controller():
    def __init__(self, speed_max, steering_max, wheelbase, map_size, map_res):
        self.action = np.zeros(2)
        self.dtype = torch.float
        self.d = torch.device("cuda")
        self.dt = 0.02
        self.speed_max = speed_max
        self.map_size = map_size
        self.map_res = map_res
        self.wheelbase = wheelbase
        self.steering_max = steering_max
        with torch.no_grad():
            ## BEGIN MPPI
            ## potentially these things should be loaded in from some config file? Will torchscript work with that?
            dynamics = SimpleCarDynamics(
                wheelbase=1.0,
                speed_max=self.speed_max,
                steering_max=self.steering_max,
                dt=self.dt,
                BEVmap_size=map_size,
                BEVmap_res=map_res,
                ROLLOUTS=512,
                TIMESTEPS=32,
                BINS=1,
            )
            costs = SimpleCarCost(
                goal_w=1,
                speed_w=0,
                roll_w=0, ## weight on roll index, but also controls for lateral acceleration limits.. something to think about is how longitudenal accel affects accel limits..
                lethal_w=1, # weight on lethal stuff. Note that this is applied to a piecewise function which is = 1/cos(surface angle) for SA < thresh and 1000 for SA > thresh
                speed_target=10, ## target speed in m/s
                critical_SA=1/np.cos(0.5), # 0.5 is the critical slope angle, 1/cos(angle) is used for state cost evaluation
                critical_RI=1.0, ## limiting ratio of lateral to vertical acceleration
                BEVmap_size=map_size,
                BEVmap_res=map_res,
            )

            ns = torch.zeros((2, 2), device = self.d, dtype = self.dtype)
            ns[0, 0] = 1.0  # steering
            ns[1, 1] = 1.0  # throttle/brake

            self.controller = MPPI(
                dynamics,
                costs,
                CTRL_NOISE=ns,
                lambda_= 0.1,
            )

    def update(self, goal, state, map_cost, map_elev, map_norm, map_cent):
        BEV_heght = torch.from_numpy(map_elev).to(device=self.d, dtype=self.dtype)
        BEV_normal = torch.from_numpy(map_norm).to(device=self.d, dtype=self.dtype)
        BEV_cost = torch.from_numpy(map_cost).to(device=self.d, dtype=self.dtype)

        self.controller.Dynamics.set_BEV(BEV_heght, BEV_normal)
        self.controller.Costs.set_BEV(BEV_heght, BEV_normal)
        self.controller.Costs.set_goal(
            torch.from_numpy(goal - map_cent).to(device=self.d, dtype=self.dtype)
        )

        state[:3] -= map_cent # this is for the MPPI: technically this should be state[:3] -= BEV_center

        state[15:17] = self.action ## adhoc wheelspeed.
        state = torch.from_numpy(state).to(device=self.d, dtype=self.dtype)
        now = time.time()
        
        delta_action = self.controller(state)
        
        self.action += np.array(delta_action.cpu().numpy()[0], dtype=np.float64) * self.dt
        self.action = np.clip(self.action, -1, 1)
        return self.action