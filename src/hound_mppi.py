#!/usr/bin/env python3
import numpy as np
import torch
from BeamNGRL.control.UW_mppi.MPPI import MPPI
from BeamNGRL.control.UW_mppi.Dynamics.SimpleCarDynamicsCUDA import SimpleCarDynamics
from BeamNGRL.control.UW_mppi.Costs.SimpleCarCost import SimpleCarCost
from BeamNGRL.control.UW_mppi.Sampling.Delta_Sampling import Delta_Sampling
from BeamNGRL.utils.visualisation import costmap_vis
import yaml
import cv2

torch.manual_seed(0)


class mppi:
    def __init__(self, Config):
        self.Dynamics_config = Config["Dynamics_config"]
        self.Cost_config = Config["Cost_config"]
        self.Sampling_config = Config["Sampling_config"]
        self.MPPI_config = Config["MPPI_config"]
        self.Map_config = Config["Map_config"]
        self.map_res = self.Map_config["map_res"]
        self.map_size = self.Map_config["map_size"]

        self.DEBUG = Config["debug"]

        self.dtype = torch.float
        self.device = torch.device("cuda")
        self.all_bad = False

        dynamics = SimpleCarDynamics(
            self.Dynamics_config, self.Map_config, self.MPPI_config
        )
        costs = torch.jit.script(SimpleCarCost(self.Cost_config, self.Map_config))
        sampling = Delta_Sampling(self.Sampling_config, self.MPPI_config)
        self.mppi = MPPI(dynamics, costs, sampling, self.MPPI_config)
        self.mppi.reset()
        self.print_states = None
        self.default_max_thr = self.Sampling_config["max_thr"] # set the default value. consider using a deep-copy?

    def set_hard_limit(self, hard_limit):
        self.Sampling_config["max_thr"] = min(
            hard_limit / self.Dynamics_config["throttle_to_wheelspeed"],
            self.default_max_thr,
        )
        self.mppi.Sampling.max_thr = torch.tensor(
            self.Sampling_config["max_thr"], device=self.device, dtype=self.dtype
        )

    def update(self, state, goal, map_elev, map_norm, map_cost, map_cent, speed_limit):
        ## get robot_centric BEV (not rotated into robot frame)
        BEV_heght = torch.from_numpy(map_elev).to(device=self.device, dtype=self.dtype)
        BEV_normal = torch.from_numpy(map_norm).to(device=self.device, dtype=self.dtype)
        BEV_path = torch.from_numpy(map_cost).to(device=self.device, dtype=self.dtype)

        self.mppi.Dynamics.set_BEV_numpy(map_elev, map_norm)
        self.mppi.Costs.set_BEV(BEV_heght, BEV_normal, BEV_path)
        self.mppi.Costs.set_goal(
            torch.from_numpy(np.copy(goal) - np.copy(map_cent)).to(
                device=self.device, dtype=self.dtype
            )
        )
        self.mppi.Costs.speed_target = torch.tensor(
            speed_limit, device=self.device, dtype=self.dtype
        )

        state_to_ctrl = np.copy(state)
        state_to_ctrl[:3] -= map_cent
        action = np.array(
            self.mppi.forward(
                torch.from_numpy(state_to_ctrl).to(device=self.device, dtype=self.dtype)
            )
            .cpu()
            .numpy(),
            dtype=np.float64,
        )[0]
        _, indices = torch.topk(self.mppi.Sampling.cost_total, k=10, largest=False)
        min_cost = torch.min(self.mppi.Sampling.cost_total)
        if min_cost.item() > 1000.0:
            self.all_bad = True
            action[1] = 0.0  ## recovery behavior
        else:
            self.all_bad = False

        self.print_states = self.mppi.Dynamics.states[:, indices, :, :3].cpu().numpy()
        if self.DEBUG:
            costmap_vis(
                self.print_states,
                state[:2] - map_cent[:2],
                goal[:2] - map_cent[:2],
                cv2.applyColorMap(
                    ((map_elev + 4) * 255 / 8).astype(np.uint8), cv2.COLORMAP_JET
                ),
                1 / self.map_res,
            )

        action[1] = np.clip(
            action[1], self.Sampling_config["min_thr"], self.Sampling_config["max_thr"]
        )
        return action
