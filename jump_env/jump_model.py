#### other imports ####
from math import pi, cos, sqrt
import numpy as np

import sys
import os

#### icecream for debug ####
from icecream import ic

import pinocchio

current_path = os.path.dirname(os.path.abspath(__file__))
# sys.path.append(current_path + "/../jump_model")
pinocchio_model_dir = os.path.join(current_path + "/../jump_model")

urdf_filename = pinocchio_model_dir + "/model.sdf" if len(sys.argv) < 2 else sys.argv[1]

model = pinocchio.buildModelsFromSdf(urdf_filename)
print("model name: " + model.name)


class CreatVectorData(object):
    def __init__(self, d_lenght, d_startpos):
        # self.data = np.random.rand(d_lenght, 1)
        self.data = np.zeros((d_lenght, 1), dtype=np.double)
        self.startpos = d_startpos
        self.lenght = d_lenght

    def UpdateData(self, d_source):
        for i in range(self.lenght):
            self.data[i] = d_source.data[self.startpos + i]


class JumpModel(object):
    def __init__(self):
        # TODO import this infos from the SDF file
        self.joint_name = ["hfe", "kfe", "base"]
        self.joint_type = ["revolute", "revolute", "prismatic"]
        self.joint_p_max = [0 * pi / 180, 120 * pi / 180, 0.7]
        self.joint_p_min = [-55 * pi / 180, 60 * pi / 180, 0.4]

        nn_states_input_dim = 18
        n_actions = 6
        alpha = 0.0001
        gamma = 0.99
        epsilon = 1.0
        batch_size = 64
        epsilon_decay = 5e-6
        epsilon_min = 0.1
        mem_size = 50000
        replace_target = 1000

        self.nn_input_dim = nn_states_input_dim

        # Predict horizon
        self.N = 10

        # sample time of the simulation
        sim_ts = 0.001
        # duration of one episode (seconds)
        episode_duration = 5

        self.dt = self.N * sim_ts

        # number of the current episode
        self.n_ep = 0
        # maximum episodes
        self.max_ep = 5000
        # number of episodes to save the model
        self.save_model_every = 10
        # number of the current step
        self.steps = 0
        # how many "post_update" is a step
        self.step_size = 10
        # how many steps is an episode
        self.max_steps = episode_duration / (sim_ts * self.step_size)

        # vector of the last states
        self.last_states = np.zeros((nn_states_input_dim, 1))
        # self.teste_v = np.zeros((nn_states_input_dim + 1, 1))
        # vector of the actual state
        self.actual_state = np.zeros((nn_states_input_dim, 1))

        # vector of the last self.N actions
        self.actions = -np.ones((self.N, 1))

        # vector os the states
        self.base_pos = CreatVectorData(2, 0)  # 0  1
        self.base_vel = CreatVectorData(2, 2)  # 2  3
        self.com_pos = CreatVectorData(2, 4)  # 4  5
        self.com_vel = CreatVectorData(2, 6)  # 6  7
        self.foot_pos = CreatVectorData(2, 8)  # 8  9
        self.foot_vel = CreatVectorData(2, 10)  # 10 11
        self.joint_pos = CreatVectorData(2, 12)  # 12 13
        self.joint_vel = CreatVectorData(2, 14)  # 14 15
        self.joint_tau = CreatVectorData(2, 16)  # 16 17
        self.joint_refh = CreatVectorData(2, 18)  # 18 19
        self.joint_refl = CreatVectorData(2, 20)  # 20 21
        self.cons_viol = CreatVectorData(1, 22)  # 22
        self.foot_con = CreatVectorData(1, 23)  # 23
        self.npo = CreatVectorData(1, 24)  # 24

        self.states_list = [
            self.base_pos,
            self.base_vel,
            self.com_pos,
            self.com_vel,
            self.foot_pos,
            self.foot_vel,
            self.joint_pos,
            self.joint_vel,
            self.joint_tau,
            self.joint_refh,
            self.joint_refl,
            self.cons_viol,
            self.foot_con,
            self.npo,
        ]

        self.base_vel_a = np.array(self.base_vel.data.shape)
        self.foot_vel_a = np.array(self.foot_vel.data.shape)

        self.episode_reward = 0
        self.episode_base_distance = 0
        self.episode_foot_distance = 0
