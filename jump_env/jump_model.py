#### other imports ####
from math import pi, cos, sin, sqrt
import numpy as np

import sys
import os

#### icecream for debug ####
from icecream import ic

import random


class CreatVectorData(object):
    def __init__(self, d_lenght, d_startpos):
        # self.data = np.random.rand(d_lenght, 1)
        self.data = np.zeros((d_lenght, 1), dtype=np.double)
        self.startpos = d_startpos
        self.lenght = d_lenght

    def UpdateData(self, d_source):
        for i in range(self.lenght):
            self.data[i] = d_source.data[self.startpos + i]


class KinoDynModel(object):
    def __init__(self):
        self.HT_foot = np.zeros((4, 4), dtype=np.double)
        self.HT_com1 = np.zeros((4, 4), dtype=np.double)
        self.HT_com2 = np.zeros((4, 4), dtype=np.double)

        self.M = np.zeros((2, 2), dtype=np.double)
        self.C = np.zeros((2, 2), dtype=np.double)
        self.J_com = np.zeros((2, 2), dtype=np.double)
        self.J_com1 = np.zeros((2, 2), dtype=np.double)
        self.J_com2 = np.zeros((2, 2), dtype=np.double)
        self.J_foot = np.zeros((2, 2), dtype=np.double)

        self.q = np.zeros((2, 1), dtype=np.double)
        self.dq = np.zeros((2, 1), dtype=np.double)
        self.G = np.zeros((2, 1), dtype=np.double)
        self.com_pos = np.zeros((2, 1), dtype=np.double)
        self.com_pos_w = np.zeros((2, 1), dtype=np.double)
        self.foot_pos = np.zeros((2, 1), dtype=np.double)

        self.M[1, 1] = 0.01323
        self.C[1, 0] = 0

        self.HT_foot[3, 3] = 1
        self.HT_com1[3, 3] = 1
        self.HT_com2[3, 3] = 1

        self.HT_foot[1, 2] = 1
        self.HT_com1[1, 2] = 1
        self.HT_com2[1, 2] = 1

        self.m_base = 30.0
        self.m_upr = 1.5
        self.m_lwr = 1.5

        self.m_total = self.m_lwr + self.m_upr + self.m_base

    def DynamicMtxs(self):
        self.M[0, 0] = 0.1348935 + 0.07182 * cos(self.q[1, 0])
        self.M[0, 1] = 0.01323 + 0.03591 * cos(self.q[1, 0])
        self.M[1, 0] = 0.01323 + 0.03591 * cos(self.q[1, 0])

        self.C[0, 0] = -0.07182 * sin(self.q[1, 0]) * self.dq[1, 0]
        self.C[0, 1] = -0.03591 * sin(self.q[1, 0]) * self.dq[1, 0]
        self.C[1, 0] = 0.03591 * sin(self.q[1, 0]) * self.dq[0, 0]

        self.G[0, 0] = 0.5325 * sin(self.q[0, 0]) + 0.126 * sin(
            self.q[0, 0] + self.q[1, 0]
        )
        self.G[1, 0] = 0.126 * sin(self.q[0, 0] + self.q[1, 0])

    def KinematicMtxs(self):
        self.HT_foot[0, 0] = -sin(self.q[0, 0] + self.q[1, 0])
        self.HT_foot[0, 1] = -cos(self.q[0, 0] + self.q[1, 0])
        self.HT_foot[0, 3] = -0.25 * sin(self.q[0, 0] + self.q[1, 0]) - 0.2850 * sin(
            self.q[0, 0]
        )
        self.HT_foot[2, 0] = -cos(self.q[0, 0] + self.q[1, 0])
        self.HT_foot[2, 2] = sin(self.q[0, 0] + self.q[1, 0])
        self.HT_foot[2, 3] = (
            -0.25 * cos(self.q[0, 0] + self.q[1, 0])
            - 0.2850 * cos(self.q[0, 0])
            - 0.125
        )

        self.HT_com1[0, 0] = -sin(self.q[0, 0])
        self.HT_com1[0, 1] = -cos(self.q[0, 0])
        self.HT_com1[0, 3] = -0.127 * sin(self.q[0, 0])
        self.HT_com1[2, 0] = -cos(self.q[0, 0])
        self.HT_com1[2, 2] = sin(self.q[0, 0])
        self.HT_com1[2, 3] = -0.127 * cos(self.q[0, 0]) - 0.125

        self.HT_com2[0, 0] = -sin(self.q[0, 0] + self.q[1, 0])
        self.HT_com2[0, 1] = -cos(self.q[0, 0] + self.q[1, 0])
        self.HT_com2[0, 3] = -0.105 * sin(self.q[0, 0] + self.q[1, 0]) - 0.2850 * sin(
            self.q[0, 0]
        )
        self.HT_com2[2, 0] = -cos(self.q[0, 0] + self.q[1, 0])
        self.HT_com2[2, 2] = sin(self.q[0, 0] + self.q[1, 0])
        self.HT_com2[2, 3] = (
            -0.105 * cos(self.q[0, 0] + self.q[1, 0])
            - 0.2850 * cos(self.q[0, 0])
            - 0.125
        )

        self.J_com1[0, 0] = -0.127 * cos(self.q[0, 0])
        self.J_com1[1, 0] = 0.127 * sin(self.q[0, 0])

        self.J_com2[0, 0] = -0.105 * cos(self.q[0, 0] + self.q[1, 0]) - 0.2850 * cos(
            self.q[0, 0]
        )
        self.J_com2[0, 1] = -0.105 * cos(self.q[0, 0] + self.q[1, 0])
        self.J_com2[1, 0] = 0.105 * sin(self.q[0, 0] + self.q[1, 0]) + 0.2850 * sin(
            self.q[0, 0]
        )
        self.J_com2[1, 1] = 0.105 * sin(self.q[0, 0] + self.q[1, 0])

        self.J_foot[0, 0] = -0.25 * cos(self.q[0, 0] + self.q[1, 0]) - 0.2850 * cos(
            self.q[0, 0]
        )
        self.J_foot[0, 1] = -0.25 * cos(self.q[0, 0] + self.q[1, 0])
        self.J_foot[1, 0] = 0.25 * sin(self.q[0, 0] + self.q[1, 0]) + 0.2850 * sin(
            self.q[0, 0]
        )
        self.J_foot[1, 1] = 0.25 * sin(self.q[0, 0] + self.q[1, 0])

    def updateModelMtx(self, b, q, dq):
        self.q = q
        self.dq = dq

        self.KinematicMtxs()
        self.DynamicMtxs()

        # evaluate the CoM position (w.r.t)

        self.com_pos[0, 0] = (
            self.HT_com1[0, 3] * self.m_upr + self.HT_com2[0, 3] * self.m_lwr
        ) / self.m_total
        self.com_pos[1, 0] = (
            self.HT_com1[2, 3] * self.m_upr + self.HT_com2[2, 3] * self.m_lwr
        ) / self.m_total

        self.com_pos_w = (b * self.m_base / self.m_total) + self.com_pos

        self.J_com = (
            self.m_upr * self.J_com1 + self.m_lwr * self.J_com2
        ) / self.m_total

        self.com_vel = np.matmul(self.J_com, self.dq)

        self.foot_pos[0, 0] = self.HT_foot[0, 3]
        self.foot_pos[1, 0] = self.HT_foot[2, 3]

        self.foot_vel = np.matmul(self.J_foot, self.dq)


class JumpModel(object):
    def __init__(self):
        # TODO import this infos from the SDF file
        self.joint_name = ["hfe", "kfe", "base"]
        self.joint_type = ["revolute", "revolute", "prismatic"]
        self.joint_p_max = [0 * pi / 180, 120 * pi / 180, 0.7]
        self.joint_p_min = [-55 * pi / 180, 60 * pi / 180, 0.4]

        self.KDmdl = KinoDynModel()

        self.step_length = 10

        # robot states
        self.q = np.zeros((2, 1), dtype=np.double)
        self.dq = np.zeros((2, 1), dtype=np.double)
        self.qr = np.zeros((2, 1), dtype=np.double)
        self.dqr = np.zeros((2, 1), dtype=np.double)
        self.b = np.zeros((2, 1), dtype=np.double)
        self.db = np.zeros((2, 1), dtype=np.double)
        self.tau = np.zeros((2, 1), dtype=np.double)
        self.fContSt = 0

        #
        self.N = 10
        self.actions = -np.ones((self.N, 1))

        self.nn_states_input_dim = 19
        self.n_actions = 6

        qd_max = 20
        tau_max = 100
        q1_max = 1.57
        q2_max = 2.18
        lvel_max = 5
        self.alfa = np.diagflat(
            [
                1,
                1,
                0.5 / lvel_max,
                0.5 / lvel_max,
                1,
                1,
                0.5 / lvel_max,
                0.5 / lvel_max,
                0.5 / q1_max,
                0.5 / q2_max,
                0.5 / qd_max,
                0.5 / qd_max,
                0.5 / q1_max,
                0.5 / q2_max,
                0.5 / tau_max,
                0.5 / tau_max,
                1 / self.N,
                1 / 10,
                1,
            ]
        )

        self.beta = np.array(
            [
                [0],
                [0],
                [0.5],
                [0.5],
                [0],
                [0],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0.5],
                [0],
                [0.1],
                [0],
            ]
        )

        self.solver_status = 0

    def robotStates(self, msg):
        self.b[1] = msg.q.data[0]
        self.db[1] = msg.dq.data[0]

        self.q[0] = msg.q.data[1]
        self.q[1] = msg.q.data[2]

        self.dq[0] = msg.dq.data[1]
        self.dq[1] = msg.dq.data[2]

        self.tau[0] = msg.tau.data[1]
        self.tau[1] = msg.tau.data[2]

        self.qr[0] = msg.qr.data[0]
        self.qr[1] = msg.qr.data[1]

        self.dqr[0] = msg.dqr.data[0]
        self.dqr[1] = msg.dqr.data[1]

        self.fContSt = msg.fc

        self.KDmdl.updateModelMtx(self.b, self.q, self.dq)

    def randonJointPos(self):
        q = np.zeros((len(self.joint_name), 1), dtype=np.double)
        # ensure that the feet is not inside the ground
        while (
            q[2, 0] - 0.25 * cos(q[0, 0] + q[1, 0]) - 0.2850 * cos(self.q[0, 0]) - 0.125
            < 0.1
        ):
            for index in range(len(q)):
                q[index] = random.uniform(
                    self.joint_p_min[index],
                    self.joint_p_max[index],
                )
        # ic(q[0, 0])
        return q

    def normalize_states(self, states):
        normalized_states = np.clip(
            np.matmul(self.alfa, states.reshape(self.nn_states_input_dim, 1))
            + self.beta,
            0,
            1,
        )
        return normalized_states

    def reward(self):
        pass

    def done(self):
        pass

    def observation(self):
        # r, dr, fp, dfp, q, dq, qr, tau,ac, trans_val
        self.transition_history = self.CheckTransition()
        states = np.vstack(
            (
                self.KDmdl.com_pos_w,
                self.KDmdl.com_vel,
                self.KDmdl.foot_pos,
                self.KDmdl.foot_vel,
                self.q,
                self.dq,
                self.qr,
                self.tau,
                self.transition_history,
                self.actions[0],
                np.array([self.fContSt], dtype=np.double),
            )
        )
        # ic(states)
        return (self.normalize_states(states)).reshape((self.nn_states_input_dim, 1))

    def actionList(self, action):
        self.actions[:] = np.roll(self.actions, 1)
        self.actions[0] = action

    def CheckTransition(self):
        count = 0
        for i in range((len(self.actions) - 1), 0, -1):
            if self.actions[i] != self.actions[i - 1] and self.actions[i] != -1:
                count += 1
        # if count == 1 or count == 0:
        #     return 0
        # else:
        return count
