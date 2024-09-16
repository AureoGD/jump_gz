import numpy as np
import gymnasium as gym
from gymnasium import spaces

from gz.transport13 import Node
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean


class GzTrainning(gym.Env):
    def __init__(self, N_ACTIONS, N_OBSERVATIONS):
        super().__init__()
        self.node = Node()
        # cria um objeto do que é desejado simular
        # self.model = JumpController()
        # self.action_space = spaces.Discrete(N_ACTIONS)
        # self.observation_space = spaces.Box(low=0, high=1, shape=(N_OBSERVATIONS,), dtype=np.float32)

        # talvez algo do tipo
        # self.action_space = spaces.Discrete(model.N_ACTIONS)
        # self.observation_space = spaces.Box(low=model.lowValue, high=model.highValue, shape=(model.N_OBSERVATIONS,), dtype=np.float32)

        self.world_control_msg = WorldControl()
        self.request_info = Boolean()
        self.timeout_reset = 1000
        self.world_control_msg.multi_step = 10  # model.step_size
        self.world_control_msg.step = True
        self.world_control_msg.pause = True

        self.observation = 0
        self.reward = 0
        self.done = 0

    def step(self, action):
        # envia a ação para o gz, ex:
        # self.model.action(action)

        # simula o ambiente por algumas interações
        result, self.response_reset = self.node.request(
            "/world/default/control",
            self.world_control_msg,
            WorldControl,
            Boolean,
            self.timeout_reset,
        )

        # self.observation = self.model.observation()
        # self.reward = self.model.reward()
        # self.done = self.model.done()
        info = True

        return self.observation, self.reward, self.done, info

    def reset(self):
        # self.model.reset()
        info = 0
        return self.observation, self.reward, self.done, info


tes = GzTrainning(5, 2)

tes.step(2)
