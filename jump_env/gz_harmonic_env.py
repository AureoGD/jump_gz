import numpy as np
import gymnasium as gym
from gymnasium import spaces

from gz.transport13 import Node
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean

import agent_interface


class GzTrainning(gym.Env):
    def __init__(
        self,
        render_mode=None,
    ):
        # super().__init__()
        self.node = Node()
        # create the model interface method
        self.model = agent_interface.AgenteInterface()

        self.action_space = spaces.Discrete(self.model.N_ACTIONS)
        self.observation_space = spaces.Box(
            low=self.model.obsLowValue,
            high=self.model.obsHighValue,
            shape=(self.model.N_OBSERVATIONS,),
            dtype=np.double,
        )

        self.world_control_msg = WorldControl()
        self.request_info = Boolean()
        self.timeout_reset = 1000
        self.world_control_msg.multi_step = self.model.step_length  # model.step_size
        self.world_control_msg.step = True
        self.world_control_msg.pause = True

        self.observation = 0
        self.reward = 0
        self.done = 0

    def step(self, action):
        # envia a ação para o gz, ex:
        self.model.action(action)

        # simula o ambiente por algumas interações
        result, self.response_reset = self.node.request(
            "/world/default/control",
            self.world_control_msg,
            WorldControl,
            Boolean,
            self.timeout_reset,
        )

        self.observation = self.model.observation()
        self.reward = self.model.reward()
        self.terminated = self.model.done()

        truncated = self.terminated
        info = {}

        return self.observation, self.reward, self.terminated, truncated, info

    def reset(self, seed=None):
        super().reset(seed=seed)
        self.observation = self.model.reset()
        info = {"ep": self.model.ep}
        return self.observation, info


# env = GzTrainning()

# env.step(0)
