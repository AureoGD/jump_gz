import sys
import os
import random
import time
from icecream import ic
import numpy as np


from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.int32_pb2 import Int32


#### import the custom msg ####
current_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_path + "/../jump_cmsgs/build/jump_cmsgs-msgs_genmsg/python")

from jump.msgs.lowcmd_pb2 import LowCmd
from jump.msgs.lowstates_pb2 import LowStates


class AgenteInterface:
    def __init__(self):
        self.node = Node()

        # request low states service variables
        self.req_boolean = Boolean()
        self.rep_lowStates = LowStates()

        # request low control service variables
        self.req_action = Int32()
        self.rep_lowCmd = LowCmd()

    def action(self, action):
        # chama o serviço que executa uma ação
        self.req_action.data = action
        result, self.rep_lowCmd = self.node.request(
            "/jump/RGC/low_cmd", self.req_action, Int32, LowCmd, 5000
        )
        ic(self.rep_lowCmd)

    def requestSensorSt(self):
        self.req_boolean.data = True

        result, self.rep_lowStates = self.node.request(
            "jump/low_state", self.req_boolean, Boolean, LowStates, 5000
        )

        ic("Result:", result)
        ic(self.rep_lowStates)

    def reset(self):
        # chama o serviço que reseta o modelo no controlador
        pass

    def observation(self):
        # chama o serviço que coleta os estados de interesse
        pass

    def reward(self):
        # cálcula a recompensar
        pass

    def normalizeReward(self):
        # normaliza o vetor de recopensa
        pass


ag = AgenteInterface()

ag.requestSensorSt()
ag.action(11)
