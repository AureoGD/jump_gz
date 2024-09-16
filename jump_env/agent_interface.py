import sys
import os
import random
import time

import test


import numpy as np
from gz.transport13 import Node
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.double_v_pb2 import Double_V

from icecream import ic

#### import the custom msg ####
current_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_path + "/../jump_cmsgs/build/jump_cmsgs-msgs_genmsg/python")

from jump.msgs.lowcmd_pb2 import LowCmd
from jump.msgs.teste_pb2 import Teste


class AgenteInterface:
    def __init__(self):
        self.node = Node()

        self.req_empty_msg = Empty()
        self.res_doubleV_msg = Double_V()

        self.req_boolean = Boolean()
        self.res_boolean = Boolean()

    #     def reset(self):
    #         # chama o serviço que reseta o modelo no controlador
    #         pass

    #     def action(self):
    #         # chama o serviço que executa uma ação
    #         pass

    #     def observation(self):
    #         # chama o serviço que coleta os estados de interesse
    #         pass

    #     def reward(self):
    #         # cálcula a recompensar
    #         pass

    # def normalizeReward(self):
    #     # normaliza o vetor de recopensa
    #     pass

    def requestSensorSt(self):
        result, self.res_doubleV_msg = self.node.request(
            "jump/adapter/req_states", self.req_empty_msg, Empty, Double_V, 1000
        )

        if result:
            print(self.res_doubleV_msg)

    def requestData(self):
        self.req_boolean.data = True

        result, self.res_boolean = self.node.request(
            "/ser/teste", self.req_boolean, Boolean, Boolean, 1000
        )

        if result:
            print(self.res_boolean.data)


ag = AgenteInterface()

ag.requestSensorSt()
ag.requestData()
