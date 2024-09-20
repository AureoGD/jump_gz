import sys
import os
import random
import time
from icecream import ic
import numpy as np


from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.int32_pb2 import Int32
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.world_control_pb2 import WorldControl


#### import the custom msg ####
current_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_path + "/../jump_cmsgs/build/jump_cmsgs-msgs_genmsg/python")

from jump.msgs.lowcmd_pb2 import LowCmd
from jump.msgs.lowstates_pb2 import LowStates

import jump_model


class AgenteInterface:
    def __init__(self):
        self.robot_model = jump_model.JumpModel()

        self.node = Node()

        self.newRef_pub = self.node.advertise("jump/Control/low_cmd", LowCmd)

        # variables for request low states service
        self.req_boolean = Boolean()
        self.rep_lowStates = LowStates()

        # variables for request low control service
        self.req_action = Int32()
        self.rep_lowCmd = LowCmd()

        # variables for set new position

        self.req_pause = WorldControl()
        self.rep_pause = Boolean()

        self.req_JointPos = Double_V()
        self.rep_JointPos = Boolean()
        self.req_JointPos.data.extend(
            np.zeros((1, len(self.robot_model.joint_name))).tolist()[0]
        )

    def action(self, action):  # call the service for new action (requeste new qr)
        self.req_action.data = action

        result, self.rep_lowCmd = self.node.request(
            "/jump/RGC/low_cmd", self.req_action, Int32, LowCmd, 5000
        )
        ic(self.rep_lowCmd)

        if result:
            pass
            # update qr (service or a simple publisher?)

    def requestSensorSt(self):
        self.req_boolean.data = True

        result, self.rep_lowStates = self.node.request(
            "jump/low_state", self.req_boolean, Boolean, LowStates, 5000
        )

        ic("Result:", result)
        ic(self.rep_lowStates)

    def newRef(self):
        msg_test = LowCmd()
        msg_test.qr.data.append(1)
        if not self.newRef_pub.publish(msg_test):
            ic("--- New reference publish error ---")

    def reset(self):  # method to reset the model
        # first, ensure that yhe simulation is paused
        self.req_pause.pause = True
        result, self.rep_pause = self.node.request(
            "/world/default/control",
            self.req_pause,
            WorldControl,
            Boolean,
            1000,
        )
        # if pause, random joints position are created an request new pose
        if result and self.rep_pause.data:
            for index in range(len(self.req_JointPos.data)):
                self.req_JointPos.data[index] = random.uniform(
                    self.robot_model.joint_p_min[index],
                    self.robot_model.joint_p_max[index],
                )

            ic(self.req_JointPos.data)

            result, self.rep_JointPos = self.node.request(
                "jump/Control/reset_jpos", self.req_JointPos, Double_V, Boolean, 5000
            )
        else:
            ic("--- Reset error ---")

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

while 1:
    ag.newRef()
    time.sleep(1)
    ic("hey")
# ag.requestSensorSt()
# ag.action(11)
# ag.reset()
# ag.requestSensorSt()
