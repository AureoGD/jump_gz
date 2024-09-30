import sys
import os
import random
import time
from icecream import ic
import numpy as np

from threading import Lock


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

# export GZ_DESCRIPTOR_PATH=/home/agd/jump_gz/jump_cmsgs/build


class AgenteInterface:
    def __init__(self):
        ic.enable()
        self.robot_model = jump_model.JumpModel()

        self.step_length = self.robot_model.step_length
        self.N_ACTIONS = self.robot_model.n_actions
        self.N_OBSERVATIONS = self.robot_model.nn_states_input_dim
        self.obsLowValue = self.robot_model.obs_min
        self.obsHighValue = self.robot_model.obs_max

        self.ep = 0

        self.node = Node()

        self.newRef_pub = self.node.advertise("JumpRobot/Control/lowCmd", LowCmd)

        # variables for request low states service
        self.req_boolean = Boolean()
        self.rep_lowStates = LowStates()

        # variables for request low control service
        self.req_action = Int32()
        self.rep_lowCmd = LowCmd()

        # variables for set new position
        self.req_pause = WorldControl()
        self.rep_pause = Boolean()

        # variables to perform one step
        self.req_world_control = WorldControl()
        self.rep_world_control = Boolean()
        self.world_control_timeout = 1000
        self.req_world_control.multi_step = 1  # model.step_size
        self.req_world_control.step = True
        self.req_world_control.pause = True

        self.req_JointPos = Double_V()
        self.rep_JointPos = Boolean()
        self.req_JointPos.data.extend(
            np.zeros((1, len(self.robot_model.joint_name))).tolist()[0]
        )

        self.wainting_rgc = False

        self.rgc_lowcmd = LowCmd()
        self.node.subscribe(LowCmd, "/JumpRobot/RGC/lowcmd", self.rgc_cb)
        self.valid_rep = False
        self.mutex = Lock()

    def action(self, action):  # call the service for new action (requeste new qr)
        self.req_action.data = action
        self.robot_model.actionList(action)
        self.rep_bool = Boolean()

        result, self.rep_bool = self.node.request(
            "JumpRobot/RGC/reqsol", self.req_action, Int32, Boolean, 5000
        )

        if self.rep_bool.data and result:
            self.wainting_rgc = True
            while self.wainting_rgc:
                pass

            print(self.rep_lowCmd)
            if not self.newRef_pub.publish(self.rep_lowCmd):
                ic("--- New reference publish error ---")

    # method to "reset" the model. This method do not realy reset the simulation, but set the robot to a new position
    def reset(self):
        # first, ensure that yhe simulation is paused
        self.req_pause.pause = True
        result, self.rep_pause = self.node.request(
            "/world/default/control",
            self.req_pause,
            WorldControl,
            Boolean,
            1000,
        )

        # if pause, random values for joints position are created
        if result and self.rep_pause.data:
            q = self.robot_model.randonJointPos()

            for index in range(len(q)):
                self.req_JointPos.data[index] = q[index, 0]

            result, self.rep_JointPos = self.node.request(
                "/JumpRobot/Control/setJointPos",
                self.req_JointPos,
                Double_V,
                Boolean,
                5000,
            )
        else:
            ic("--- Reset error ---")

        # run some steps to update the sensors data
        result, self.rep_world_control = self.node.request(
            "/world/default/control",
            self.req_world_control,
            WorldControl,
            Boolean,
            self.world_control_timeout,
        )

        time.sleep(0.003)
        self.robot_model.resetVars()

        return self.observation()

    def observation(self):
        self.req_boolean.data = True

        result, self.rep_lowStates = self.node.request(
            "/JumpRobot/States/lowState", self.req_boolean, Boolean, LowStates, 5000
        )

        # update robot variables
        self.robot_model.robotStates(self.rep_lowStates)

        return self.robot_model.observation()

    def reward(self):
        return self.robot_model.reward()

    def done(self):
        return self.robot_model.done()

    def incrementStep(self):
        self.robot_model.self.steps += 1

    def rgc_cb(self, msg: LowCmd):
        with self.mutex:
            if msg.valid:
                self.valid_rep = True
            else:
                self.valid_rep = False
            self.rep_lowCmd = msg
            self.wainting_rgc = False


ag = AgenteInterface()
time.sleep(1)
while 1:
    ag.action(1)
    time.sleep(0.01)
