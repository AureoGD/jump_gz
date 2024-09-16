#### imports ####
import sys
import os


current_path = os.path.dirname(os.path.abspath(__file__))

current_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_path + "/custom_msg/build/custom_msgs-msgs_genmsg/python")

from gz.custom_msgs.low_states_pb2 import low_states

msg = low_states()

print(msg.q.data)

msg.q.data.append(1)
print(msg.HasField("q"))
msg.q.data.set(0, 0)
print(msg.q.data)
