from stable_baselines3.common.env_checker import check_env
import gz_harmonic_env
from icecream import ic
import time
import agent_interface
# env = gz_harmonic_env.GzTrainning()

env = agent_interface.AgenteInterface()
# check_env(env)

episodes = 10000
for episode in range(episodes):
    ic(episode)
    ic(env.observation())
    time.sleep(0.1)
    ic(env.reset2())
    time.sleep(0.2)
    ic("----")
    # rand_ac = env.action_space.sample()
    # print("action: ", rand_ac)
    # obs, reward, done, info, _ = env.step(rand_ac)
    # print(done)
