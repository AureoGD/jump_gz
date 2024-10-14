from stable_baselines3.common.env_checker import check_env
import gz_harmonic_env
from icecream import ic
import time

env = gz_harmonic_env.GzTrainning()


# check_env(env)

episodes = 10000
for episode in range(episodes):
    ic("begin")
    env.reset()
    ic("end", episode)
    time.sleep(0.5)
    # rand_ac = env.action_space.sample()
    # print("action: ", rand_ac)
    # obs, reward, done, info, _ = env.step(rand_ac)
    # print(done)
