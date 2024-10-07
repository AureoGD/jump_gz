from stable_baselines3.common.env_checker import check_env
import gz_harmonic_env
from icecream import ic
import time

env = gz_harmonic_env.GzTrainning()


# check_env(env)

episodes = 5000
for episode in range(episodes):
    env.reset()
    time.sleep(0.01)
    # rand_ac = env.action_space.sample()
    # print("action: ", rand_ac)
    # obs, reward, done, info, _ = env.step(rand_ac)
    # print(done)
