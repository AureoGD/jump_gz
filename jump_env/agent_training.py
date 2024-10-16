from stable_baselines3 import PPO
import os
from gz_harmonic_env import GzTrainning
import time

# tensorboard --logdir /home/agd/jump_gz/logs/


models_dir = f"models/{int(time.time())}/"
logdir = f"logs/{int(time.time())}/"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)

if not os.path.exists(logdir):
    os.makedirs(logdir)

env = GzTrainning()
env.reset()

model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=logdir)

TIMESTEPS = 100
iters = 0
while iters < 5000:
    iters += 1
    # print("Inter: ", iters)
    model.learn(
        total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name=f"PPO"
    )
    model.save(f"{models_dir}/{TIMESTEPS*iters}")
