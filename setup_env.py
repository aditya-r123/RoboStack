
import robosuite                                
import envs                           
import numpy as np
from controllers.policies import *


env = robosuite.make(
    "MultiColorBlockEnv",
    robots="Panda",
    has_renderer=True,
    has_offscreen_renderer=True,
    horizon=10000,  # Increased from 250 to 1000 steps per episode
    control_freq=20,
    controller_configs={
        "type": "OSC_POSE",
        # "input_max": 1,
        # "input_min": -1,
        # "output_max": [0.05, 0.05, 0.05, 0.5, 0.5, 0.5],
        # "output_min": [-0.05, -0.05, -0.05, -0.5, -0.5, -0.5],
        # "kp": 150,
        # "damping": 1,
        "impedance_mode": "fixed",
        # "kp_limits": [0, 300],
        # "damping_limits": [0, 10],
        "position_limits": None,
        "orientation_limits": None,
        "uncouple_pos_ori": True,
        "control_delta": True,
        "interpolation": None,
        # "ramp_ratio": 0.2,
    }
    # n_blocks=8,            
)

# Print controller information
# print("Controller type:", env.robots[0].controller.name)
# print("Controller attributes:", dir(env.robots[0].controller))
# print("Action space:", env.action_spec)

obs = env.reset()

print(obs.keys())
low, high = env.action_spec
policy = MultiColorStackPolicy(obs)         
for _ in range(100000):
    action = policy.get_action(obs)
    # print("Final action:", action)
    print()
    obs, reward, done, info = env.step(action)
    env.render()
env.close()
