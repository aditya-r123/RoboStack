"""
Quick manual test – just runs random actions so you can see the blocks spawn.
"""

import robosuite                                 # must come *before* env import
import envs                                      # noqa: F401  (triggers register_env)
import numpy as np

env = robosuite.make(
    "MultiColorBlockEnv",
    robots="Panda",
    has_renderer=True,
    has_offscreen_renderer=True,
    horizon=500,
    control_freq=20,
    # controller_configs=None  # ← omit to use robosuite's default OSC-Pose
    # n_blocks=8,             # pass extra kwargs straight through
)

obs = env.reset()
# ------------------------------------------------------------------ #
#  Random rollout: sample uniformly within [low, high] for each step #
# ------------------------------------------------------------------ #
low, high = env.action_spec          # each is an array of same length
for _ in range(500):
    action = np.random.uniform(low, high)
    obs, reward, done, info = env.step(action)
    env.render()
env.close()
