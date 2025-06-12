"""
Quick manual test – just runs random actions so you can see the blocks spawn.
"""

import robosuite                                 # must come *before* env import
import envs                                      # noqa: F401  (triggers register_env)
from robosuite.models.objects import BoxObject

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
for _ in range(500):
    obs, reward, done, info = env.step(env.action_space.sample())
    env.render()
env.close()
