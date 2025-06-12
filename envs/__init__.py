"""
Register the custom environment so robosuite.make("MultiColorBlock", …) works.
"""
from robosuite.environments.base import register_env
from .multi_color_block_env import MultiColorBlockEnv

register_env(MultiColorBlockEnv)
