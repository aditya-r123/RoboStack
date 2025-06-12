"""
Where we’ll put higher-level logic: choosing which block to pick,
what target pose to place it at, colour detection, etc.
For now, only a dummy policy that does nothing.
"""

class IdlePolicy:
    def reset(self, env):
        pass

    def step(self, obs):
        # Return zeros -> the PID above will hold current joint positions.
        return env.robots[0].controller.control(np.zeros_like(
            env.robots[0].controller.control_limits[0]
        ))
