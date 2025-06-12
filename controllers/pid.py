"""
Simple joint-space PID controller skeleton.

👉  For milestone 2 we will flesh this out; right now we only return zeros
    so the robot stays still while we inspect the environment.
"""

import numpy as np

class JointSpacePID:
    def __init__(self, kp=None, ki=None, kd=None, num_joints: int = 7):
        self.kp = np.array(kp) if kp is not None else np.full(num_joints,  50.0)
        self.ki = np.array(ki) if ki is not None else np.zeros(num_joints)
        self.kd = np.array(kd) if kd is not None else np.full(num_joints,  2.0)

        self.integral = np.zeros(num_joints)
        self.prev_err = np.zeros(num_joints)

    def reset(self):
        self.integral[:] = 0
        self.prev_err[:] = 0

    def __call__(self, q, q_des, dt):
        err = q_des - q
        self.integral += err * dt
        deriv = (err - self.prev_err) / max(dt, 1e-6)
        self.prev_err = err

        return self.kp * err + self.ki * self.integral + self.kd * deriv
