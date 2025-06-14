import numpy as np

class PID:
    def __init__(self, kp, ki, kd, target):
        """
        Initialize a variable-dimension PID controller.

        Args:
            kp (float or list): Proportional gain(s) per axis (or scalar).
            ki (float or list): Integral gain(s) per axis (or scalar).
            kd (float or list): Derivative gain(s) per axis (or scalar).
            target (tuple or array): Target position of any dimension.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.error = np.zeros_like(self.target)
    def reset(self, target=None):
        """
        Reset the internal state of the PID controller.

        Args:
            target (optional): New target to reset to.
        """

        if target is not None:
            self.target = target
            self.error = np.zeros_like(self.target)
        
    def get_error(self):    
        """
        Returns:
            float: Magnitude of the last error vector.
        """
        
        return self.error
    
    def update(self, current_pos, dt=0.1):
        """
        Compute the PID control signal.

        Args:
            current_pos (array-like): Current position (any dimension).
            dt (float): Time delta in seconds.

        Returns:
            np.ndarray: Control output vector.
        """
        new_error = -1*(current_pos-self.target)
        pidSignal = self.kp*(new_error) + self.kd*((new_error) - (self.error))/dt + self.ki*self.error*dt
        self.error = new_error
        return pidSignal

