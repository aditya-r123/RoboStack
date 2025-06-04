import numpy as np

class PID:
    """
    A simple Proportional-Integral-Derivative (PID) controller.
    """
    def __init__(self, kp, ki, kd, target):
        """
        Initializes the PID controller.

        Args:
            kp (float or array-like): Proportional gain.
            ki (float or array-like): Integral gain.
            kd (float or array-like): Derivative gain.
            target (array-like): The target setpoint for the controller.
        """
        self.target = np.asarray(target, dtype=float).copy()
        # Broadcast gains to match target shape if they are scalar
        self.kp = np.broadcast_to(kp, self.target.shape).astype(float)
        self.ki = np.broadcast_to(ki, self.target.shape).astype(float)
        self.kd = np.broadcast_to(kd, self.target.shape).astype(float)
        
        self.integral = np.zeros_like(self.target)
        self.prev_error = np.zeros_like(self.target)
        self.error_norm = np.inf

    def reset(self, target=None):
        """
        Resets the PID controller's internal state.

        Args:
            target (array-like, optional): A new target setpoint. 
                                           If None, keeps the current target.
        """
        if target is not None:
            self.target = np.asarray(target, dtype=float).copy()
        self.integral.fill(0.0)
        self.prev_error.fill(0.0)
        self.error_norm = np.inf # Reset error norm

    def get_error(self):
        """
        Returns the current error norm.
        """
        return self.error_norm

    def update(self, current_pos, dt):
        """
        Updates the PID controller and returns the control output.

        Args:
            current_pos (array-like): The current position/value.
            dt (float): The time step (delta time) since the last update.

        Returns:
            np.ndarray: The control output.

        Raises:
            ValueError: If dt is not positive.
        """
        current_pos = np.asarray(current_pos, dtype=float)
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        
        error = self.target - current_pos
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )
        
        self.prev_error = error.copy() # Store a copy of the error
        self.error_norm = np.linalg.norm(error)
        
        return output
