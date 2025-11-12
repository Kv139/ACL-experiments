# ruff: noqa
# import scenic.core.distributions import
from controllers.pid import PID
import numpy as np


class LateralControl:
    def __init__(self, dt) -> None:
        self.pid_keeping = PID(K_P=0.2, K_D=0.2, K_I=0.02, dt=dt)
        self.pid_changing = PID(K_P=0.1, K_D=0.1, K_I=0.02, dt=dt)

        # State management
        self.target_lane = None
        self.mode = "keeping"  # Can be 'keeping' or 'changing'
        self.past_steer = 0.0

    def set_target(self, target_lane, mode):
        """
        Public method called from Scenic to update the controller's goal.
        """
        self.target_lane = target_lane
        self.mode = mode

    def _regulate_steering(self, steer_angle, max_rate=0.1):
        """
        Smooths the steering angle to prevent jerky movements.
        """
        steer_change = np.clip(steer_angle - self.past_steer, -max_rate, max_rate)
        return self.past_steer + steer_change

    def compute_control(self, car):
        """
        Computes the steering command based on the current state.
        """
        if not self.target_lane:
            return 0.0

        # Calculate Cross-Track Error (CTE) to the target lane's centerline
        target_centerline = self.target_lane.centerline
        cte = target_centerline.signedDistanceTo(car.position)

        # Select the appropriate PID controller based on the current mode
        if self.mode == "changing":
            current_steer = self.pid_changing.run_step(cte)  # Use negative CTE to steer towards the line
        else:  # 'keeping'
            current_steer = self.pid_keeping.run_step(cte)
        speed = abs(car.speed)
        if speed < 10:
            div = 1
        elif speed < 20:
            div = 2
        elif speed < 30:
            div = 4
        else:
            div = 8
        current_steer = current_steer / div

        # Regulate and store the steering angle
        self.past_steer = current_steer

        # Check if a lane change is complete
        if self.mode == "changing" and abs(cte) < 1:
            self.mode = "keeping"

        return np.clip(current_steer, -1.0, 1.0)

    def reset(self):
        """Resets the internal PID controllers to prevent stale states after a mode change."""
        # Assuming your PID class has a 'reset' method
        self.pid_keeping.reset()
        self.pid_changing.reset()