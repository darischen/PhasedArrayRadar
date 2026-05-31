"""
Extended Kalman Filter for target tracking (2D position + velocity).
"""

import numpy as np


class ExtendedKalmanFilter:
    """EKF for constant-velocity 2D motion model."""

    def __init__(self, dt=0.1):
        """
        Initialize EKF.

        Args:
            dt (float): Time step between measurements (seconds)
        """
        self.dt = dt

        # State: [x, y, vx, vy] (position and velocity)
        self.state = np.array([0.0, 0.0, 0.0, 0.0])

        # State transition matrix (constant velocity)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=float)

        # Process noise covariance
        self.Q = np.eye(4) * 0.5
        self.Q[2, 2] = 0.3  # Speed noise
        self.Q[3, 3] = 0.3

        # Measurement noise covariance (range, angle)
        self.R = np.array([
            [1.0, 0.0],
            [0.0, np.radians(5) ** 2]
        ])

        # State covariance
        self.P = np.eye(4) * 10.0

    def predict(self):
        """
        Predict step: x_pred = F * x
        """
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, range_m, angle_deg):
        """
        Update step with measurement (range, angle).

        Args:
            range_m (float): Range estimate in meters
            angle_deg (float): Angle estimate in degrees
        """
        # Measurement vector
        z = np.array([range_m, np.radians(angle_deg)])

        # Measurement function: h(x) = [x, arctan2(y, x)]
        x, y = self.state[0], self.state[1]
        h = np.array([
            np.sqrt(x ** 2 + y ** 2),
            np.arctan2(y, x)
        ])

        # Jacobian of h
        r = np.sqrt(x ** 2 + y ** 2) + 1e-10
        H = np.array([
            [x / r, y / r, 0, 0],
            [-y / r ** 2, x / r ** 2, 0, 0]
        ])

        # Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Innovation
        y_err = z - h
        # Wrap angle error to [-π, π]
        y_err[1] = np.arctan2(np.sin(y_err[1]), np.cos(y_err[1]))

        # Update state and covariance
        self.state += K @ y_err
        self.P = (np.eye(4) - K @ H) @ self.P

    def get_state(self):
        """Return current state [x, y, vx, vy]."""
        return self.state.copy()

    def reset(self):
        """Reset filter to initial state."""
        self.state = np.array([0.0, 0.0, 0.0, 0.0])
        self.P = np.eye(4) * 10.0
