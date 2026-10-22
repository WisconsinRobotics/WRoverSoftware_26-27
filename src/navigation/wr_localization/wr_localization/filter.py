import math

import numpy as np
from numpy.typing import NDArray


EARTH_RADIUS = 6371000.0
ODOM_NOISE_STDDEV = 0.2  # m/s, TODO
GNSS_MIN_STDDEV = 0.1  # meters, TODO
INITIAL_VEL_VAR = 1.0  # (m/s)^2, TODO
ACCELERATION_NOISE = 0.25  # m^2/s^3, TODO
C_GNSS = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
C_ODOM = np.array([[0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
R_ODOM = np.eye(2) * ODOM_NOISE_STDDEV**2


def wrap_degrees(angle):
    """Binds the angle from [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


class PositionKalmanFilter:
    """
    Kalman filter ONLY for the GNSS and odometry observations.
    TODO: add imu reading


    x = state vector ([east, north, east_vel, north_vel])
    y = measurement vector
    P = state estimation error covariance
    A = state transition matrix
    C = observation matrix (C_GNSS for position, C_ODOM for velocity)
    Q = process noise covariance
    R = measurement noise covariance
    K = Kalman gain
    """

    def __init__(self, x0: NDArray | None = None, P0: NDArray | None = None):
        self.origin = None
        self.x = np.zeros(4) if x0 is None else x0
        self.P = (
            np.diag([0.0, 0.0, INITIAL_VEL_VAR, INITIAL_VEL_VAR]) if P0 is None else P0
        )

    @staticmethod
    def A(dt: float) -> NDArray:
        """Return A(dt)."""
        return np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    def predict(self, dt: float):
        """Prediction step, update uncertainty matrix and apply state transition matrix."""
        A = self.A(dt)
        # Continuous acceleration noise, so splitting a time interval into
        # more timer/sensor callbacks does not change the prediction.
        Q = ACCELERATION_NOISE * np.array(
            [
                [dt**3 / 3, 0.0, dt**2 / 2, 0.0],
                [0.0, dt**3 / 3, 0.0, dt**2 / 2],
                [dt**2 / 2, 0.0, dt, 0.0],
                [0.0, dt**2 / 2, 0.0, dt],
            ]
        )
        self.x = A @ self.x
        self.P = A @ self.P @ A.T + Q

    def correct(self, y: NDArray, C: NDArray, R: NDArray):
        """Correction step, update state with Kalman gain and lower uncertainty."""
        err = y - C @ self.x
        K = self.P @ C.T @ np.linalg.inv(C @ self.P @ C.T + R)
        self.x += K @ err
        # Using joseph form here...
        self.P = (np.eye(4) - K @ C) @ self.P @ (np.eye(4) - K @ C).T + K @ R @ K.T

    def correct_odometry(self, east_vel: float, north_vel: float):
        """Correction step for odometry only."""
        self.correct(
            np.array([east_vel, north_vel]),
            C_ODOM,
            R_ODOM,
        )

    def correct_gnss(self, lat: float, lon: float, acc_m: float):
        """Correction step for GNSS only."""
        R = np.eye(2) * max(acc_m, GNSS_MIN_STDDEV) ** 2
        if self.origin is None:
            self.origin = (lat, lon)
            self.x[:2] = 0.0
            self.P[:2, :] = 0.0
            self.P[:, :2] = 0.0
            self.P[:2, :2] = R
            return

        origin_lat, origin_lon = self.origin
        y = EARTH_RADIUS * np.array(
            [
                math.cos(math.radians(origin_lat))
                * math.radians(wrap_degrees(lon - origin_lon)),
                math.radians(lat - origin_lat),
            ]
        )

        self.correct(y, C_GNSS, R)

    def coordinates(self) -> tuple[float, float]:
        """Convert local east/north meters back to latitude/longitude."""
        if self.origin is None:
            raise ValueError("no origin")
        lat, lon = self.origin
        east, north = self.x[:2]
        return (
            lat + math.degrees(north / EARTH_RADIUS),
            wrap_degrees(
                lon + math.degrees(east / (EARTH_RADIUS * math.cos(math.radians(lat))))
            ),
        )
