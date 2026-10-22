import math
from typing import TypeAlias

import numpy as np
from numpy.typing import NDArray
from geometry_msgs.msg import Pose2D, Point


FloatArray: TypeAlias = NDArray[np.float64]

EARTH_RADIUS = 6371000.0
GNSS_MIN_STDDEV = 0.1  # meters, in case h_acc is too low, TODO: tune
ODOM_VEL_STDDEV = 0.2  # m/s, TODO: tune
ODOM_OMEGA_STDDEV = 0.1  # rad/s, TODO: tune
IMU_YAW_STDDEV = math.radians(2.0)  # radians, TODO: tune
INITIAL_VEL_VAR = 1.0  # (m/s)^2, TODO: tune
INITIAL_OMEGA_VAR = 1.0  # (rad/s)^2, TODO: tune
ACC_NOISE = 0.25  # m^2/s^3, TODO: tune
ROT_ACC_NOISE = 0.1  # rad^2/s^3, TODO: tune
C_GNSS = np.eye(2, 6)
C_IMU = np.array([[0.0, 0.0, 0.0, 0.0, 1.0, 0.0]])
C_ODOM = np.array(
    [
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]
)
# sensor measurement errors are assumed independent.
# note that R_GNSS is calculated on the fly
R_ODOM = np.diag([ODOM_VEL_STDDEV**2, ODOM_VEL_STDDEV**2, ODOM_OMEGA_STDDEV**2])
R_IMU = np.array([[IMU_YAW_STDDEV**2]])


def normalize_angle(angle: float) -> float:
    """Wrap radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def wrap_degrees(angle: float) -> float:
    """Wrap degrees to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


class RoverKalmanFilter:
    """
    The greatest Kalman filter in the world!

    x = state vector ([east, north, east_vel, north_vel, yaw, omega])
    y = measurement vector
    P = uncertainty matrix
    A = state transition matrix
    C = observation matrix
    Q = process noise covariance
    R = measurement noise covariance
    K = Kalman gain
    """

    def __init__(self, x0: FloatArray | None = None, P0: FloatArray | None = None):
        self.origin = None
        self.x = np.zeros(6) if x0 is None else x0
        self.P = (
            np.diag(
                [
                    0.0,
                    0.0,
                    INITIAL_VEL_VAR,
                    INITIAL_VEL_VAR,
                    math.pi**2,
                    INITIAL_OMEGA_VAR,
                ]
            )
            if P0 is None
            else P0
        )
        if self.x.shape != (6,) or self.P.shape != (6, 6):
            raise ValueError("invalid initial state estimate or uncertainty matrix")
        self.x[4] = normalize_angle(self.x[4])

    @property
    def east(self) -> float:
        return float(self.x[0])

    @property
    def north(self) -> float:
        return float(self.x[1])

    @property
    def yaw(self) -> float:
        return float(self.x[4])

    @staticmethod
    def A(dt: float) -> FloatArray:
        """Return A(dt), the state transition matrix after dt seconds."""
        return np.array(
            [
                [1.0, 0.0, dt, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, dt, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0, dt],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ]
        )

    @staticmethod
    def Q(dt: float) -> FloatArray:
        """Return Q(dt), the process noise covariance matrix after dt seconds."""
        # TODO: understand this
        t2, t3 = dt**2 / 2.0, dt**3 / 3.0
        q, q_yaw = ACC_NOISE, ROT_ACC_NOISE
        return np.array(
            [
                [q * t3, 0.0, q * t2, 0.0, 0.0, 0.0],
                [0.0, q * t3, 0.0, q * t2, 0.0, 0.0],
                [q * t2, 0.0, q * dt, 0.0, 0.0, 0.0],
                [0.0, q * t2, 0.0, q * dt, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, q_yaw * t3, q_yaw * t2],
                [0.0, 0.0, 0.0, 0.0, q_yaw * t2, q_yaw * dt],
            ]
        )

    def predict(self, dt: float):
        """Predict state and update uncertainty using elapsed seconds."""
        A = self.A(dt)
        self.x = A @ self.x
        self.x[4] = normalize_angle(self.yaw)
        self.P = A @ self.P @ A.T + self.Q(dt)

    def correct(self, y: FloatArray, C: FloatArray, R: FloatArray):
        """Correction step using SOME observation vector and matrix, and covariance."""
        err = y - C @ self.x
        K = self.P @ C.T @ np.linalg.inv(C @ self.P @ C.T + R)
        self.x += K @ err
        self.x[4] = normalize_angle(self.yaw)
        I_KC = np.eye(6) - K @ C
        self.P = I_KC @ self.P @ I_KC.T + K @ R @ K.T

    def correct_imu(self, yaw: float):
        """Correction step for JUST the IMU."""
        y = np.array([self.yaw + normalize_angle(yaw - self.yaw)])
        self.correct(y, C_IMU, R_IMU)

    def correct_odometry(self, east_vel: float, north_vel: float, omega: float):
        """Correction step for JUST the odometry."""
        y = np.array([east_vel, north_vel, omega])
        self.correct(y, C_ODOM, R_ODOM)

    def correct_gnss(self, lat: float, lon: float, acc_m: float):
        """Correct local position using a geographic fix and accuracy in meters."""
        # Calculate covariance on the fly.
        R = np.eye(2) * max(acc_m, GNSS_MIN_STDDEV) ** 2
        if self.origin is None:
            # We work in this different coordinate system for now.
            # TODO: find a better way.
            self.origin = Point(x=lat, y=lon)
            self.x[:2] = 0.0
            # Reset the matrix to be something like:
            # [ R 0 ]
            # [ 0 U ]
            # (where U is unchanged)
            self.P[:2, :] = 0.0
            self.P[:, :2] = 0.0
            self.P[:2, :2] = R
            return

        y = EARTH_RADIUS * np.array(
            [
                math.cos(math.radians(self.origin.x))
                * math.radians(wrap_degrees(lon - self.origin.y)),
                math.radians(lat - self.origin.x),
            ]
        )
        self.correct(y, C_GNSS, R)

    def pose(self) -> Pose2D:
        """Return latitude, longitude, and yaw as a geographic pose."""
        if self.origin is None:
            raise ValueError("no origin")
        # This is kinda like inverse dpos_to_geo().
        return Pose2D(
            x=self.origin.x + math.degrees(self.north / EARTH_RADIUS),
            y=wrap_degrees(
                self.origin.y
                + math.degrees(
                    self.east / (EARTH_RADIUS * math.cos(math.radians(self.origin.x)))
                )
            ),
            theta=self.yaw,
        )
