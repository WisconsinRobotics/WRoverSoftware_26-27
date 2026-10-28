import math
from enum import IntEnum
from typing import TypeAlias

import numpy as np
from numpy.typing import NDArray
from geometry_msgs.msg import Pose2D, Point
from wr_nav.navigation import wrap_degrees, normalize_angle


FloatArray: TypeAlias = NDArray[np.float64]


class State(IntEnum):
    """Indices of the state vector."""

    EAST = 0
    NORTH = 1
    EAST_VEL = 2
    NORTH_VEL = 3
    YAW = 4
    OMEGA = 5


def C(*states: State) -> FloatArray:
    """
    Returns an observation matrix of JUST the required states.

    Note that this doesn't work for any observation matrices that can't be expressed as rows of I.
    """
    return np.eye(len(State))[list(states)]


EARTH_RADIUS = 6371000.0

# Tunable constants
## Standard deviation of observations
GNSS_MIN_STDDEV = 0.1  # meters, in case h_acc is too low, TODO: tune
ODOM_VEL_STDDEV = 0.2  # m/s, TODO: tune
ODOM_OMEGA_STDDEV = 0.1  # rad/s, TODO: tune
IMU_YAW_STDDEV = math.radians(2.0)  # radians, TODO: tune

## Initial uncertainties
INITIAL_VEL_VAR = 1.0  # (m/s)^2, TODO: tune
INITIAL_OMEGA_VAR = 1.0  # (rad/s)^2, TODO: tune

## Process noise
ACC_NOISE = 0.25  # m^2/s^3, TODO: tune (though this is hard to tune, a guess is okay.)
ROT_ACC_NOISE = (
    0.1  # rad^2/s^3, TODO: tune (though this is hard to tune, a guess is okay.)
)

## Input constants and noise
# (tau here refers to the fact that these are both time constants)
TAU_VEL = 0.5  # seconds to reach ~63.2% of input velocity, TODO: tune
TAU_ROT_VEL = 0.5  # seconds to reach ~63.2% of input angular velocity, TODO: tune
INPUT_VEL_STDDEV = 1.0  # m/s, uncertainty in input velocity, TODO: tune
INPUT_OMEGA_STDDEV = 1.0  # rad/s, TODO: tune

# Matrices relevant to the Kalman filter.
GNSS_STATES = (State.EAST, State.NORTH)
ODOM_STATES = (State.EAST_VEL, State.NORTH_VEL, State.OMEGA)
IMU_STATES = (State.YAW,)
ANGLE_STATES = (State.YAW,)
# sensor measurement errors are assumed independent.
# note that R_GNSS is calculated on the fly
R_ODOM = np.diag([ODOM_VEL_STDDEV**2, ODOM_VEL_STDDEV**2, ODOM_OMEGA_STDDEV**2])
R_IMU = np.array([[IMU_YAW_STDDEV**2]])
U = np.diag([INPUT_VEL_STDDEV**2, INPUT_VEL_STDDEV**2, INPUT_OMEGA_STDDEV**2])


class RoverKalmanFilter:
    """
    The greatest Kalman filter in the world!

    x = state vector ([east, north, east_vel, north_vel, yaw, omega])
    y = measurement vector
    u = input vector
    P = uncertainty matrix
    A = state transition matrix
    B = input matrix
    C = observation matrix
    Q = process noise covariance
    R = measurement noise covariance
    U = input noise covariance
    K = Kalman gain
    """

    def __init__(self, x0: FloatArray | None = None, P0: FloatArray | None = None):
        self.origin = None
        self.x = np.zeros(len(State)) if x0 is None else x0
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
        if self.x.shape != (len(State),) or self.P.shape != (len(State), len(State)):
            raise ValueError("invalid initial state estimate or uncertainty matrix")
        self.wrap_angles()

    @property
    def east(self) -> float:
        """Current east coordinate estimate."""
        return float(self.x[State.EAST])

    @property
    def north(self) -> float:
        """Current north coordinate estimate."""
        return float(self.x[State.NORTH])

    @property
    def yaw(self) -> float:
        """Current yaw estimate."""
        return float(self.x[State.YAW])

    @property
    def omega(self) -> float:
        """Current yaw rate estimate."""
        return float(self.x[State.OMEGA])

    @staticmethod
    def A(dt: float) -> FloatArray:
        """Return A(dt), the state transition matrix after dt seconds."""
        # This matrix is basically just I but with dt at (0, 2), (1, 3), and (4, 5).
        # Those correspond to change in east, change in north, and change in yaw respectively.
        A = np.eye(len(State))
        A[GNSS_STATES + IMU_STATES, ODOM_STATES] = dt
        return A

    @staticmethod
    def B(dt: float) -> FloatArray:
        """Returns B(t), the input matrix after dt seconds."""
        # We model the rover as exponentially decaying towards the desired velocity.
        # Essentially, we assume that the rover covers ~63.2% of its velocity error per time constant.
        vel_gain = -math.expm1(-dt / TAU_VEL)
        omega_gain = -math.expm1(-dt / TAU_ROT_VEL)
        # TODO: what is this
        pos_gain = dt - TAU_VEL * vel_gain
        yaw_gain = dt - TAU_ROT_VEL * omega_gain
        return np.array(
            [
                [pos_gain, 0.0, 0.0],
                [0.0, pos_gain, 0.0],
                [vel_gain, 0.0, 0.0],
                [0.0, vel_gain, 0.0],
                [0.0, 0.0, yaw_gain],
                [0.0, 0.0, omega_gain],
            ]
        )

    @staticmethod
    def Q(dt: float) -> FloatArray:
        """Return Q(dt), the process noise covariance matrix after dt seconds."""
        t2, t3 = dt**2 / 2.0, dt**3 / 3.0
        q, q_rot = ACC_NOISE, ROT_ACC_NOISE
        return np.array(
            [
                [q * t3, 0.0, q * t2, 0.0, 0.0, 0.0],
                [0.0, q * t3, 0.0, q * t2, 0.0, 0.0],
                [q * t2, 0.0, q * dt, 0.0, 0.0, 0.0],
                [0.0, q * t2, 0.0, q * dt, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, q_rot * t3, q_rot * t2],
                [0.0, 0.0, 0.0, 0.0, q_rot * t2, q_rot * dt],
            ]
        )

    def predict(self, dt: float, u: FloatArray | None = None):
        """Predict state and update uncertainty using elapsed seconds."""
        A = self.A(dt)
        B = self.B(dt)
        Q = self.Q(dt)
        # Technically, the rover has to decay to zero velocity too.
        A = A - B @ C(*ODOM_STATES)
        u = np.zeros(B.shape[1]) if u is None else u
        self.x = A @ self.x + B @ u
        self.wrap_angles()
        Q = Q + B @ U @ B.T
        self.P = A @ self.P @ A.T + Q

    def correct(
        self,
        y: FloatArray,
        C: FloatArray,
        R: FloatArray,
        angle_rows: tuple[int, ...] = tuple(),
    ):
        """Correction step using SOME observation vector and matrix, and covariance."""
        res = y - C @ self.x
        # If the state vector is wrapped over the [-pi, pi] boundary,
        # then the error becomes drastically big. Thus, we wrap it here.
        for row in angle_rows:
            res[row] = normalize_angle(res[row])
        # K = PC^T(CPC^T+R)^-1
        # => (CPC^T+R)(K^T)=CP
        K = np.linalg.solve(C @ self.P @ C.T + R, C @ self.P).T
        self.x += K @ res
        self.wrap_angles()
        I_KC = np.eye(len(State)) - K @ C
        # Joseph form here makes this a bit more complicated.
        self.P = I_KC @ self.P @ I_KC.T + K @ R @ K.T

    def correct_imu(self, yaw: float):
        """Correction step for JUST the IMU."""
        y = np.array([yaw])
        self.correct(y, C(*IMU_STATES), R_IMU, angle_rows=(0,))

    def correct_odometry(self, east_vel: float, north_vel: float, omega: float):
        """Correction step for JUST the odometry."""
        y = np.array([east_vel, north_vel, omega])
        self.correct(y, C(*ODOM_STATES), R_ODOM)

    def correct_gnss(self, lat: float, lon: float, acc_m: float):
        """Correction step for JUST the GNSS."""
        # Calculate covariance on the fly.
        R = np.eye(2) * max(acc_m, GNSS_MIN_STDDEV) ** 2
        if self.origin is None:
            # We work in this different coordinate system because meters are much more convenient.
            self.origin = Point(x=lat, y=lon)
            rel = list(GNSS_STATES)
            self.x[rel] = 0.0
            # Reset the matrix to be something like:
            # [ R 0 ]
            # [ 0 U ]
            # (where U is unchanged)
            self.P[rel, :] = 0.0
            self.P[:, rel] = 0.0
            self.P[np.ix_(rel, rel)] = R
            return

        y = EARTH_RADIUS * np.array(
            [
                math.cos(math.radians(self.origin.x))
                * math.radians(wrap_degrees(lon - self.origin.y)),
                math.radians(lat - self.origin.x),
            ]
        )
        self.correct(y, C(*GNSS_STATES), R)

    def wrap_angles(self):
        """Wraps all of our angle states from [-pi, pi]."""
        for state in ANGLE_STATES:
            self.x[state] = normalize_angle(self.x[state])

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
