"""
Utilities for estimating an object's 3D camera-relative position from
2D image coordinates and aligned depth data.

This module provides two primary operations:

    deproject_pixels_to_point_cloud:
        Converts a collection of 2D pixel coordinates (u, v) into a 3D
        camera-relative point cloud (x, y, z) using an aligned depth image
        and pinhole camera model.

    compute_geometric_median:
        Estimates the geometric median of a 3D point cloud using a
        numerically stabilized variant of Weiszfeld's algorithm. The
        geometric median is a robust location estimator with a breakdown
        point of 50%.

These functions are primarily intended to support object-position estimation
for the Detection subteam, but are designed to operate independently of the
specific detection pipeline.

Depth measurements are assumed to be in millimeters. Position estimates are
therefore also returned in millimeters unless the input depth data is converted
beforehand.

Note:
    The current depth-camera configuration is calibrated for measurements
    within approximately 3 meters. Estimates outside this range may therefore
    have reduced accuracy.
"""

__author__ = "Cameron Myhre"

import numpy as np
import warnings
from numpy.typing import NDArray

# ---------------------------------------------------------------------------
# Estimation
# ---------------------------------------------------------------------------

# Computational constants.
DISTANCE_EPSILON = 1e-12
TOLERANCE = 1e-6
MAX_ITERATIONS = 100

def compute_geometric_median(
    points: NDArray[np.float64]
) -> NDArray[np.float64]:
    """
    Computes the geometric median of the given point cloud using a
    numerically stabilized variant of Weiszfeld's algorithm.

    Time complexity is O(KN), where N is the number of points and K is
    the number of iterations required for convergence.

    Args:
        points: An array of shape (N, 3), where each row represents
                a point (x, y, z).

    Returns:
        The geometric median as an array (x, y, z).
    """

    # 0) Verify that the input isn't empty or malformed.
    if points is None:
        raise ValueError("Point cloud cannot be None.")

    if len(points) == 0:
        raise ValueError("Point cloud cannot be empty.")

    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(
            f"Expected point cloud with shape (N, 3), got {points.shape}."
        )

    # A single point is trivially its own geometric median.
    if len(points) == 1:
        return points[0].copy()

    # 1) Compute an initial estimate.
    #    The coordinate-wise median provides a robust starting point.
    previous_estimate: NDArray[np.float64] = np.median(points, axis=0)

    # 2) Iterate until convergence.
    for _ in range(MAX_ITERATIONS):

        # Find the distance from every point to our current estimate.
        distances = np.linalg.norm(
            points - previous_estimate,
            axis=1
        )

        # Points closer to the current estimate receive greater weight.
        weights = 1.0 / np.maximum(
            distances,
            DISTANCE_EPSILON
        )

        # Compute the next weighted estimate.
        next_estimate = np.average(
            points,
            axis=0,
            weights=weights
        )

        # Stop if the estimate has barely changed.
        if np.linalg.norm(next_estimate - previous_estimate) < TOLERANCE:
            return next_estimate

        previous_estimate = next_estimate

    # Reaching this point means MAX_ITERATIONS was exhausted.
    warnings.warn(
        "[object_pose_estimator]: Geometric median did not converge "
        f"within {MAX_ITERATIONS} iterations."
    )

    return previous_estimate

# ---------------------------------------------------------------------------
# Conversion
# ---------------------------------------------------------------------------

def deproject_pixels_to_point_cloud(
    points: NDArray[np.int32],
    depth_image: NDArray[np.uint16],
    camera_matrix: NDArray[np.float64]
) -> NDArray[np.float64]:
    """
    Computes a 3D point cloud from a collection of 2D pixel coordinates,
    given an aligned depth image and camera intrinsic matrix.

    Assumes:
        - Depth values are measured in millimeters.
        - The depth image is aligned with the image coordinate system
          represented by the camera matrix.
        - Pixel coordinates are integer (u, v) coordinates.

    Args:
        points:
            An array of shape (N, 2), where each row represents a pixel
            coordinate (u, v).

        depth_image:
            An array of shape (H, W), where each element represents the
            depth of that pixel in millimeters.

        camera_matrix:
            A (3, 3) camera intrinsic matrix.

    Returns:
        An array of shape (M, 3), where each row represents a valid
        3D point (x, y, z) in millimeters. M may be less than N if
        invalid depth measurements are discarded.
    """

    # Extract focal lengths and principal point.
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]

    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    # Extract pixel coordinates.
    u = points[:, 0]
    v = points[:, 1]

    # Look up the depth corresponding to every pixel.
    z = depth_image[v, u].astype(np.float64)

    # Remove pixels with invalid depth measurements.
    valid = z > 0

    u = u[valid]
    v = v[valid]
    z = z[valid]

    # Deproject pixel coordinates into camera-space coordinates.
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # Construct the (N, 3) point cloud.
    return np.column_stack((x, y, z))