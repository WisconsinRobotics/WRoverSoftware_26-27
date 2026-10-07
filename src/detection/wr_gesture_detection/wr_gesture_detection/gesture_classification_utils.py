"""
Utilities for preparing YOLO pose-estimation keypoints for gesture classification.

This module converts raw human-pose detections into a consistent feature
representation suitable for input to a gesture-classification model. It
normalizes joint positions relative to the detected person's shoulder midpoint
and shoulder width, removes facial keypoints, and computes selected anatomical
angle features for the arms.

The resulting feature vector contains normalized joint coordinates, keypoint
confidence values, and sine/cosine representations of elbow and upper-arm
angles. Invalid or degenerate poses are rejected to prevent malformed values
from entering the classification pipeline.
"""

__author__ = "Cameron Myhre"

from ultralytics import YOLO
from ultralytics.engine.results import Keypoints
import numpy as np
from numpy.typing import NDArray
import warnings

# Load a model
model = YOLO("yolo26n-pose.pt")

# Predict with the model
results = model("img.jpg")


def normalize_figure(
    keypoints: Keypoints,
    person_index=0,
) -> NDArray | None:
    """
    Normalize a person's pose relative to their shoulders.

    The midpoint between the shoulders becomes (0, 0), and the
    distance between the shoulders becomes 1.

    Args:
        keypoints: Ultralytics Keypoints object.

    Returns:
        NumPy array with shape (3, 12):
            [normalized_x, normalized_y, confidence]

        Facial keypoints are omitted.
    """

    # 0) Verify that the keypoints aren't malformed.
    if keypoints is None or keypoints.conf is None:
        return None

    if len(keypoints.xy) <= person_index:
        return None

    # 1) Get the numpy representation of the joint coordinates and confidence.
    points_xy = keypoints.xy[person_index].cpu().numpy()
    confidence = keypoints.conf[person_index].cpu().numpy()

    # 2) Compute the shoulder midpoint position and shoulder distance.
    #    All points will be scaled to be measurable in shoulder lengths
    #    and positioned relative to the shoulder center.
    left_shoulder = points_xy[5]
    right_shoulder = points_xy[6]

    center_x = (left_shoulder[0] + right_shoulder[0]) / 2
    center_y = (left_shoulder[1] + right_shoulder[1]) / 2

    shoulder_distance = np.sqrt(
        (right_shoulder[0] - left_shoulder[0]) ** 2
        + (right_shoulder[1] - left_shoulder[1]) ** 2
    )

    # Shoulder distance of zero would lead to division by zero.
    # Further, no gesture requires the figure to face 90 degrees from the camera.
    # Thus it's safe to omit this frame.
    if shoulder_distance < 1e-10:
        return None

    # 3) Normalize all data points.
    normalized_x = (points_xy[:, 0] - center_x) / shoulder_distance

    normalized_y = (points_xy[:, 1] - center_y) / shoulder_distance

    normalized = np.array(
        [
            normalized_x,
            normalized_y,
            confidence,
        ]
    )

    # Remove YOLO facial keypoints 0-4.
    return normalized[:, 5:]


def compute_joint_angles(
    normalized_joints: NDArray,
) -> NDArray | None:
    """
    Computes the anatomical elbow angles and upper-arm orientations
    for the left and right arms.

    Each angle is represented using cosine and sine.

    Args:
        normalized_joints:
            NumPy array containing normalized joint data.

    Returns:
        NumPy array containing:
            [right_elbow_cos, right_elbow_sin,
             left_elbow_cos, left_elbow_sin,
             right_upper_cos, right_upper_sin,
             left_upper_cos, left_upper_sin]

        Shape: (8,)

        Returns None if a required joint vector has near-zero magnitude.
    """

    # 1) Compute cos(theta) and sin(theta) for the right elbow.
    # 1a) Vectorize the right arm joints.
    right_elbow_to_shoulder = np.array(
        [
            normalized_joints[0, 1] - normalized_joints[0, 3],
            normalized_joints[1, 1] - normalized_joints[1, 3],
        ]
    )

    right_elbow_to_wrist = np.array(
        [
            normalized_joints[0, 5] - normalized_joints[0, 3],
            normalized_joints[1, 5] - normalized_joints[1, 3],
        ]
    )

    # 1b) Compute the magnitudes of the joint vectors.
    right_shoulder_mag = np.linalg.norm(right_elbow_to_shoulder)
    right_wrist_mag = np.linalg.norm(right_elbow_to_wrist)

    # We don't want to train NaN in our data.
    if right_shoulder_mag < 1e-10 or right_wrist_mag < 1e-10:
        return None

    # 1c) Use vector properties to compute cos(theta) and sin(theta).
    # We represent the angle using cosine and sine so angles near -pi and pi
    # are not interpreted as being far apart.
    right_elbow_cos = np.dot(right_elbow_to_shoulder, right_elbow_to_wrist) / (
        right_shoulder_mag * right_wrist_mag
    )

    # Use the signed 2D cross product to preserve the sign of sin(theta).
    right_elbow_sin = (
        right_elbow_to_shoulder[0] * right_elbow_to_wrist[1]
        - right_elbow_to_shoulder[1] * right_elbow_to_wrist[0]
    ) / (right_shoulder_mag * right_wrist_mag)

    # 2) Compute cos(theta) and sin(theta) for the left elbow.
    # 2a) Vectorize the left arm joints.
    left_elbow_to_shoulder = np.array(
        [
            normalized_joints[0, 0] - normalized_joints[0, 2],
            normalized_joints[1, 0] - normalized_joints[1, 2],
        ]
    )

    left_elbow_to_wrist = np.array(
        [
            normalized_joints[0, 4] - normalized_joints[0, 2],
            normalized_joints[1, 4] - normalized_joints[1, 2],
        ]
    )

    # 2b) Compute the magnitudes of the joint vectors.
    left_shoulder_mag = np.linalg.norm(left_elbow_to_shoulder)
    left_wrist_mag = np.linalg.norm(left_elbow_to_wrist)

    # We don't want to train NaN in our data.
    if left_shoulder_mag < 1e-10 or left_wrist_mag < 1e-10:
        return None

    # 2c) Use vector properties to compute cos(theta) and sin(theta).
    left_elbow_cos = np.dot(left_elbow_to_shoulder, left_elbow_to_wrist) / (
        left_shoulder_mag * left_wrist_mag
    )

    # Use the signed 2D cross product to preserve the sign of sin(theta).
    left_elbow_sin = (
        left_elbow_to_shoulder[0] * left_elbow_to_wrist[1]
        - left_elbow_to_shoulder[1] * left_elbow_to_wrist[0]
    ) / (left_shoulder_mag * left_wrist_mag)

    # 3) Compute cos(theta) and sin(theta) for the right upper arm.
    # 3a) Vectorize the right shoulder-to-elbow segment.
    right_upper_arm = np.array(
        [
            normalized_joints[0, 3] - normalized_joints[0, 1],
            normalized_joints[1, 3] - normalized_joints[1, 1],
        ]
    )

    # 3b) Compute the magnitude of the upper-arm vector.
    right_upper_mag = np.linalg.norm(right_upper_arm)

    # We don't want to train NaN in our data.
    if right_upper_mag < 1e-10:
        return None

    # 3c) Compute cos(theta) and sin(theta) relative to the positive x-axis.
    right_upper_cos = right_upper_arm[0] / right_upper_mag
    right_upper_sin = right_upper_arm[1] / right_upper_mag

    # 4) Compute cos(theta) and sin(theta) for the left upper arm.
    # 4a) Vectorize the left shoulder-to-elbow segment.
    left_upper_arm = np.array(
        [
            normalized_joints[0, 2] - normalized_joints[0, 0],
            normalized_joints[1, 2] - normalized_joints[1, 0],
        ]
    )

    # 4b) Compute the magnitude of the upper-arm vector.
    left_upper_mag = np.linalg.norm(left_upper_arm)

    # We don't want to train NaN in our data.
    if left_upper_mag < 1e-10:
        return None

    # 4c) Compute cos(theta) and sin(theta) relative to the positive x-axis.
    left_upper_cos = left_upper_arm[0] / left_upper_mag
    left_upper_sin = left_upper_arm[1] / left_upper_mag

    # 5) Return all angle features.
    return np.array(
        [
            right_elbow_cos,
            right_elbow_sin,
            left_elbow_cos,
            left_elbow_sin,
            right_upper_cos,
            right_upper_sin,
            left_upper_cos,
            left_upper_sin,
        ]
    )


def process_keypoints_for_classification(
    keypoints: Keypoints,
) -> NDArray | None:
    """
    Converts YOLO pose-estimation keypoints into features for gesture classification.

    Joint coordinates are normalized relative to the person's shoulder midpoint
    and shoulder width. Facial keypoints are removed, and selected arm angles
    are appended as sine/cosine pairs.

    Args:
        keypoints:
            Ultralytics Keypoints object containing the detected pose.

    Returns:
        NumPy array with shape (44,) containing:
            - 12 normalized non-facial joint x-coordinates
            - 12 normalized non-facial joint y-coordinates
            - 12 joint confidence values
            - 8 anatomical angle features

        Returns None if normalization or angle computation fails.
    """

    # 1) Normalize the keypoints.
    normalized_joints = normalize_figure(keypoints)
    if normalized_joints is None:
        warnings.warn("[gesture_classification_utils]: Failed to normalize keypoints.")
        return None

    # 2) Compute the anatomical angles for the normalized joints.
    joint_angles = compute_joint_angles(normalized_joints)
    if joint_angles is None:
        warnings.warn(
            "[gesture_classification_utils]: Failed to compute anatomical joint angles for keypoints."
        )
        return None

    # 3) Flatten and combine joint and angle features into one feature vector.
    return np.concatenate(
        (
            normalized_joints.ravel(),
            joint_angles,
        )
    )
