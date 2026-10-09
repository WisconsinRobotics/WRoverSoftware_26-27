"""
Thread-safe storage for synchronized RGB and depth camera data.

This module provides a shared container for the latest RGB image, aligned
depth image, frame index, and effective RGB camera intrinsic matrix.

A threading lock ensures that camera frames and their associated frame index
are read and updated atomically. The intrinsic matrix is stored as a read-only
copy to prevent accidental modification.

Note:
    Thread safety applies to stored references, not the underlying NumPy
    arrays. Consumers must not modify retrieved frames in place.

    Data is shared only between threads within the same Python process.
"""

import threading
import numpy as np
from numpy.typing import NDArray


class CameraFrames:
    """
    Stores the latest synchronized camera data for thread-safe access.

    Provides methods for updating RGB and depth frames, storing the effective
    RGB intrinsic matrix, and retrieving the latest camera data.

    Attributes:
        color_frame:
            Latest undistorted RGB image in BGR format.
            Shape: (H, W, 3).

        depth_frame:
            Latest aligned depth image in millimeters.
            Shape: (H, W).

        rgb_camera_matrix:
            Effective RGB intrinsic matrix stored as a read-only array.
            Shape: (3, 3).

        frame_index:
            Index of the latest stored frame pair, initially -1.
            Incremented after each successful frame update.

        buffer_lock:
            Lock protecting access to the stored camera data.
    """

    def __init__(self):
        """
        Initializes an empty camera frame buffer and its synchronization lock.

        Frame data and the intrinsic matrix are initially None.
        The frame index is initialized to -1 to indicate that no
        frames have been received.
        """

        # 1) Store the latest camera data.
        self.color_frame = None
        self.depth_frame = None
        self.rgb_camera_matrix = None

        # 1a) Track frame updates.
        self.frame_index = -1

        # 1b) Protect the stored camera data.
        self.buffer_lock = threading.Lock()

    def write_frames(self, color_frame: NDArray, depth_frame: NDArray) -> int:
        """
        Atomically stores a synchronized RGB and depth frame pair.

        Updates both frame references under a single lock and increments
        the frame index. The input arrays are stored by reference rather
        than copied.

        Args:
            color_frame:
                Undistorted RGB image in BGR format.
                Expected shape: (H, W, 3).

            depth_frame:
                Depth image aligned to the RGB image, in millimeters.
                Expected shape: (H, W).

        Returns:
            int:
                Updated frame index corresponding to the stored frame pair.

        Note:
            Input arrays are not copied or validated. Callers must ensure
            that the frames are aligned and are not subsequently modified
            in place.
        """

        # 1) Atomically update the camera frames.
        with self.buffer_lock:
            self.color_frame = color_frame
            self.depth_frame = depth_frame

            # 1a) Increment the frame index.
            self.frame_index += 1

            return self.frame_index

    def write_camera_matrix(self, camera_matrix: NDArray):
        """
        Stores a read-only copy of the effective RGB intrinsic matrix.

        Converts the supplied matrix to a float64 NumPy array and disables
        writing before storing it under the synchronization lock.

        Args:
            camera_matrix:
                Effective intrinsic matrix of the undistorted RGB camera.
                Expected shape: (3, 3).

        Note:
            The matrix is copied before storage to prevent modifications
            to the original array from affecting the stored matrix.

            The matrix dimensions and calibration are not validated.
        """

        # 1) Store a read-only copy of the camera matrix.
        matrix = np.array(camera_matrix, dtype=np.float64, copy=True)
        matrix.setflags(write=False)

        with self.buffer_lock:
            self.rgb_camera_matrix = matrix

    def get_frames(self):
        """
        Retrieves the latest camera data in a single synchronized operation.

        Acquires the buffer lock to obtain a consistent snapshot of the
        stored frame references, frame index, and RGB intrinsic matrix.

        Returns:
            tuple:
                (color_frame, depth_frame, frame_index, rgb_camera_matrix)

                color_frame:
                    Latest RGB image as a NumPy array, or None.
                    Shape: (H, W, 3).

                depth_frame:
                    Latest aligned depth image in millimeters, or None.
                    Shape: (H, W).

                frame_index:
                    Integer index of the latest frame pair.
                    Returns -1 if no frames have been stored.

                rgb_camera_matrix:
                    Read-only effective RGB intrinsic matrix, or None.
                    Shape: (3, 3).

        Note:
            Returned arrays are not copied. The lock guarantees a
            consistent snapshot of references but does not protect the
            array contents after the method returns.
        """

        # 1) Retrieve the most recent camera data.
        with self.buffer_lock:
            return (
                self.color_frame,
                self.depth_frame,
                self.frame_index,
                self.rgb_camera_matrix,
            )
