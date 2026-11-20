"""
Provides shared memory storage for synchronized DepthAI camera data.

This module implements a publisher for sharing RGB frames, aligned depth
frames, frame indices, and RGB camera intrinsics between Python processes.

Camera data is stored in named shared memory buffers, allowing independent
processes to access the latest information without transferring entire
frames through ROS 2 messages.

A named inter-process lock ensures that updates to related buffers occur
atomically. All consumers MUST acquire the same lock when accessing shared
data to prevent partially updated frames or mismatched frame indices.

Shared memory buffers are created on their first respective update and
retain fixed sizes for subsequent updates.

Note:
    The standard lock name for DepthAI camera data is 'rover_camera'.

    Shared memory names:
        color_frame:
            Latest RGB camera image in BGR format.

        depth_frame:
            Latest stereo depth image aligned to RGB coordinates.

        frame_count:
            Index identifying the most recently published frame pair.

        color_matrix:
            Effective RGB camera intrinsic matrix.

    Consumers must know the dimensions and data types of shared arrays
    to correctly interpret their contents.

    Shared memory updates are synchronized only between processes
    that explicitly use the corresponding named lock.
"""

import numpy as np
from numpy.typing import NDArray
from multiprocessing import shared_memory

from wr_commons.shared_lock import SharedLock


class FrameSharer:
    """
    Manages shared memory buffers for DepthAI camera information.

    Provides methods for publishing synchronized color and depth frames,
    tracking frame updates, and sharing the RGB intrinsic matrix.

    Frame buffers are initialized on the first update and subsequently
    overwritten with the latest available data. A shared inter-process
    lock protects updates to ensure consistency between cooperating
    readers and writers.

    Attributes:
        lock:
            Named inter-process lock used to synchronize buffer access.

        shm_color_matrix:
            Shared memory containing the RGB camera intrinsic matrix.

        shm_color:
            Shared memory containing the latest color image.

        shm_depth:
            Shared memory containing the latest aligned depth image.

        shm_count:
            Shared memory containing the latest frame index.

        frame_count:
            Internal frame counter, initially -1.

        sharing_frames:
            Indicates whether the frame buffers have been initialized.
    """

    def __init__(self):

        # 1) Initialize the named lock for synchronized shared memory access.
        self.lock = SharedLock("rover_camera")

        # 2) Initialize references to the shared memory buffers.
        self.shm_color_matrix: shared_memory.SharedMemory | None = None
        self.shm_color: shared_memory.SharedMemory | None = None
        self.shm_depth: shared_memory.SharedMemory | None = None
        self.shm_count: shared_memory.SharedMemory | None = None

        # 3) Initialize the frame counter.
        self.frame_count = -1  # The first published frame has index zero.

        # 4) Track whether the shared frame buffers have been created.
        self.sharing_frames = False

    def update_frames(self, color_frame: NDArray, depth_frame: NDArray):
        """
        Publishes a synchronized color and depth frame pair to shared memory.

        Creates the shared color, depth, and frame index buffers on the first
        invocation. Subsequent calls overwrite the existing buffers with
        the latest frames.

        Both frames and their associated index are updated while holding
        the named inter-process lock to prevent cooperating consumers from
        reading partially updated data.

        Args:
            color_frame:
                Undistorted RGB camera image in BGR format.
                Shape: (H, W, 3).

            depth_frame:
                Stereo depth image aligned to RGB image coordinates.
                Shape: (H, W). Depth measurements are in millimeters.

        Note:
            The allocated buffers retain their initial sizes. Subsequent
            frames must have compatible dimensions and data types.

            The frame index is stored as a four-byte, big-endian integer.
            Its initial published value is zero.
        """

        # 1) Create the shared memory buffers on the first frame update.
        if self.shm_color is None or self.shm_depth is None or self.shm_count is None:
            # 1a) Allocate buffers for color, depth, and frame index data.
            self.shm_color = shared_memory.SharedMemory(
                create=True, size=color_frame.nbytes, name="color_frame"
            )
            self.shm_depth = shared_memory.SharedMemory(
                create=True, size=depth_frame.nbytes, name="depth_frame"
            )
            self.shm_count = shared_memory.SharedMemory(
                create=True, size=4, name="frame_count"
            )

            # 1b) Mark the frame buffers as initialized.
            self.sharing_frames = True

        # 2) Atomically publish the synchronized frame pair.
        with self.lock:
            # 2a) Increment the frame index.
            self.frame_count += 1

            # 2b) Create NumPy views over the shared memory buffers.
            new_color = np.ndarray(
                color_frame.shape, dtype=color_frame.dtype, buffer=self.shm_color.buf
            )
            new_depth = np.ndarray(
                depth_frame.shape, dtype=depth_frame.dtype, buffer=self.shm_depth.buf
            )

            # 2c) Copy the latest frames into shared memory.
            new_color[:] = color_frame[:]
            new_depth[:] = depth_frame[:]

            # 2d) Publish the updated frame index.
            if self.shm_count.buf is None:  # Pylance bug
                return

            self.shm_count.buf[:4] = self.frame_count.to_bytes(4, byteorder="big")

    def update_color_matrix(self, color_matrix: NDArray):
        """
        Publishes the effective RGB camera intrinsic matrix to shared memory.

        Creates the matrix buffer on the first invocation and overwrites
        its contents on subsequent calls.

        Matrix updates are protected by the named inter-process lock.

        Args:
            color_matrix:
                Effective intrinsic matrix for the undistorted RGB image.
                Shape: (3, 3).

        Note:
            The buffer size is determined by the first supplied matrix.
            Consumers must use a compatible shape and data type when
            interpreting the shared matrix.
        """

        # 1) Create the shared intrinsic matrix buffer if necessary.
        if self.shm_color_matrix is None:
            self.shm_color_matrix = shared_memory.SharedMemory(
                create=True, size=color_matrix.nbytes, name="color_matrix"
            )

        # 2) Update the matrix while holding the shared lock.
        with self.lock:
            # 2a) Create a NumPy view over the shared matrix buffer.
            new_matrix = np.ndarray(
                color_matrix.shape,
                dtype=color_matrix.dtype,
                buffer=self.shm_color_matrix.buf,
            )

            # 2b) Copy the updated intrinsic matrix into shared memory.
            new_matrix[:] = color_matrix[:]

    def unlink(self):
        """
        Closes this instance's shared memory handles.

        Releases the local references to the shared memory buffers without
        removing the named shared memory objects from the operating system.

        Note:
            Despite its name, this method does not call SharedMemory.unlink().
            The underlying shared memory objects may remain available to
            other processes after these handles are closed.
        """

        # 1) Close each initialized shared memory handle.
        for shm in (
            self.shm_count,
            self.shm_color,
            self.shm_depth,
            self.shm_color_matrix,
        ):
            if shm is not None:
                shm.close()
