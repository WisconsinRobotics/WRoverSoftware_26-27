"""
Acquires and publishes synchronized RGB and depth data from an OAK-D Wide camera.

This module configures and executes a DepthAI V3 pipeline to retrieve
undistorted RGB images and stereo depth measurements aligned to the RGB
camera's image coordinates.

Synchronized frame pairs are published through the FrameSharer interface,
which stores the latest color frame, depth frame, and frame index in named
shared memory buffers. The effective RGB camera intrinsic matrix is also
published when the first valid frame pair is received.

This allows independent Python processes, including ROS 2 nodes, to access
camera information without transferring full-resolution images through
ROS 2 topics.

Note:
    Depth measurements are represented in millimeters.

    Shared memory access is synchronized using the 'rover_camera' lock.
    All consumers must acquire this lock when reading related buffers.

    The effective RGB intrinsic matrix is assumed to remain constant
    throughout pipeline execution. Its consistency with the transformed
    RGB output must be verified.
"""

__author__ = "Cameron Myhre"

import depthai as dai
import numpy as np
import time

from datetime import timedelta
from typing import cast
from .frame_sharer import FrameSharer


# 0) Initialize the shared memory publisher for camera data.
frame_sharer = FrameSharer()


def run_pipeline():
    """
    Initializes and continuously publishes synchronized camera frames.

    Constructs a DepthAI V3 pipeline using the OAK-D Wide RGB and stereo
    cameras. The RGB output is undistorted, while stereo depth is calculated
    in millimeters and aligned to the RGB camera's image coordinates.

    A Sync node groups RGB and depth messages using a 20 ms synchronization
    threshold. Each synchronized pair is converted into NumPy arrays and
    published to shared memory through the FrameSharer interface.

    The effective RGB intrinsic matrix is retrieved from the first
    synchronized RGB message and published separately to shared memory.

    Camera configuration:
        RGB resolution:
            1920 x 1080 pixels, BGR888p, 30 FPS.

        Stereo resolution:
            1280 x 800 pixels, 30 FPS.

        Stereo processing:
            Left-right consistency checking and extended disparity.

        Depth units:
            Millimeters.

        Synchronization threshold:
            20 milliseconds.

        Output queue:
            Maximum of four synchronized message groups.
            Non-blocking behavior permits older groups to be discarded.

    Note:
        Frame acquisition continues while the DepthAI pipeline is running.

        Frame buffers are allocated when the first synchronized frame pair
        is received. Their dimensions and data types are assumed to remain
        constant throughout execution.

        The intrinsic matrix is published only once, using metadata from
        the first received RGB frame.

        The shared memory publisher copies camera data into its buffers.
        Consumers must use the corresponding named lock to read consistent
        frame pairs and frame indices.
    """

    # 1) Create the DepthAI pipeline.
    with dai.Pipeline() as pipeline:
        # 2) Initialize and configure the RGB camera.
        rgb_camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

        # 2a) Enable automatic white balance.
        rgb_camera.initialControl.setAutoWhiteBalanceMode(
            dai.CameraControl.AutoWhiteBalanceMode.AUTO
        )  # TEST

        # 2b) Configure the RGB output resolution, format, and undistortion.
        rgb_output = rgb_camera.requestOutput(
            size=(1920, 1080),
            type=dai.ImgFrame.Type.BGR888p,
            resizeMode=dai.ImgResizeMode.CROP,
            enableUndistortion=True,
            fps=30,
        )

        # 3) Initialize the left and right stereo cameras.
        left_camera = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B
        )
        right_camera = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C
        )

        # 3a) Configure stereo depth estimation.
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)  # Improves close-range depth estimation.

        # 3b) Configure depth measurements in millimeters.
        stereo.initialConfig.setDepthUnit(
            dai.StereoDepthConfig.AlgorithmControl.DepthUnit.MILLIMETER
        )

        # 3c) Configure both stereo camera outputs.
        left_output = left_camera.requestOutput(size=(1280, 800), fps=30)
        right_output = right_camera.requestOutput(size=(1280, 800), fps=30)

        # 3d) Connect both stereo cameras to the depth processor.
        left_output.link(stereo.left)
        right_output.link(stereo.right)

        # 3e) Align the resulting depth measurements to the RGB output.
        rgb_output.link(stereo.inputAlignTo)

        # 4) Initialize synchronization between RGB and depth messages.
        sync = pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(timedelta(milliseconds=20))

        # 4a) Connect the RGB and depth outputs to the synchronization node.
        rgb_output.link(sync.inputs["rgb"])
        stereo.depth.link(sync.inputs["depth"])

        # 4b) Create a non-blocking queue for synchronized message groups.
        sync_queue = sync.out.createOutputQueue(maxSize=4, blocking=False)

        # 5) Start the DepthAI pipeline.
        pipeline.start()

        # 5a) Track whether the effective RGB intrinsic matrix is published.
        matrix_initialized = False

        # 6) Continuously retrieve synchronized RGB and depth data.
        while pipeline.isRunning():
            # 6a) Attempt to retrieve the next synchronized message group.
            message_group = sync_queue.tryGet()

            # Avoid busy-waiting when no synchronized frames are available.
            if message_group is None:
                time.sleep(0.005)
                continue

            message_group = cast(dai.MessageGroup, message_group)

            # 6b) Extract the RGB and depth messages.
            rgb_message = cast(dai.ImgFrame, message_group["rgb"])
            depth_message = cast(dai.ImgFrame, message_group["depth"])

            if rgb_message is None or depth_message is None:
                continue

            # 6c) Convert the synchronized messages into NumPy arrays.
            color_frame = rgb_message.getCvFrame()
            depth_frame = depth_message.getFrame()

            # 6d) Publish the effective RGB intrinsic matrix once.
            if not matrix_initialized:
                camera_matrix = np.asarray(
                    rgb_message.getTransformation().getIntrinsicMatrix(),
                    dtype=np.float64,
                )

                frame_sharer.update_color_matrix(camera_matrix)
                matrix_initialized = True

            # 6e) Publish the synchronized frame pair and update its index.
            frame_sharer.update_frames(color_frame, depth_frame)


if __name__ == "__main__":
    run_pipeline()
