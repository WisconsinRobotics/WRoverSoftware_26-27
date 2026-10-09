# =============================================================================
# Background:
#   The rover uses an OAK-D Wide camera to acquire RGB images and stereo depth
#   measurements for computer vision tasks, including gesture detection and
#   object localization.
#
#   This node runs a DepthAI V3 pipeline on a dedicated thread to continuously
#   acquire synchronized color and depth frames. Depth measurements are aligned
#   to the undistorted RGB image and represented in millimeters.
#
#   The latest frames, frame index, and RGB intrinsic matrix are stored in a
#   thread-safe CameraFrames object. This object is accessible to other modules
#   within the same Python process, but not to separate ROS 2 processes.
#
# =============================================================================
# Brief:
#   This ROS 2 node manages the OAK-D Wide camera acquisition pipeline:
#     1. Initializes the RGB and stereo cameras using DepthAI V3.
#     2. Configures RGB undistortion, stereo depth estimation, and depth
#        alignment to the RGB camera.
#     3. Synchronizes RGB and depth frames using a 20 ms threshold.
#     4. Continuously retrieves synchronized frame groups on a separate
#        thread and stores them in the shared CameraFrames object.
#     5. Retrieves and stores the effective RGB intrinsic matrix for
#        converting image coordinates into camera-relative 3D positions.
#     6. Publishes an Empty message whenever a new frame group is stored.
#     7. Handles camera acquisition errors and provides thread shutdown.
#
# =============================================================================
# Subscribes to:
#   None
#
# =============================================================================
# Publishes to:
#   '/camera_frames' (std_msgs/Empty)
#       Description:
#           Notifies subscribers whenever a new synchronized RGB and depth
#           frame group has been stored.
#       Format:
#           Empty message containing no frame data.
#       Frequency:
#           Up to approximately 30 Hz, depending on camera performance.
#
# =============================================================================
# Shared data:
#   camera_frames (CameraFrames)
#       Description:
#           Thread-safe storage for the latest synchronized camera data.
#           Accessible to modules within the same Python process.
#       Contains:
#           color_frame       - Undistorted RGB image (BGR NumPy array).
#           depth_frame       - Aligned depth image (millimeters).
#           frame_index       - Incrementing frame update counter.
#           rgb_camera_matrix - Effective RGB intrinsic matrix (3x3).
#
# =============================================================================
# Notes:
#   CameraFrames protects frame references using a threading lock. Consumers
#   should not modify the returned NumPy arrays in place.
#
#   The '/camera_frames' topic publishes notifications only. Image data is
#   not transmitted through ROS 2, so independently launched nodes cannot
#   access the stored frames without an additional communication mechanism.
#
#   The effective RGB intrinsic matrix and RGB/depth alignment must be
#   validated against the actual camera configuration.
#
# =============================================================================

__author__ = "Cameron Myhre"

import depthai as dai
import numpy as np
from datetime import timedelta
from typing import cast
import threading
from .camera_frames import CameraFrames
from std_msgs.msg import Empty
from rclpy.node import Node
import rclpy

# 0) Allow other modules to access the latest camera data.
# TODO: Verify that this is accessible to ROS2 nodes.
camera_frames: CameraFrames = CameraFrames()


class DepthCamera(Node):
    """
    ROS 2 node for acquiring synchronized RGB and depth camera data.

    Initializes and manages a DepthAI V3 pipeline for the OAK-D Wide camera.
    Camera acquisition runs on a dedicated thread, independently of the
    ROS 2 executor.

    The node stores synchronized RGB and depth frames in the shared
    CameraFrames object and publishes an Empty message whenever a new
    frame pair becomes available.

    Attributes:
        frame_update_publisher:
            ROS 2 publisher that notifies subscribers of new frame pairs.
            Topic: /camera_frames
            Message type: std_msgs/Empty

        stop_event:
            Threading event used to request camera acquisition shutdown.

        camera_thread:
            Background thread responsible for executing the DepthAI
            pipeline and retrieving synchronized camera frames.

    Note:
        Image data is shared through a module-level CameraFrames object
        and is therefore accessible only within the same Python process.

        The published Empty messages do not contain camera frame data.
    """

    def __init__(self):
        """
        Initializes the ROS 2 camera node and starts frame acquisition.

        Creates the frame-update publisher, initializes the shutdown
        event, and starts a daemon thread responsible for running
        the camera acquisition pipeline.
        """

        super().__init__("depth_camera")

        # Create a publisher to let
        self.frame_update_publisher = self.create_publisher(Empty, "camera_frames", 1)

        # Create a new thread for camera images.
        self.stop_event = threading.Event()
        self.camera_thread = threading.Thread(
            target=self.run_pipeline_subprocess, name="camera_acquisition", daemon=True
        )

        self.camera_thread.start()

    def run_pipeline_subprocess(self):
        """
        Executes the camera acquisition pipeline with exception handling.

        Calls run_pipeline() from the dedicated acquisition thread and
        catches exceptions raised during pipeline execution.

        If an exception occurs, logs an error and sets the shutdown
        event to indicate that camera acquisition has stopped.

        Note:
            Exceptions are handled within the acquisition thread and
            do not automatically terminate the ROS 2 node.
        """
        try:
            self.run_pipeline()
        except Exception:
            self.get_logger().error("Camera pipeline closed unexpectedly.")

            self.stop_event.set()

    def run_pipeline(self):
        """
        Initializes and continuously retrieves synchronized camera frames.

        Constructs a DepthAI V3 pipeline using the OAK-D Wide RGB and
        stereo cameras. The RGB output is undistorted, stereo depth is
        calculated in millimeters, and the depth output is aligned to
        the RGB camera's image coordinates.

        A Sync node groups RGB and depth messages using a 20 ms
        synchronization threshold. Each received frame pair is converted
        into NumPy arrays and stored in the shared CameraFrames object.

        The effective RGB intrinsic matrix is retrieved from the first
        received RGB frame and stored for downstream 3D deprojection.

        An Empty ROS 2 message is published after every successful
        frame-buffer update.

        Camera configuration:
            RGB resolution:
                1920 x 1080 pixels, BGR888p, 30 FPS.

            Stereo resolution:
                1280 x 800 pixels, 30 FPS.

            Stereo processing:
                Left-right consistency checking and subpixel estimation.

            Depth units:
                Millimeters.

            Synchronization threshold:
                20 milliseconds.

        Note:
            Frame acquisition continues until the pipeline stops or
            the stop_event is set.

            The intrinsic matrix is assumed to remain constant after
            initialization. Its consistency with the transformed RGB
            image must be verified.

            Camera frames are stored by reference rather than copied.
            Consumers must not modify the underlying arrays in place.
        """
        # 0) Update the global color / depth frames.
        global camera_frames

        # 1) Create the pipeline.
        with dai.Pipeline() as pipeline:
            # 2) Create and setup the color camera.
            rgb_camera = pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_A
            )
            rgb_camera.initialControl.setAutoWhiteBalanceMode(
                dai.CameraControl.AutoWhiteBalanceMode.AUTO
            )  # TEST

            # 2a) Setup the camera output
            rgb_output = rgb_camera.requestOutput(
                size=(1920, 1080),
                type=dai.ImgFrame.Type.BGR888p,
                resizeMode=dai.ImgResizeMode.CROP,
                enableUndistortion=True,
                fps=30,
            )

            # 3) Setup the stereo depth cameras.
            left_camera = pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_B
            )
            right_camera = pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_C
            )

            # 3a) Create the stereo depth processing node.
            stereo = pipeline.create(dai.node.StereoDepth)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(True)

            stereo.initialConfig.setDepthUnit(
                dai.StereoDepthConfig.AlgorithmControl.DepthUnit.MILLIMETER
            )

            # 3b) Configure both monochrome cameras.
            left_output = left_camera.requestOutput(size=(1280, 800), fps=30)
            right_output = right_camera.requestOutput(size=(1280, 800), fps=30)

            # 3c) Connect the cameras to the stereo processor.
            left_output.link(stereo.left)
            right_output.link(stereo.right)

            # 3d) Align the depth map to the RGB camera.
            rgb_output.link(stereo.inputAlignTo)

            # 4) Sync the depth and the color input.
            sync = pipeline.create(dai.node.Sync)
            sync.setSyncThreshold(timedelta(milliseconds=20))

            # 4a) Connect the RGB and depth outputs to the Sync node.
            rgb_output.link(sync.inputs["rgb"])
            stereo.depth.link(sync.inputs["depth"])

            # 4b) Create a queue for synchronized frames.
            sync_queue = sync.out.createOutputQueue(maxSize=4, blocking=False)

            # 5) Start the pipeline.
            pipeline.start()
            matrix_initialized = False

            # 6) Retrieve synchronized RGB and depth frames.
            while pipeline.isRunning() and not self.stop_event.is_set():
                # 7a) Retrieve the synchronized message group.
                message_group = sync_queue.tryGet()

                if message_group is None:
                    self.stop_event.wait(0.005)
                    continue

                message_group = cast(dai.MessageGroup, message_group)

                # 7b) Retrieve the RGB and depth messages.
                rgb_message = cast(dai.ImgFrame, message_group["rgb"])
                depth_message = cast(dai.ImgFrame, message_group["depth"])
                if rgb_message is None or depth_message is None:
                    continue

                # 7c) Convert both to NumPy arrays.
                color_frame = rgb_message.getCvFrame()
                depth_frame = depth_message.getFrame()

                # 7d) Initialize the intrinsic matrix when first available.
                if not matrix_initialized:
                    camera_matrix = np.asarray(
                        rgb_message.getTransformation().getIntrinsicMatrix(),
                        dtype=np.float64,
                    )

                    camera_frames.write_camera_matrix(camera_matrix)
                    matrix_initialized = True

                # 7e) Atomically update the stored frames.
                camera_frames.write_frames(color_frame, depth_frame)

                # 7f) Tell other ROS2 nodes that a new frame has been published.
                self.frame_update_publisher.publish(Empty())

    def stop_camera(self):
        """
        Requests camera acquisition shutdown and waits for the thread.

        Sets the stop_event to signal the acquisition loop to terminate,
        then joins the camera thread to wait for its completion.

        Note:
            This method blocks until the acquisition thread terminates.
            It should be called before destroying the ROS 2 node.
        """
        self.stop_event.set()
        self.camera_thread.join()


def main(args=None):
    """
    Initializes, runs, and shuts down the DepthCamera ROS 2 node.

    Initializes the ROS 2 context, constructs the DepthCamera node,
    and spins the executor until execution is interrupted or stopped.

    During shutdown, requests termination of the camera acquisition
    thread, destroys the node, and shuts down the ROS 2 context.

    Args:
        args:
            Optional command-line arguments passed to rclpy.init().
            Defaults to None.

    Note:
        Cleanup is performed through a finally block to ensure that
        the acquisition thread is stopped during normal shutdown or
        when an exception propagates from the executor.
    """
    rclpy.init(args=args)

    try:
        node = DepthCamera()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.stop_camera()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
