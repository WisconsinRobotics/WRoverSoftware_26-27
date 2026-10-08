import depthai as dai
import numpy as np
from datetime import timedelta
from typing import cast

# 0) Provide variables to store the most recent frame.
# 0a) Keep track of constants.
RGB_CAMERA_MATRIX = None

# 0b) Allow other modules to access the latest camera data.
# TODO: Verify that this is acessible to ROS2 nodes.
frame_index = -1  # -1 indicated no frame has been received yet.
color_frame = None
depth_frame = None

# 1) Create the pipeline.
with dai.Pipeline() as pipeline:
    # 2) Create and setup the color camera.
    rgb_camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

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
    left_camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)

    right_camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

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

    # 5) Retrieve camera calibration. (TEMP--MAY BE REMOVED IF UNDISTORTION MATRIX WORKS)
    # device = pipeline.getDefaultDevice()
    # calibration = device.readCalibration()

    # RGB_CAMERA_MATRIX = np.asarray(
    #    calibration.getCameraIntrinsics(
    #        dai.CameraBoardSocket.CAM_A,
    #        1920,
    #        1080,
    #        keepAspectRatio=True
    #    ),
    #    dtype=np.float64
    # )

    # 6) Start the pipeline.
    pipeline.start()

    # 6) Retrieve synchronized RGB and depth frames.
    while pipeline.isRunning():
        # 7a) Retrieve the synchronized message group.
        message_group = cast(dai.MessageGroup, sync_queue.get())

        # 7b) Retrieve the RGB and depth messages.
        rgb_message = cast(dai.ImgFrame, message_group["rgb"])
        depth_message = cast(dai.ImgFrame, message_group["depth"])

        # 7c) Convert both to NumPy arrays.
        frame_index += 1
        color_frame = rgb_message.getCvFrame()
        depth_frame = depth_message.getFrame()

        # 7d) Get the distorted camera matrix
        # TODO: Verify this remains constant across all frames.
        if RGB_CAMERA_MATRIX is None:
            RGB_CAMERA_MATRIX = np.asarray(
                rgb_message.getTransformation().getIntrinsicMatrix(), dtype=np.float64
            )
