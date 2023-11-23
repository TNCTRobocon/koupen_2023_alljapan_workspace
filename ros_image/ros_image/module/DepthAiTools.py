import numpy as np
import cv2
import depthai as dai

import json
from pathlib import Path


class DepthAiTools():
    CONFIG = "honrobo_main/src/ros_main/models/best.json"
    MODEL =  "honrobo_main/src/ros_main/models/best_openvino_2022.1_6shave.blob"
    CAMERA_PREV_DIM = (640, 640)
    
    def __init__(self):
        self.pipeline = self.create_camera_pipeline()
        self.device = dai.Device(self.pipeline)

    
    def create_camera_pipeline(self):
        # initialize a depthai pipeline
        pipeline = dai.Pipeline()
 
        print("[INFO] configuring source and outputs...")
        # define sources and outputs
        # since OAK's camera is used in this pipeline
        # a color camera node is defined
        camRgb = pipeline.create(dai.node.ColorCamera)
        # create a Yolo detection node
        # detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
        xoutRgb = pipeline.create(dai.node.XLinkOut)
        # create a XLinkOut node for getting the detection results to host
        # nnOut = pipeline.create(dai.node.XLinkOut)
        print("[INFO] setting stream names for queues...")
        # set stream names used in queue to fetch data when the pipeline is started
        xoutRgb.setStreamName("rgb")
        # nnOut.setStreamName("nn")

        print("[INFO] setting camera properties...")
        # setting camera properties like the output preview size,
        # camera resolution, color channel ordering and FPS
        
        # camRgb.setPreviewSize(self.CAMERA_PREV_DIM)
        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        # camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        # camRgb.setInterleaved(False)
        # camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setVideoSize(1270, 720)
        camRgb.setFps(60)
        
        xoutRgb.input.setBlocking(False)
        xoutRgb.input.setQueueSize(1)

        print("[INFO] creating links...")
        # linking the nodes - camera stream output is linked to detection node
        # RGB frame is passed through detection node linked with XLinkOut
        # used for annotating the frame with detection output
        # detection network node output is linked to XLinkOut input
        camRgb.video.link(xoutRgb.input)
        
        
        # camRgb.preview.link(detectionNetwork.input)
        # detectionNetwork.passthrough.link(xoutRgb.input)
        # detectionNetwork.out.link(nnOut.input)
        # return the pipeline to the calling function
        return pipeline
    
    def get_frame(self):
        self.video = self.device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
        videoIn = self.video.get()
        frame = videoIn.getCvFrame()
        return frame
    
# rec = DepthAiTools()
# with dai.Device(rec.pipeline) as device:
#     video = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
#     while True:
#         # frame, bbox = rec.recognition(device)
#         videoIn = video.get()
#         frame = videoIn.getCvFrame()
#         cv2.imshow("vid", frame)
#         # print(bbox)
#         if cv2.waitKey(1) == ord('q'):
#             break
#     cv2.destroyAllWindows()