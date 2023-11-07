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
        self.pipeline = self.create_camera_pipeline(self.CONFIG, self.MODEL)

        
    def recognition(self,device):
        # output queues will be used to get the rgb frames
        # and nn data from the outputs defined above
        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

        frame = None
        
        # fetch the RGB frames and YOLO detections for the frame
        inRgb = qRgb.get()
        inDet = qDet.get()
        if inRgb is not None:
            # convert inRgb output to a format OpenCV library can work
            frame = inRgb.getCvFrame()
        if inDet is not None:
            # if inDet is not none, fetch all the detections for a frame
            detections = inDet.detections
        if frame is not None:
            # annotate frame with detection results
            bbox_np = self.annotateFrame(frame, detections)
            return frame, bbox_np
        return frame, None
    
    def load_config(self,config_path):
    # open the config file and load using json module
        with config_path.open() as f:
            config = json.load(f)
            return config
    
    def frameNorm(self,frame, bbox):
        # nn data, being the bounding box locations, are in <0..1> range
        # normalized them with frame width/height
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    def annotateFrame(self,frame, detections):
        # loops over all detections in a given frame
        # annotates the frame with model name, class label,
        # confidence score, and draw bounding box on the object
        
        color = (0, 0, 255)
        bbox_np = []
        for detection in detections:
            bbox = self.frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
            bbox_np.append(bbox)
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 1)
        bbox_np = np.array(bbox_np)
        print(bbox_np)
        return bbox_np
    
    def create_camera_pipeline(self,config_path, model_path):
        # initialize a depthai pipeline
        pipeline = dai.Pipeline()
        # load model config file and fetch nn_config parameters
        print("[INFO] loading model config...")
        configPath = Path(config_path)
        model_config = self.load_config(configPath)
        nnConfig = model_config.get("nn_config", {})
        print("[INFO] extracting metadata from model config...")
        # using nnConfig extract metadata like classes,
        # iou and confidence threshold, number of coordinates
        metadata = nnConfig.get("NN_specific_metadata", {})
        classes = metadata.get("classes", {})
        coordinates = metadata.get("coordinates", {})
        anchors = metadata.get("anchors", {})
        anchorMasks = metadata.get("anchor_masks", {})
        iouThreshold = metadata.get("iou_threshold", {})
        confidenceThreshold = metadata.get("confidence_threshold", {})
        # output of metadata - feel free to tweak the threshold parameters
        #{'classes': 5, 'coordinates': 4, 'anchors': [], 'anchor_masks': {},
        # 'iou_threshold': 0.5, 'confidence_threshold': 0.5}
        print("[INFO] configuring source and outputs...")
        # define sources and outputs
        # since OAK's camera is used in this pipeline
        # a color camera node is defined
        camRgb = pipeline.create(dai.node.ColorCamera)
        # create a Yolo detection node
        detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
        xoutRgb = pipeline.create(dai.node.XLinkOut)
        # create a XLinkOut node for getting the detection results to host
        nnOut = pipeline.create(dai.node.XLinkOut)
        print("[INFO] setting stream names for queues...")
        # set stream names used in queue to fetch data when the pipeline is started
        xoutRgb.setStreamName("rgb")
        nnOut.setStreamName("nn")

        print("[INFO] setting camera properties...")
        # setting camera properties like the output preview size,
        # camera resolution, color channel ordering and FPS
        camRgb.setPreviewSize(self.CAMERA_PREV_DIM)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(15)
        
        print("[INFO] setting YOLO network properties...")
        # network specific settings - parameters read from config file
        # confidence and iou threshold, classes, coordinates are set
        # most important the model .blob file is used to load weights
        detectionNetwork.setConfidenceThreshold(confidenceThreshold)
        detectionNetwork.setNumClasses(classes)
        detectionNetwork.setCoordinateSize(coordinates)
        detectionNetwork.setAnchors(anchors)
        detectionNetwork.setAnchorMasks(anchorMasks)
        detectionNetwork.setIouThreshold(iouThreshold)
        detectionNetwork.setBlobPath(model_path)
        detectionNetwork.setNumInferenceThreads(2)
        detectionNetwork.input.setBlocking(False)
        print("[INFO] creating links...")
        # linking the nodes - camera stream output is linked to detection node
        # RGB frame is passed through detection node linked with XLinkOut
        # used for annotating the frame with detection output
        # detection network node output is linked to XLinkOut input
        camRgb.preview.link(detectionNetwork.input)
        detectionNetwork.passthrough.link(xoutRgb.input)
        detectionNetwork.out.link(nnOut.input)
        # return the pipeline to the calling function
        return pipeline
    
# rec = DepthAiTools()
# with dai.Device(rec.pipeline) as device:
#     while True:
#         frame, bbox = rec.recognition(device)
#         cv2.imshow("vid", frame)
#         print(bbox)
#         if cv2.waitKey(1) == ord('q'):
#             break
#     cv2.destroyAllWindows()