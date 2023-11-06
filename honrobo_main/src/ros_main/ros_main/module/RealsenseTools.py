import pyrealsense2 as rs
import numpy as np
import cv2

class Realsense():
    def __init__(self):
        config = rs.config()
        config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 15)
        config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 15)
        
        self.pipeline = rs.pipeline()
        self.profile = self.pipeline.start(config)
        
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
        self.decimate = rs.decimation_filter()
        self.spatial = rs.spatial_filter()
        self.hole_filling = rs.hole_filling_filter()
        self.depth_to_disparity = rs.disparity_transform(True)
        self.disparity_to_depth = rs.disparity_transform(False)
        
    def stopper(self):
        self.pipeline.stop()
        
    def get_realsense_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        
        RGB_frame = aligned_frames.get_color_frame()
        RGB_image = np.asanyarray(RGB_frame.get_data())
        RGB_image_s = cv2.resize(RGB_image, (424, 240))
        
        depth_frame = aligned_frames.get_depth_frame()
        
        # filter_frame = self.decimate.process(depth_frame)
        # filter_frame = self.depth_to_disparity.process(filter_frame)
        # filter_frame = self.spatial.process(filter_frame)
        # filter_frame = self.disparity_to_depth.process(filter_frame)
        # filter_frame = self.hole_filling.process(filter_frame)
        filter_frame = self.hole_filling.process(depth_frame)
        
        result_frame = filter_frame.as_depth_frame()
        
        depth_image = np.asanyarray(result_frame.get_data()) 
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)
        depth_colormap_s = cv2.resize(depth_colormap, (424, 240))
        
        
        
        return RGB_image_s, depth_colormap_s, depth_frame