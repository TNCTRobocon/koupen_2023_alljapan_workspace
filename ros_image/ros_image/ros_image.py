import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy,Image
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

import pyrealsense2
import depthai as dai

from .module.RealsenseTools import Realsense
from .module.Recognition import Recog
from .module.DepthAiTools import DepthAiTools

class RosImage(Node):
    node_name = "ros_image"
    selected_point_sub_topic = "result_mouse_left"
    config_sub_topic = "config"
    config2_sub_topic = "config2"
    
    override_joy_sub_topic_name = 'override_joy'
    img_pub_topic_name = 'result'
    
    CONTOROLLER_MODE = 1 # 0=Portable-PC 1=F310
    USE_CAMERA = 1 # 0=Manual 1=Auto
    DETECT_TYPE = 2 #0=Only DepthAI 1=Only Realsense 2=Use Both
    
    FRAME_RATE = 30
    ARROW_LOST_FRAME = 10
    MAX_MOVE_SIDE_AXES = 100
    MAX_MOVE_FRONT_AXES = 50
    METER_TH = 0.5
    
    use_which_cam = 0 #0=Depth 1=RS  #Switched by GUI
    rs_connected = 0
    depthai_connected = 0
    
    move_distance = 0
    
    def __init__(self):
        super().__init__(self.node_name)

        self.get_logger().info("Start init")
        
        if self.CONTOROLLER_MODE:
            self.get_logger().info("Using F310")
        else:
            self.get_logger().info("Using PotablePC controller")
        
        self.get_logger().info("Applying CTX settings")
        self.ctx = pyrealsense2.context()
        self.ctx.set_devices_changed_callback(self.rs_changed) 
        
        self.get_logger().info("Applying Node settings")
        self.selected_point_sub = self.create_subscription(Point, self.selected_point_sub_topic, self.sub_point_callback, 10)
        self.config_sub = self.create_subscription(Int16MultiArray, self.config_sub_topic, self.sub_config_callback, 10)
        self.config2_sub = self.create_subscription(Int16MultiArray, self.config2_sub_topic, self.sub_config2_callback, 10)
        
        self.pub_image = self.create_publisher(Image, self.img_pub_topic_name, 10)
        self.pub_override_joy = self.create_publisher(Int16MultiArray, self.override_joy_sub_topic_name, 10)
        
        timer_period = 1/self.FRAME_RATE
        
        self.timer = self.create_timer(timer_period, self.image_timer_callback)
        
        self.get_logger().info("Generating Object")
        self.recog = Recog()
        self.imsg = Image()
        self.joymsg = Joy()
        self.bridge = CvBridge()
        
        self.point = None
        self.config = [1] * 6
        self.command_side_value = 0
        self.command_front_value = 0
        
        if not self.USE_CAMERA:
            self.get_logger().info("Camera isn't used")
            self.get_logger().info("Starting main process")
            return
        
        if self.DETECT_TYPE == 1 or self.DETECT_TYPE == 2:
            try:
                self.get_logger().info("Waiting Realsense...")
                self.rs = Realsense()
                self.get_logger().info("Connected Realsense")
                self.rs_connected = 1
            except:
                self.get_logger().info("Realsense connect failed")
                self.rs_connected = 0
                
        if self.DETECT_TYPE == 0 or self.DETECT_TYPE == 2:
            try:
                self.depthai = DepthAiTools()
                self.device = dai.Device(self.depthai.pipeline)
                self.get_logger().info("Connected DepthAI")
                self.depthai_connected = 1
            except:
                self.get_logger().info("DepthAI connect failed")
                self.depthai_connected = 0
        
        self.get_logger().info("Starting main process")
    
    def rs_changed(self,inf):
        self.get_logger().info("Changed Realsense status")
        devs = inf.get_new_devices()
        if len(devs):
            self.get_logger().info("Realsense connected")
            self.rs = Realsense()
            self.rs_connected = 1
        else:
            self.rs_connected = 0
            self.get_logger().info("Realsense disconnected")
            self.get_logger().info("Please reconnect after 10sec")
            self.rs.stopper()
    
    def sub_point_callback(self, data):
        self.point = data
        
    def sub_config_callback(self, data):
        self.config = data.data
    
    def sub_config2_callback(self, data):
        self.use_which_cam = data.data[0]
        
    def image_timer_callback(self):
        camera_usage = (self.USE_CAMERA == 1 and self.rs_connected == 1 and self.use_which_cam == 1) or (self.USE_CAMERA == 1 and self.depthai_connected == 1 and self.use_which_cam == 0)
        if camera_usage:
            image, _, side_distance, front_distance= self.recognition()
            print(side_distance, front_distance)
            try:
                img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                self.pub_image.publish(img)
            except:
                pass
        else:
            side_distance = 0
            front_distance = 0
        self.publish_override_joy(int(side_distance), int(front_distance))
        
    def publish_override_joy(self, side_distance, front_distance):
        send_data_array = [side_distance, front_distance]
        send_data = Int16MultiArray(data=send_data_array)
        self.pub_override_joy.publish(send_data)
        
        
    def recognition(self):
        # Get imageframe and detected data
        if self.rs_connected == 1 and self.use_which_cam == 1:
            image, depth, result = self.rs.get_realsense_frame()
            bbox_np = self.recog.detect_fruits(image)
        elif self.depthai_connected == 1 and self.use_which_cam == 0:
            # processed in inside of camera
            image, bbox_np = self.depthai.recognition(self.device)
            depth = None
        else:
            return RuntimeError("Irrigal recognition option")
        
        # calc fruits point
        origin_point, detected_list = self.recog.calc_point(image,bbox_np)
        drew_frame_image = self.recog.draw_frame_line(image, origin_point)
        
        # check mode and number of fruits
        if self.recog.detecting_check(bbox_np, self.config[5]) < self.ARROW_LOST_FRAME :
            if detected_list == None:
                return drew_frame_image, depth, self.command_side_value, self.command_front_value
            
            drew_frame_fruits_image = self.recog.draw_all_fruits_line(drew_frame_image,detected_list)
            
            if self.point == None:
                return drew_frame_fruits_image, depth, 0, 0
            
            detected_rect_point = self.recog.search_pointed_fruits_from_list(detected_list,self.point)
            
            if detected_rect_point == None:
                return drew_frame_fruits_image, depth, self.command_side_value, self.command_front_value
            
            if self.config[5] != 1:
                return drew_frame_fruits_image, depth, 0, 0
            
            drew_all_info_image = self.recog.mark_pointed_fruits(drew_frame_fruits_image, origin_point, detected_rect_point)
            self.point.x = float(detected_rect_point.detected_centor_x)
            self.point.y = float(detected_rect_point.detected_centor_y)
            
            self.command_side_value = self.recog.calc_side_movement(origin_point, detected_rect_point) * self.MAX_MOVE_SIDE_AXES
            
            # if camera is DepthAI, dont use depth
            if self.use_which_cam == 1:
                self.fruits_distance = self.recog.calc_front_movement(detected_rect_point,result)
                self.command_front_value = (self.fruits_distance / self.METER_TH) * self.MAX_MOVE_FRONT_AXES
                
                return drew_all_info_image, depth, self.command_side_value, self.command_front_value
            return drew_all_info_image, depth, self.command_side_value, 0
        else :
            self.get_logger().info("No Fruits Detected")
            self.point = None
            self.command_side_value = 0
            self.command_front_value = 0
            return drew_frame_image, depth, self.command_side_value, self.command_front_value

def main(args=None):
    rclpy.init(args=args)
    rosimage = RosImage()
    try:
        rclpy.spin(rosimage)
    except KeyboardInterrupt:
        rosimage.destroy_node()
        
if __name__ == '__main__':
    main()
        
        
        