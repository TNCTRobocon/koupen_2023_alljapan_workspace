import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Image
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import pyrealsense2

import time

from .module.RealsenseTools import Realsense
from .module.JoyCalcTools import JoyCalcTools
from .module.Recognition import Recog
from .module.Switch import *



class RosMain(Node):
    node_name = "ros_main"
    
    joy_linux_sub_topic = "joy"
    selected_point_sub_topic = "result_mouse_left"
    config_sub_topic = "config"
    
    joy_pub_topic_name = 'can_joy'
    btn_pub_topic_name = 'can_btn'
    data_pub_topic_name = 'can_data'
    img_pub_topic_name = 'result'
    
    CONTOROLLER_MODE = 1 # 0=Portable-PC 1=F310
    ARROW_LOST_FRAME = 10
    MAX_MOVE_AXES = 50
    MAX_MOVE_METER = 0.5
    USE_CAMERA = 1 # 0=Manual 1=Auto
    CONNECT_CAMERA = 0
    
    move_distance = 0
    
    rs = None
    
    def __init__(self):
        
        super().__init__(self.node_name)
        self.get_logger().info("Start init")
        
        self.ctx = pyrealsense2.context()
        self.ctx.set_devices_changed_callback(self.rs_changed)
        
        self.get_logger().info("Applying CTX settings")
        
        self.joy_sub = self.create_subscription(Joy, self.joy_linux_sub_topic, self.sub_joy_callback, 10)
        self.selected_point_sub = self.create_subscription(Point, self.selected_point_sub_topic, self.sub_point_callback, 10)
        self.config_sub = self.create_subscription(Int16MultiArray, self.config_sub_topic, self.sub_config_callback, 10)
        
        self.pub_joy = self.create_publisher(Int16MultiArray, self.joy_pub_topic_name, 10)
        self.pub_btn = self.create_publisher(Int16MultiArray, self.btn_pub_topic_name, 10)
        self.pub_data = self.create_publisher(Int16MultiArray, self.data_pub_topic_name, 10)
        self.pub_image = self.create_publisher(Image, self.img_pub_topic_name, 10)
        self.get_logger().info("Applying Node settings")
        
        self.t_switch = ToggleSwitch()
        self.joy_tool = JoyCalcTools(self.CONTOROLLER_MODE)
        self.recog = Recog()
        self.imsg = Image()
        self.bridge = CvBridge()
        self.get_logger().info("Generated Object")
        
        self.point = None
        self.config = [1, 1, 1, 1, 1, 1]
        self.move_side_distance = 0
        self.move_front_distance = 0
        self.button_data = [0] * 4

        if self.USE_CAMERA == 1:
            try:
                self.get_logger().info("Waiting Realsense for 10sec...")
                self.rs = Realsense()
                self.CONNECT_CAMERA = 1
            except:
                self.get_logger().info("No Realsense detected")
        else:
            self.get_logger().info("Realsense is not used")
                
        self.get_logger().info("Main process start")
        
    def rs_changed(self,inf):
        self.get_logger().info("Changed Realsense status")
        devs = inf.get_new_devices()

        if len(devs):
            self.get_logger().info("Realsense connected")
            self.rs = Realsense()
            self.CONNECT_CAMERA = 1
        else:
            self.CONNECT_CAMERA = 0
            self.get_logger().info("Realsense disconnected")
            self.get_logger().info("Please reconnect after 10sec")
            self.rs.stopper()
            
    def sub_point_callback(self, data):
        self.point = data
        
    
    def sub_config_callback(self, data):
        self.config = data.data
        
        
    def sub_joy_callback(self, data):
        camera_status = self.USE_CAMERA == 1 and self.CONNECT_CAMERA == 1
        if camera_status: 
            image, _, side_distance, front_distance= self.recognition()
            print(side_distance, front_distance)
        
        joy_data, copied_button, hat_msg_data = self.contoroller(data)
        
        if camera_status:
            joy_data = self.joy_tool.override_joy(joy_data, 0, side_distance)
            joy_data = self.joy_tool.override_joy(joy_data, 1, front_distance)
        
        to_int = lambda x: list(map(int, x))
        
        joy_data = to_int(joy_data)
        tmp_data_1 = Int16MultiArray(data=joy_data)
        self.pub_joy.publish(tmp_data_1)
        
        time.sleep(0.001)
        copied_button = to_int(copied_button)
        tmp_data_2 = Int16MultiArray(data=copied_button)
        self.pub_btn.publish(tmp_data_2)
        time.sleep(0.001)
        
        hat_msg_data = to_int(hat_msg_data)
        self.joy_tool.override_config(hat_msg_data, self.config)
        tmp_data_3 = Int16MultiArray(data=hat_msg_data)
        self.pub_data.publish(tmp_data_3)
        time.sleep(0.001)
        
        if camera_status:
            try:
                img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                self.pub_image.publish(img)
            except UnboundLocalError:
                pass

        
    def recognition(self):
        image, depth, result = self.rs.get_realsense_frame()
        bbox_np = self.recog.detect_fruits(image)
        origin_point, detected_list = self.recog.calc_point(image,bbox_np)

        image = self.recog.draw_frame_line(image, origin_point)

        if self.recog.detecting_check(bbox_np, self.config[5]) < self.ARROW_LOST_FRAME :
            if detected_list == None:
                return image, depth, self.move_side_distance, self.move_front_distance
            
            image = self.recog.draw_all_fruits_line(image,detected_list)
            
            if self.point == None:
                return image, depth, 0, 0
            
            detected_rect_point = self.recog.search_from_list(detected_list,self.point)
            
            if detected_rect_point == None:
                return image, depth, self.move_side_distance, self.move_front_distance
            
            if self.config[5] != 1:
                return image, depth, 0, 0
            
            image = self.recog.draw_to_fruits_line(image, origin_point, detected_rect_point)
            self.point.x = float(detected_rect_point.detected_centor_x)
            self.point.y = float(detected_rect_point.detected_centor_y)
            
            self.move_side_distance = self.recog.calc_side_movement(origin_point, detected_rect_point) * self.MAX_MOVE_AXES 
            
            self.fruits_distance = self.recog.calc_front_movement(detected_rect_point,result)
            self.move_front_distance = (self.fruits_distance / self.MAX_MOVE_METER) * self.MAX_MOVE_AXES 
            return image, depth, self.move_side_distance, self.move_front_distance
        else :
            print("lost")
            self.point = None
            self.move_side_distance = 0
            self.move_front_distance = 0
            return image, depth, self.move_side_distance, self.move_front_distance
    
    def contoroller(self, joy):
        # 1
        joy_data = [0] * 8
        joy_data = self.joy_tool.recaluculating_joy(joy)
        
        # 2
        self.toggle_button_data = self.joy_tool.deplicate_btn_data(joy)
        changed_switch = self.t_switch.judge_changed_button(self.toggle_button_data)
 
        if changed_switch < 4 and changed_switch != -1:
            self.button_data[0] = changed_switch 

        copied_button = self.joy_tool.copy_button(self.toggle_button_data, self.button_data)
        # 3
        else_msg_data = self.joy_tool.recaluculating_hat(joy)
        
        return joy_data, copied_button, else_msg_data
    
def main(args=None):
    rclpy.init(args=args)
    rosmain = RosMain()
    try:
        rclpy.spin(rosmain)
    except KeyboardInterrupt:
        rosmain.destroy_node()

if __name__ == '__main__':
    main()