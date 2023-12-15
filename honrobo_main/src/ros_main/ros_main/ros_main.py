import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray

import time

from .module.JoyCalcTools import JoyCalcTools
from .module.Switch import *

class RosMain(Node):
    node_name = "ros_main"
    
    joy_linux_sub_topic = "joy"
    override_joy_sub_topic = "override_joy"
    config_sub_topic = "config"
    config2_sub_topic = "config2"
    
    joy_pub_topic_name = 'can_joy'
    btn_pub_topic_name = 'can_btn'
    data_pub_topic_name = 'can_data'
    
    CONTOROLLER_MODE = 1 # 0=Portable-PC 1=F310
    
    def __init__(self):
        super().__init__(self.node_name)
        self.get_logger().info("Start init")

        self.joy_sub = self.create_subscription(Joy, self.joy_linux_sub_topic, self.sub_joy_callback, 10)
        self.override_joy_sub = self.create_subscription(Int16MultiArray, self.override_joy_sub_topic, self.sub_override_joy_callback, 10)
        self.config_sub = self.create_subscription(Int16MultiArray, self.config_sub_topic, self.sub_config_callback, 10)
        self.config2_sub = self.create_subscription(Int16MultiArray, self.config2_sub_topic, self.sub_config2_callback, 10)
        
        self.pub_joy = self.create_publisher(Int16MultiArray, self.joy_pub_topic_name, 10)
        self.pub_btn = self.create_publisher(Int16MultiArray, self.btn_pub_topic_name, 10)
        self.pub_data = self.create_publisher(Int16MultiArray, self.data_pub_topic_name, 10)
        self.get_logger().info("Applying Node settings")
        
        self.t_switch = ToggleSwitch()
        self.joy_tool = JoyCalcTools(self.CONTOROLLER_MODE)
        self.get_logger().info("Generated Object")
        
        self.config = [1, 1, 1, 1, 1, 1]
        self.button_data = [0] * 4
        
        self.side_distance = 0
        self.front_distance = 0
   
        self.get_logger().info("Main process start")

    def sub_point_callback(self, data):
        self.point = data
        
    def sub_override_joy_callback(self, data):
        self.side_distance = data.data[0]
        self.front_distance = data.data[1]
    
    def sub_config_callback(self, data):
        self.config = data.data
        
    def sub_config2_callback(self, data):
        self.USE_WHICH = data.data[0]
        
        
    def sub_joy_callback(self, data):
        
        joy_data, copied_button, hat_msg_data = self.contoroller(data)
        
        joy_data = self.joy_tool.override_joy(joy_data, 0, self.side_distance)
        joy_data = self.joy_tool.override_joy(joy_data, 1, self.front_distance)
        
        to_int = lambda x: list(map(int, x))
        
        joy_data = to_int(joy_data)
        tmp_data_1 = Int16MultiArray(data=joy_data)
        self.pub_joy.publish(tmp_data_1)
        
        sleep_time = 0.000001
        # sleep_time = 0.03
        
        time.sleep(sleep_time)
        copied_button = to_int(copied_button)
        tmp_data_2 = Int16MultiArray(data=copied_button)
        self.pub_btn.publish(tmp_data_2)
        time.sleep(sleep_time)
        
        hat_msg_data = to_int(hat_msg_data)
        self.joy_tool.override_config(hat_msg_data, self.config)
        tmp_data_3 = Int16MultiArray(data=hat_msg_data)
        self.pub_data.publish(tmp_data_3)
        time.sleep(sleep_time)
        
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