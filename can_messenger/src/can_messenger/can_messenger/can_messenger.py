import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

from .module.UsbCan import UsbCan 
from .module.UseMessage import UseMessage

class CanMessenger(Node):
    node_name = "can_messenger"
    joy_topic = "can_joy"
    btn_topic = "can_btn"
    data_topic = "can_data"
    
    def __init__(self) -> None:
        super().__init__(self.node_name)
        self.sub_joy = self.create_subscription(Int16MultiArray, self.joy_topic, self.joy_callback, 10)
        self.sub_btn = self.create_subscription(Int16MultiArray, self.btn_topic, self.btn_callback, 10)
        self.sub_data = self.create_subscription(Int16MultiArray, self.data_topic, self.data_callback, 10)
        
        self.can_joy = UseMessage(0x009)
        self.can_btn = UseMessage(0x00A)
        self.can_data = UseMessage(0x00B)
        
        self.Ucan = UsbCan(0) # 1:CAN OFF 0: CAN ON
        self.Ucan.open()
        self.get_logger().info("setup done")
        
    def joy_callback(self, data):
        txmsg = self.can_joy.update_message(data.data)
        self.Ucan.send(txmsg)
        print_text = "axis:{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3}\n"
        self.get_logger().info(print_text.format(*data.data))
        
    def btn_callback(self, data):
        txmsg = self.can_btn.update_message(data.data)
        self.Ucan.send(txmsg)
        print_text = "button:{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3}\n"
        self.get_logger().info(print_text.format(*data.data))
    
    def data_callback(self, data):
        txmsg = self.can_data.update_message(data.data)
        self.Ucan.send(txmsg)
        print_text = "data:{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3}\n"
        self.get_logger().info(print_text.format(*data.data))
        
    def __del__(self):
        self.Ucan.close()
    
def main(args=None):
    rclpy.init(args=args)
    can_messenger = CanMessenger()
    rclpy.spin(can_messenger)
    can_messenger.destroy_node()
        
if __name__ == '__main__':
    main()