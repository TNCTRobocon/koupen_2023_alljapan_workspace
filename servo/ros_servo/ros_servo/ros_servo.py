import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

import time

import pigpio

class RosServo(Node):
    node_name = "ros_servo"
    
    config_sub_topic = "config"
    
    limit_pub_topic = "limit"

    SERVO_PIN_LIST = [12, 13]
    LIMIT_PIN_LIST = [16, 20, 19, 26, 6, 5, 8, 7]
    FREQ = 50
    LIMIT_TICK_TH = 10000
    
    def __init__(self):
        super().__init__(self.node_name)
        self.get_logger().info("Start init")
        
        self.config_sub = self.create_subscription(Int16MultiArray, self.config_sub_topic, self.config_sub_callback, 10)
        self.limit_pub = self.create_publisher(Int16MultiArray, self.limit_pub_topic, 10)
        self.pi = pigpio.pi()        
        
        self.get_pin_list = [0] * 8
        self.limit_tick_list = [0] * 8
        self.callback_ob = []
        self.config = [1] * 8
        
        self.old_tick = 0
        self.servo_status = 0
        self.servo_counter = 0
        
        for i in range(8):
            self.pi.set_mode(self.LIMIT_PIN_LIST[i], pigpio.INPUT)
            self.pi.set_pull_up_down(self.LIMIT_PIN_LIST[i], pigpio.PUD_UP)
            
        for pin in self.LIMIT_PIN_LIST:
            cb = self.pi.callback(pin, pigpio.EITHER_EDGE, self.pin_callback)
            self.callback_ob.append(cb)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def config_sub_callback(self, data):
        self.config = data.data
        
    def timer_callback(self):
        print(self.get_pin_list, self.config)
        # self.servo_check()
        tmp = Int16MultiArray(data=self.get_pin_list)
        self.limit_pub.publish(tmp)
    
    # def move_servo(self, deg):
    #     deg1 = int((deg * 9.5 / 180 + 2.5) * 10000)
    #     deg2 = int(((180 - deg) * 9.5 / 180 + 2.5) * 10000)
    #     self.pi.hardware_PWM(self.SERVO_PIN_LIST[0], self.FREQ, deg1)
    #     self.pi.hardware_PWM(self.SERVO_PIN_LIST[1], self.FREQ, deg2)
        
    # def set_servo(self):
    #     self.move_servo(180)
    #     print("set")
        
    # def reset_servo(self):
    #     self.move_servo(0)
    #     print("reset")
        
    # def servo_check(self):
    #     if self.config[0] == 3:
    #         self.reset_servo()
    #         return
    #     if self.get_pin_list[0] == 0 and self.get_pin_list[1] == 0:
    #         self.servo_counter = 0
    #         self.servo_status = 0
    #     elif self.get_pin_list[0] == 1 and self.get_pin_list[1] == 1:
    #         if self.servo_counter <= 5:
    #             self.set_servo()
    #             self.servo_status = 1
    #             self.servo_counter = 0
    #         else:
    #             self.servo_counter += 1
    #     elif self.get_pin_list[0] == 1 or self.get_pin_list[1] == 1:
    #         self.servo_counter += 1 
    #     else:
    #         return
    
    def pin_callback(self, gpio, level, tick):
        diff = abs(tick - self.limit_tick_list[self.LIMIT_PIN_LIST.index(gpio)])
        if diff > self.LIMIT_TICK_TH:
            print(gpio, level, tick)
            changed_limit = self.LIMIT_PIN_LIST.index(gpio)
            self.get_pin_list[changed_limit] = level

        self.limit_tick_list[self.LIMIT_PIN_LIST.index(gpio)] = tick 
        
def main(args=None):
    rclpy.init(args=args)
    rossevo = RosServo()
    try:
        rclpy.spin(rossevo)
    except KeyboardInterrupt:
        rossevo.destroy_node()
        
if __name__ == "__main__":
    main()
