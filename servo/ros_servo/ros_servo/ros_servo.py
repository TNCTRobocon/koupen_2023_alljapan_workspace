import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

import time

import pigpio

class RosServo(Node):
    node_name = "ros_gui"
    
    limit_pub_topic = "limit"

    SERVO_PIN_LIST = [12, 13]
    LIMIT_PIN_LIST = [2, 3, 4, 17, 27, 22, 23, 24]
    FREQ = 50
    
    def __init__(self):
        super().__init__(self.node_name)
        self.get_logger().info("Start init")
        
        self.limit_pub = self.create_publisher(Int16MultiArray, self.limit_pub_topic, 10)
        self.pi = pigpio.pi()        
        
        self.get_pin_list = [0] * 8
        self.limit_tick_list = [0] * 8
        self.callback_ob = []
        
        for i in range(8):
            self.pi.set_mode(self.LIMIT_PIN_LIST[i], pigpio.INPUT)
            self.pi.set_pull_up_down(self.LIMIT_PIN_LIST[i], pigpio.PUD_UP)
            
        for pin in self.LIMIT_PIN_LIST:
            cb = self.pi.callback(pin, pigpio.EITHER_EDGE, self.pin_callback)
            self.callback_ob.append(cb)

        self.old_tick = 0
        self.servo_status = 0
        self.servo_counter = 0

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        self.servo_check()
        print(self.get_pin_list)
    
    def move_servo(self, deg):
        deg1 = int((deg * 9.5 / 180 + 2.5) * 10000)
        deg2 = int(((180 - deg) * 9.5 / 180 + 2.5) * 10000)
        self.pi.hardware_PWM(self.SERVO_PIN_LIST[0], self.FREQ, deg1)
        self.pi.hardware_PWM(self.SERVO_PIN_LIST[1], self.FREQ, deg2)
        
    def set_servo(self):
        self.move_servo(180)
        
    def reset_servo(self):
        self.move_servo(0)
        
    def servo_check(self):
        if self.get_pin_list[0] == 0 and self.get_pin_list[1] == 0:
            self.reset_servo()
            self.servo_counter = 0
            self.servo_status = 0
        elif self.get_pin_list[0] == 1 and self.get_pin_list[1] == 1:
            if self.servo_counter <= 5:
                self.set_servo()
                self.servo_status = 1
                self.servo_counter = 0
            else:
                self.servo_counter += 1
        elif self.get_pin_list[0] == 1 or self.get_pin_list[1] == 1:
            self.servo_counter += 1 
        else:
            return
    
    def pin_callback(self, gpio, level, tick):
        diff = abs(tick - self.limit_tick_list[self.LIMIT_PIN_LIST.index(gpio)])
        if diff > 20000:
            print(gpio, level, tick)
            changed_limit = self.LIMIT_PIN_LIST.index(gpio)
            self.get_pin_list[changed_limit] = level

        self.limit_tick_list[self.LIMIT_PIN_LIST.index(gpio)] = tick 