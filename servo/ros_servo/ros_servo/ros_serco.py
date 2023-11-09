import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

import time

import wiringpi as pi

class RosServo(Node):
    node_name = "ros_servo"
    config_sub_topic = "config"
    
    config = None
    limit_detect_flag = 0
    
    
    def __init__(self):
        super().__init__(self.node_name)
        self.get_logger().info("Start init")

        self.config_sub = self.create_subscription(Int16MultiArray, self.config_sub_topic, self.callback, 10)
        pin = PinConfig(0, 0)
        self.raspi = RasPi(pin)
        self.servo_st = self.raspi.reset_servo()
        
    def callback(self,data):
        self.config = data.data
        
        if self.config[2] == 1:
            self.servo_st = self.raspi.reset_servo()
        elif self.config[2] == 2:
            if self.raspi.limSW_check() == 1:
                self.servo_st = self.raspi.set_servo()
        elif self.config[2] == 3:
            if self.servo_st == 1:
                self.servo_st = self.raspi.reset_servo()
        else:
            pass
    
        
        
class RasPi():
    RANGE = 1024
    DIVISOR = 375
    
    servo_status = 0
    
    def __init__(self,pin):
        self.LIMIT_PIN = pin.LIMIT_PIN
        self.SERVO_PIN = pin.SERVO_PIN
        
        pi.wiringPiSetupGpio()
        pi.pinMode(self.SERVO_PIN, 2)
        pi.pinMode(self.LIMIT_PIN, pi.INPUT)
        pi.pullUpDnControl(self.LIMIT_PIN,pi.PUD_DOWN)
        
        pi.pwmSetMode(pi.PWM_MODE_MS)
        pi.pwmSetRange(self.RANGE)
        pi.pwmSetClock(self.DIVISOR)
        
    def move_servo(self, deg):
        if not (deg <= 90) and (deg >= -90):
            return
        
        move_deg = int(( 4.75 * deg / 90 + 7.25) / 100 * 1024)
        pi.pwmWrite(self.SERVO_PIN, move_deg)
        
    def set_servo(self):
        self.move_servo(deg=45)
        self.servo_status = 1
        return self.servo_status
    
    def reset_servo(self):
        self.move_servo(deg=0)
        self.servo_status = 0
        return self.servo_status
        
    def limSW_check(self):
        return pi.digitalRead(self.LIMIT_PIN)
            
    
class PinConfig():
    def __init__(self,LIM,SERVO):
        self.LIMIT_PIN = LIM
        self.SERVO_PIN = SERVO
    