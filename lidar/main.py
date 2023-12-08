import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import time
import datetime

import pigpio

# class RosLineLidar(Node):
#     node_name = "ros_line_lidar"
#     line_pub_topic = "line"
    
    
#     def __init__(self):
#         super().__init__(self.node_name)
#         self.line_pub = self.create_publisher(Float32MultiArray, self.line_pub_topic, 10)
        
#         timer_period = 0.001
#         self.timer = self.create_timer(timer_period, self.timer_callback)
        

    
#     def timer_callback(self):
#         pass
        
class LineLidar:
    LINE_PIN = 2

    def __init__(self):
        self.pi = pigpio.pi()
        
        self.pi.set_mode(self.LINE_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(self.LINE_PIN,pigpio.PUD_UP)
        
        while True:
            self.mejour()
            time.sleep(0.000001)
        
    def mejour(self):
        new_pin = self.pi.read(self.LINE_PIN)
        if self.last_pin != new_pin:
            if new_pin:
                self.starttime = datetime.time.microsecond
            else:
                microsec = datetime.time.microsecond - self.starttime
            self.mm = 2*(microsec - 1000)
            print(self.mm)
        self.last_pin = new_pin
        
        
def main():
    linelidar = LineLidar()

if __name__ == "__main__":
    main()