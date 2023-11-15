import pigpio
import time

pig = pigpio.pi()

servo1 = 12

deg1 = 0
deg2 = 180
duty1 =int((deg1 * 9.5 / 180 + 2.5) * 10000)
duty2 =int((deg2 * 9.5 / 180 + 2.5) * 10000)

freq = 50 

while True:
    print("1")
    time.sleep(3)
    pig.hardware_PWM(servo1, freq, duty1)
    print("2")
    time.sleep(3)
    pig.hardware_PWM(servo1, freq, duty2)
    
