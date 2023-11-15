import pigpio
import time

pi = pigpio.pi()

servo1 = 18

deg1 = 0
deg2 = 180
duty1 = (deg1 * 9.5 / 180 + 2.5) * 10000
duty2 = (deg2 * 9.5 / 180 + 2.5) * 10000

freq = 50 

while True:
    time.sleep(3)
    pi.hardware_PWM(servo1, freq, duty1)
    time.sleep(3)
    pi.hardware_PWM(servo1, freq, duty2)
    
