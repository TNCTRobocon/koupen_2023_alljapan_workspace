import RPi.GPIO as GPIO	
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
pwm0 = GPIO.PWM(12, 50)

try:
    while True:
        deg = 0
        deg2 = 180
        move_deg1 = (deg * 9.5 / 180 + 2.5)
        move_deg2 = (deg2 * 9.5 / 180 + 2.5)
        print(move_deg1)
        pwm0.start(move_deg1)
        pwm0.ChangeDutyCycle(move_deg1)
        time.sleep(5)
        pwm0.stop()
        print(move_deg2)
        pwm0.start(move_deg2)
        pwm0.ChangeDutyCycle(move_deg2)
        time.sleep(5)
        pwm0.stop()

except KeyboardInterrupt:
    pass
GPIO.cleanup()
        
