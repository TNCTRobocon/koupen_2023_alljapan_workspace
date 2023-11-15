import RPI.GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
pwm0 = GPIO.PWM(18, 50)
pwm0.start(0)

try:
    while True:
        deg = 90
        deg2 = -90
        move_deg1 = int(( 4.75 * deg * -1 / 90 + 7.25) / 100 * 1024)
        move_deg2 = int(( 4.75 * deg2 * -1 / 90 + 7.25) / 100 * 1024)
        pwm0.ChangeDutyCycle(move_deg1)
        time.sleep(1.5)
        pwm0.ChangeDutyCycle(move_deg2)
except KeyboardInterrupt:
    pass
pwm0.stop()
GPIO.cleanup()
        