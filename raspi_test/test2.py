import wiringpi
import time

limit_pin = 2
servo_pin = 12
servo_pin2 = 13
wiringpi.wiringPiSetupGpio()

wiringpi.pinMode(limit_pin, 0)
wiringpi.pullUpDnControl(limit_pin,wiringpi.PUD_DOWN)
wiringpi.pinMode(servo_pin, 2)
wiringpi.pinMode(servo_pin2, 2)

wiringpi.pwmSetMode(wiringpi.PWM_MODE_MS)
wiringpi.pwmSetRange(1024)
wiringpi.pwmSetClock(375)

def move_servo(deg):
	print(deg)
	move_deg = int(( 4.75 * deg / 90 + 7.25) / 100 * 1024)
	wiringpi.pwmWrite(servo_pin, move_deg)
	wiringpi.pwmWrite(servo_pin2, move_deg)

while True:
	move_servo(90)
	time.sleep(3)
	move_servo(-90)
	time.sleep(3)
