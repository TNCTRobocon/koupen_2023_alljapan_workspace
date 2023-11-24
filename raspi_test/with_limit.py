import pigpio
import time

pi = pigpio.pi()

SERVO_PIN_LIST = [12, 13]
LIMIT_PIN_LIST = [16, 20, 19, 26, 6, 5, 8, 7]

freq = 50

get_pin_list = [0] * 8
limit_tick_list = [0] * 8
callback_ob = []

old_tick = 0
servo_status = 0
servo_counter = 0

for i in range(len(LIMIT_PIN_LIST)):
	pi.set_mode(LIMIT_PIN_LIST[i], pigpio.INPUT)
	pi.set_pull_up_down(LIMIT_PIN_LIST[i], pigpio.PUD_UP)

def main():
	for pin in LIMIT_PIN_LIST:
		cb = pi.callback(pin, pigpio.EITHER_EDGE, pin_callback)
		callback_ob.append(cb)
	while True:
		print(get_pin_list)
		servo_check()
		time.sleep(0.05)

def reset_servo():
	move_servo(0)

def set_servo():
	move_servo(180)

def move_servo(deg):
	deg1 = int((deg * 9.5 / 180 + 2.5) * 10000)
	deg2 = int(((180 - deg) * 9.5 / 180 + 2.5) * 10000)
	pi.hardware_PWM(SERVO_PIN_LIST[0], freq, deg1)
	pi.hardware_PWM(SERVO_PIN_LIST[1], freq, deg2)

def servo_check():
	global servo_counter, servo_status
	print(servo_counter, servo_status)
	if get_pin_list[0] == 0 and get_pin_list[1] == 0:
		reset_servo()
		servo_counter = 0
		servo_status = 0
	elif get_pin_list[0] == 1 and get_pin_list[1] == 1:
		if servo_counter <= 5:
			set_servo()
			servo_status = 1
			servo_counter = 0
		else:
			servo_counter += 1
	elif get_pin_list[0] == 1 or get_pin_list[1] == 1:
		servo_counter += 1
	else:
		return
def pin_callback(gpio, level, tick):
	diff = abs(tick - limit_tick_list[LIMIT_PIN_LIST.index(gpio)])
	if diff > 10000:
		print(gpio, level, tick)
		changed_limit = LIMIT_PIN_LIST.index(gpio)
		get_pin_list[changed_limit] = level

	limit_tick_list[LIMIT_PIN_LIST.index(gpio)] = tick



main()
