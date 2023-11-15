import pigpio
import time

pi = pigpio.pi()

SERVO_PIN_LIST = [12, 13]
LIMIT_PIN_LIST = [2, 3, 4, 17, 27, 22, 23, 24]

get_pin_list = [0] * 8
callback_ob = []

for i in range(8):
    pi.set_mode(LIMIT_PIN_LIST[i], pigpio.INPUT)
    pi.set_pull_up_down(LIMIT_PIN_LIST[i], pigpio.PUD_UP)

def main():
    for pin in LIMIT_PIN_LIST:
        cb = pi.callback(pin, pigpio.FALLING_EDGE, pin_callback)
        callback_ob.append(cb)
    while True:
        pass

def pin_callback(gpio, level, tick):
    print(gpio, level, tick)


main()
