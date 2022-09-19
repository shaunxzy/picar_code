#/usr/bin/env python3
# -*- coding: utf-8 -*-
import picar_4wd as fc
from picar_4wd.pwm import PWM
from picar_4wd.adc import ADC
from picar_4wd.pin import Pin
from picar_4wd.motor import Motor
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.speed import Speed
from picar_4wd.filedb import FileDB
from picar_4wd.utils import *
import time

# Config File:
config = FileDB("config")
left_front_reverse = config.get('left_front_reverse', default_value = False)
right_front_reverse = config.get('right_front_reverse', default_value = False)
left_rear_reverse = config.get('left_rear_reverse', default_value = False)
right_rear_reverse = config.get('right_rear_reverse', default_value = False)
ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value = 0))

# Init motors
left_front = Motor(PWM("P13"), Pin("D4"), is_reversed=left_front_reverse) # motor 1
right_front = Motor(PWM("P12"), Pin("D5"), is_reversed=right_front_reverse) # motor 2
left_rear = Motor(PWM("P8"), Pin("D11"), is_reversed=left_rear_reverse) # motor 3
right_rear = Motor(PWM("P9"), Pin("D15"), is_reversed=right_rear_reverse) # motor 4

power = 30

def moveAround():
    left_front.set_power(0.1)
    left_rear.set_power(0.1)
    right_front.set_power(power)
    right_rear.set_power(power)


    for circle in range(1):
        time.sleep(5)

    fc.stop()

def move25():
    speed4 = Speed(25)
    speed4.start()

    fc.backward(100)
    x = 0
    for i in range(1):
        time.sleep(0.1)
        speed = speed4.speed

        x += speed * 0.1
        print("%smm/s"%speed)
    print("%smm"%x)
    speed4.deinit()
    fc.stop()

def testServo(angle):
    servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)

    servo.set_angle(angle)

def obstacle_test(distance):
    init_time = time.time()
    while True:
        dis = fc.get_distance_at(0)
        print(dis)
        if (dis < distance or time.time() - init_time > 10):
            fc.stop()
            break
        else:
            fc.forward(power)

if __name__ == '__main__':
    obstacle_test(20)
