#!/usr/bin/env python3
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
import math
import signal
import sys
import simple_algorithm as sa
import numpy as np

np.set_printoptions(threshold=sys.maxsize)

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

MAXANGLE = 90
MINANGLE = -90
STEP=5

power = 10

current_angle = [0, 0]
START = False
servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)

# initiate starting position and mapping
# mapping is 25x40 2D array, where 1 unit length means 5cm in real situation
starting_pos = (20,20)
total_mapping = np.empty(shape=(25,40))
total_mapping.fill(0)

def reset_servo():
    servo.set_angle(0)
    current_angle[0] = 0
    current_angle[1] = 0

# direction = 1 => left, direction = -1 => right, direction = 0 => initial
def next_angle(small, large):
    direction = current_angle[1]

    if (direction == 0):
        current_angle[1] = 1
        current_angle[0] += STEP
    elif(direction == 1):
        current_angle[0] += STEP
    elif(direction == -1):
        current_angle[0] -= STEP

    if current_angle[0] > large:
        current_angle[0] = large
        current_angle[1] = -1
    elif current_angle[0] < small:
        current_angle[0] = small
        current_angle[1] = 1

# step=delta angle for each step
def scan_around(min_distance, small, large, full=False):

    distance_list = {}

    count = 0
    while(count <= (large - small)):
        distance = fc.get_distance_at(current_angle[0])
        if (distance < min_distance and not full):
            return False
        distance_list.setdefault(current_angle[0], [])
        distance_list[current_angle[0]].append(distance)
        next_angle(small, large)
        count += STEP

    result = []
    for angle in distance_list:
        result.append((angle, sum(distance_list[angle]) / len(distance_list[angle])))

    #print(distance_list)

    return result

def calculate_c(a, b, gama):
    return math.sqrt(a ** 2 + b ** 2 - 2 * a * b * math.cos(gama))

def find_max_gap(obs):
    if (len(obs) == 0 or len(obs) == 1):
        return -2

    left = 0
    right = 1

    result = 0

    while(right < len(obs)):
        left_point = obs[left]
        right_point = obs[right]
        diff = abs(left_point[0] - right_point[0])
        if (diff == STEP):
            left = right
            right += 1
        else:
            left_length = left_point[1]
            right_length = right_point[1]
            result = max(result, left_length, right_length, diff)
            left = right
            right += 1

    print(obs, result)

    return result


def find_best_solution(min_distance, scan_list):

    left_obs = list(filter(lambda x: x[0] > 0 and (x[1] < min_distance and x[1] != -2), scan_list))
    right_obs = list(filter(lambda x: x[0] <= 0 and (x[1] < min_distance and x[1] != -2), scan_list))

    left_gap = find_max_gap(left_obs)
    right_gap = find_max_gap(right_obs)

    if (left_gap == 0 and right_gap == 0):
        return False
    elif(left_gap == 0):
        return "right"
    elif(right_gap == 0):
        return "left"
    else:
        if (right_gap > left_gap or right_gap == -2):
            return "right"
        elif(right_gap < left_gap or left_gap == -2):
            return "left"
        else:
            left_opts = len(left_obs)
            right_opts = len(right_obs)

            if (left_opts > right_opts):
                return "right"
            else:
                return "left"

def can_move(min_distance, scan_list):
    front_cond = list(filter(lambda x:(x[0] >= -1 * 2 * STEP and x[0] <= 2 * STEP) and (x[1] > min_distance or x[1] == -2), scan_list))
    angle_map = set()

    for condition in front_cond:
        angle, distance = condition
        angle_map.add(angle)

    print(angle_map)

    if (len(angle_map) >= 4):
        return True
    else:
        return False

def init_servo():
    servo.set_angle(0)

def turn(direction, speed):
    if direction == "left":
        fc.turn_left(speed)
    elif direction == "right":
        fc.turn_right(speed)
    else:
        fc.stop()

def forward_obs(distance):
    print(fc.get_distance_at(0), distance)
    return fc.get_distance_at(0) >= distance

# t should be half of the mapping size
def scan_180(step, t=36):
    prev_res = 0
    prev_pos = (0,0)
    offset_x, offset_y = starting_pos
    for i in range(-90, 90, step):
        dis = fc.get_distance_at(i)
        r = i / 180 * math.pi
        x = offset_x + int(math.sin(r * -1) * dis) // 2
        y = offset_y - int(math.cos(r * -1) * dis) // 2
        curr_pos = (x,y)
        print(curr_pos, dis, i, t)
        #print(curr_pos, offset_y, i, math.cos(i))
        if (i == -90):
            # if obj detected is far away or no obj detected
            if (dis > t or dis < 0):
                # nothing should change in the mapping table
                prev_res = 0
                prev_pos = curr_pos
                continue
            else:
                prev_res = 1
                prev_pos = curr_pos
                total_mapping[y][x] = 1
        else:
            if (dis > t or dis < 0):
                prev_res = 0
                prev_pos = curr_pos
                continue
            else:
                if (prev_res == 1):
                    sa.fill_in_line(total_mapping, prev_pos, curr_pos)
                else:
                    total_mapping[y][x] = 1
                prev_res = 1
                prev_pos = curr_pos





def move_simple(min_distance, speed):
    print(START)
    while(START):
        dist = fc.get_distance_at(0)
        if (dist < min_distance):
            fc.turn_left(speed)
        else:
            fc.forward(speed)
    
    fc.stop()

def gracefully_stop(a, b):
    START = False
    fc.stop()
    sys.exit(1)

def write_map():
    f = open("map.txt", "w")
    for line in total_mapping:
        list_string = list(map(lambda x:str(int(x)),line))
        string_to_write = "".join(list_string)
        f.write(string_to_write + "\n")
    f.close()

if __name__ == '__main__':
    init_servo()
    #sleep(1)
    #signal.signal(signal.SIGINT, gracefully_stop)
    START = True
    #move_simple(30, 10)
    scan_180(5)
    #print(total_mapping)
    init_servo()
    total_mapping[starting_pos[1]][starting_pos[0]] = 7
    write_map()
    #signal.pause()
