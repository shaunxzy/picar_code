#!/bin/python3
import picar_4wd as fc
from picar_4wd.speed import Speed
import time
import signal
import math
import numpy as np
import simple_algorithm as sa
import astar as astr

# to what extend roadMap needs to be shrank
FACTOR = 9

# initiate roadMap, roadMap maps 200cm x 200cm surroundings
roadMap = np.empty(shape=(30,30))

def initialize_map():
    roadMap.fill(0)

# write last roadMap to result
def write_map():
    f = open("map.txt", "w")
    for line in roadMap:
        list_string = list(map(lambda x:str(int(x)),line))
        string_to_write = "".join(list_string)
        f.write(string_to_write + "\n")
    f.close()


# accept SIGINT and stop car
def stop_car(a, b):
    fc.stop()
    write_map()
    exit(1)

def pause_car(a, b):
    fc.stop()
    print("stop sign detected!")
    time.sleep(10)
    return 0

def mark_one(pos):
    row, col = pos

    row_before = max(row - 1, 0)
    row_after = min(row + 1,len(roadMap) - 1)
    col_before = max(col - 1,0)
    col_after = min(col + 1, len(roadMap[0]) - 1)

    for r in range(row_before, row_after + 1):
        for c in range(col_before, col_after + 1):
           roadMap[r][c] = 1
    #roadMap[row][col] = 1

def A_star(pos, target):
    pr, pc = pos
    roadMap[pr][pc] = 0
    #print(roadMap, pos, target)
    paths = astr.astar(roadMap, pos, target)
    #print(paths)
    if (paths == None or len(paths) < 2):
        print("no path found")
        return None
    else:
        return paths[1]

# scan 180 degree to initialize surroundings
def scan_180(pos, ref=36):
    prev_res = 0
    prev_pos = (0,0)
    offset_ro, offset_co = pos
    for i in range(-90, 90, 5):
        dis = fc.get_distance_at(i)
        #dis = fc.get_distance_at(i)
        r = i / 180 * math.pi
        ro = offset_ro - int(math.cos(r) * dis) // FACTOR
        co = offset_co - int(math.sin(r) * dis) // FACTOR
        curr_pos = (ro, co)
        #print(curr_pos, dis, i, ref)
        #print(curr_pos, offset_y, i, math.cos(i))
        if (i == -90):
            # if obj detected is far away or no obj detected
            if (dis > ref or dis < 0):
                # nothing should change in the mapping table
                prev_res = 0
                prev_pos = curr_pos
                continue
            else:
                prev_res = 1
                prev_pos = curr_pos
                mark_one(curr_pos)
        else:
            if (dis > ref or dis < 0):
                prev_res = 0
                prev_pos = curr_pos
                continue
            else:
                if (prev_res == 1):
                    sa.fill_in_line(roadMap, prev_pos, curr_pos)
                else:
                    mark_one(curr_pos)
                prev_res = 1
                prev_pos = curr_pos
        
    #print("initial scan finished")

# define car class here
class Car():
    def __init__(self, pos, target, power=10):
        # current position of car in the map (row, col)
        self.pos = pos
        # target is tuple (row, col)
        self.target = target
        # next pos to arrive in order to reach target
        self.next_pos = None
        # default direction is north
        self.direction = (-1,0)
        # speed object from Speed class, initialize it
        self.speed = Speed(25)
        self.speed.start()
        # define speed exptected
        self.power = power
        # distance(cm) moved from the last position
        self.moved = 0
        # current servo angle, in degree
        self.angle = 0
        # whether servo is turning left(-1), right(1)
        self.swing = -1
        # last pos where an obstacle was spot
        self.obs_pos = None
        # last time car's position is updated
        self.time = time.time()
        
        # four different direction map, south, east, north, west
        self.directionMap = [(1,0), (0,1), (-1,0), (0,-1)]

    # make car move towwads the direction provided
    # direction could be (1,0), (0,1), (-1,0), (0,-1)
    def move(self, direction):
        self.update_pos()
        if (direction == self.direction):
            fc.forward(self.power)
        else:
            ci = self.directionMap.index(self.direction)
            di = self.directionMap.index(direction)

            if (abs(ci - di) > 2):
                # ci is at (1,0), di at (0,-1)
                if (ci < di):
                    self.turn_right_90()
                # ci at (0,-1), di at (1,0)
                else:
                    self.turn_left_90()
            elif(abs(ci - di) == 2):
                # revert direction
                self.turn_180()
            else:
                if (ci < di):
                    # turn left
                    self.turn_left_90()
                else:
                    # turn right
                    self.turn_right_90()

            # reset moved distance since we changed our direction
            self.moved = 0
            self.direction = direction
            self.time = time.time()
            fc.forward(self.power)
            #print("move to " + str(direction))
    
    # update car's position in the roadMap, assuming car's speed and direction kept constant
    # from last self.time input till current
    def update_pos(self):
        t = time.time()
        spd = self.speed
        spd_value = spd()
        time_lapse = t - self.time
        distance = spd_value * time_lapse

        self.moved += distance

        if (self.moved > FACTOR):
            unit = int(self.moved // FACTOR)
            self.pos = (int(self.pos[0] + unit * self.direction[0]),
                        int(self.pos[1] + unit * self.direction[1]))
            self.moved -= unit * FACTOR
        #print("move to " + str(self.direction)  +  "current pos is " + str(self.pos))
        self.time = time.time()
    
    # make servo scan a certain angle, and update roadMap
    def scan_next(self,ref=36):
        curr_angle = 0
        if (self.angle == -72):
            curr_angle = -60
            self.swing = 1
        elif(self.angle == 72):
            curr_angle = 60
            self.swing = -1
        else:
            curr_angle = self.angle + 12 * self.swing

        dist = fc.get_distance_at(curr_angle)
        #dist = fc.get_distance_at(curr_angle)
        offset_ro, offset_co = self.pos
        rd = curr_angle / 180 * math.pi
        ro,co = (0,0)

        # pointing to north
        if (self.direction == (-1,0)):
            ro = offset_ro - int(math.cos(rd) * dist) // FACTOR
            co = offset_co - int(math.sin(rd) * dist) // FACTOR
        elif (self.direction == (1,0)):
            ro = offset_ro + int(math.cos(rd) * dist) // FACTOR
            co = offset_co + int(math.sin(rd) * dist) // FACTOR
        elif (self.direction == (0,-1)):
            ro = offset_ro + int(math.sin(rd) * dist) // FACTOR
            co = offset_co - int(math.cos(rd) * dist) // FACTOR
        else:
            ro = offset_ro - int(math.sin(rd) * dist) // FACTOR
            co = offset_co + int(math.cos(rd) * dist) // FACTOR
        
        curr_pos = (ro,co)

        if (dist >= 0 and dist <= ref):
            if (self.obs_pos == None):
                mark_one(curr_pos)
                self.obs_pos = curr_pos
            else:
                sa.fill_in_line(roadMap, self.obs_pos, curr_pos)
                self.obs_pos = curr_pos
        else:
            # no obstacle detected, do nothing
            self.obs_pos = None

        self.angle = curr_angle
        self.update_pos()
    
    # route to destination
    def routing(self):
        scan_180(self.pos)
        while(not self.arrive()):
            #print("routing...")
            self.next_pos = A_star(self.pos, self.target)
            if (self.next_pos == None):
                print("no route break")
                break
            print("routing")
            temp_dir = (self.next_pos[0] - self.pos[0], self.next_pos[1] - self.pos[1])
            self.move(temp_dir)
            self.scan_next()
            print("updating map")
            self.update_pos()
    
        fc.stop()
        self.speed.deinit()

    def arrive(self):
        return self.pos == self.target

    def turn_left_90(self):
        fc.turn_left(10)
        time.sleep(0.93)
        fc.stop()

    def turn_right_90(self):
        fc.turn_right(10)
        time.sleep(1)
        fc.stop()

    def turn_180(self):
        fc.turn_left(10)
        time.sleep(1.86)
        fc.stop()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, stop_car)
    signal.signal(signal.SIGALRM, pause_car)
    initialize_map()
    car = Car((25,15), (0, 14), 2)
    #scan_180(car.pos)
    time.sleep(10)
    car.routing()
    #write_map()
    write_map()
    #time.sleep(5)
    #initialize_map()
    #car2 = Car((25, 15), (12,20), 10)
    #car2.routing()
    #print(astr.astar(roadMap, (25, 15), (2, 13)))
    #write_map()
    #turn_left_90()
    #time.sleep(1)
    #turn_right_90()
    #time.sleep(1)
    



