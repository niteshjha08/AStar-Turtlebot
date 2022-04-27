#!/usr/bin/python3

from AStar import AstarPlanner
import sys
from math import degrees
import time

def main():
    rpm = [int(sys.argv[7]), int(sys.argv[8])]
    planner = AstarPlanner([int((float(sys.argv[1])+5)*10), int((float(sys.argv[2])+5)*10), int(degrees(float(sys.argv[3])))],
                  [int((float(sys.argv[4])+5)*10), int((float(sys.argv[5])+5)*10)], int(sys.argv[6]), rpm)
    plan_start = time.time()
    print("Started planning...")
    ret,path,_ = planner.Astar_search()
    plan_end = time.time()

    print("Completed planning in {} seconds".format(plan_end - plan_start))

    if ret == -1 :
        print("no solution")
        exit(1)
    draw_start = time.time()
    print("Drawing path...")
    planner.draw_path(path)
    draw_end = time.time()
    print("Completed drawing path in {} seconds.".format(draw_end-draw_start))

if __name__ == '__main__':
    main()
