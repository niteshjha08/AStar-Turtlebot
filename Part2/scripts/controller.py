#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from AStar import AstarPlanner
from time import sleep
import sys
from math import degrees
from utils import get_RPM_to_velocity

def main():
    rospy.init_node('A_starNode')
    vel_publish = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    radius = 0.0033
    l = 0.9
    
    # Parse arguments and scale
    start_location = [int((float(sys.argv[1])+5)*10), int((float(sys.argv[2])+5)*10), int(degrees(float(sys.argv[3])))]
    goal_location = [int((float(sys.argv[4])+5)*10), int((float(sys.argv[5])+5)*10)]
    clearance = int(sys.argv[6])
    rpm = [int(sys.argv[7]), int(sys.argv[8])]
    action_set = [[0, rpm[1]],[rpm[0], rpm[1]],[rpm[1], rpm[1]],[rpm[1], rpm[0]],[rpm[1], 0],[0, rpm[0]],[rpm[0], rpm[0]],[rpm[0], 0]]
    
    path_planning = AstarPlanner(start_location,goal_location,radius,clearance,rpm)
    path,search = path_planning.Astar_search()


    for i in range(1, len(path)):
        velocity = get_RPM_to_velocity(radius, l, path[i], action_set)
        print(velocity.linear.x, velocity.angular.z)
        vel_publish.publish(velocity)
        sleep(10)

    # Stop 
    velocity = Twist()
    velocity.linear.x = 0
    velocity.angular.z = 0
    vel_publish.publish(velocity)


if __name__ == '__main__':
    main()
