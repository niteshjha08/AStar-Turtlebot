#!/usr/bin/python3
from math import sqrt
import cv2
from map import Map
from utils import Node, Robot


class AstarPlanner:

    def __init__(self, start, goal, clearance, step_length):
        self.start = start
        self.goal = goal
        self.clearance = clearance
        self.step_length = step_length
        map = Map()
        self.map = map.map_img
        video_format = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
        self.video_output = cv2.VideoWriter('./2D_output.avi', video_format, 100.0,(1020, 1020))

    def Astar_search(self):
        search = []
        curr_node = Node(self.start, self.start, self.goal, self.step_length)
        list_visited = [curr_node]
        visited_dict = {tuple(curr_node.env)}
        search.append(curr_node)

        while sqrt((curr_node.env[0] - self.goal[0]) ** 2 + (curr_node.env[1] - self.goal[1]) ** 2) > 1.5:

            if len(list_visited) > 0:
                curr_node = list_visited.pop()

                robot = Robot(curr_node.env, self.clearance)
                for node in robot.possibleMoves(self.start, curr_node, self.step_length):

                    if tuple((int(node.env[0]), int(node.env[1]), node.env[2])) not in visited_dict:
                        list_visited.append(node)
                        search.append(node)
                        visited_dict.add(tuple((int(node.env[0]), int(node.env[1]), node.env[2])))
                        sub_nodes = node.sub_nodes
                        for i in range(len(sub_nodes) - 1):
                            cv2.line(self.map, (int(sub_nodes[i][0] * 10), 1020 - int(sub_nodes[i][1] * 10)),
                                     (int(sub_nodes[i + 1][0] * 10), 1020 - int(sub_nodes[i + 1][1] * 10)), (0, 255, 0))
                        self.video_output.write(self.map)
                list_visited.sort(key=lambda x: x.cost, reverse=True)

            else:
                return -1, curr_node.path(), search
        x = curr_node.path()
        path = []
        for node in x:
            path.append(node)
        return 0, path, search

    def draw_path(self, path):
        red = [0, 0, 255]
        blue = [255, 0, 0]
        green = [0, 255, 0]
 
        for i in range(1, len(path)):
            sub_nodes = path[i].sub_nodes
            for j in range(len(sub_nodes)-1):
                cv2.line(self.map, (int(sub_nodes[j][0] * 10), 1020 - int(sub_nodes[j][1] * 10)),
                         (int(sub_nodes[j + 1][0] * 10), 1020 - int(sub_nodes[j + 1][1] * 10)), red, 2)
            self.video_output.write(self.map)
           
            for _ in range(50):
                self.video_output.write(self.map)
        print("Video generated as 2D_output.avi!")
        self.video_output.release()
