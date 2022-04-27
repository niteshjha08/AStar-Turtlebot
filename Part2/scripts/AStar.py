from math import sqrt
from utils import Node, Robot


class AstarPlanner:
    def __init__(self, start, goal, radius, clearance, step_length):
        self.start = start
        self.goal = goal
        self.radius = radius
        self.clearance = clearance
        self.step_length = step_length

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
               
                for action in robot.action_check(self.start, curr_node, self.step_length):

                    if tuple((int(action.env[0]), int(action.env[1]), action.env[2])) not in visited_dict:
                        list_visited.append(action)
                        search.append(action)
                        visited_dict.add(tuple((int(action.env[0]), int(action.env[1]), action.env[2])))

                list_visited.sort(key=lambda x: x.weight, reverse=True)

            else:
                return -1, curr_node.path(), search
        # Backtracking
        x = curr_node.path()
        path = []
        for node in x:
            path.append(node)
        return path, search
