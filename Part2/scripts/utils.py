from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from math import sqrt, cos, sin, radians, atan2, floor
import Config as cf
import numpy as np
from matplotlib.patches import Rectangle


def get_RPM_to_velocity(radius, l, path_point, action_set):
    velocity = Twist()
    rpm1, rpm2 = action_set[int(path_point.action)-1]
    velocity.linear.x = radius/2*(rpm1+rpm2)
    velocity.angular.z = radius/l*(rpm1-rpm2)
    return velocity
    
class Node:

    def __init__(self, start, env, goal, step_size, parent=None, action=None):
        self.env = env
        self.parent = parent
        self.goal = goal
        self.action = action

        if parent is not None:
            val_step = sqrt((env[0] - parent.env[0]) ** 2 + (env[1] - parent.env[1]) ** 2)
            self.g = parent.g + val_step
        else:
            self.g = 0

        self.weight = self.g + sqrt((env[0] - goal[0]) ** 2 + (env[1] - goal[1]) ** 2) + (
                (env[2] - floor(atan2((goal[1] - start[1]), (goal[0] - start[0])))) / 30)  

    def path(self):
        node, p = self, []
        while node:
            p.append(node)
            node = node.parent
        return reversed(p)

    def actions(self):
        if self.action is None:
            return self.env.action_check()
        else:
            return self.env.action_check(self.action)


class Robot:
    def __init__(self, curr_pose, clearance):
        self.curr_pose = curr_pose
        self.clearance = clearance

    def obs_c1(self, pose):
        if (pose[0] - 51) ** 2 + (pose[1] - 51) ** 2 <= (10 + self.clearance) ** 2:
            return True
        else:
            return False


    def obs_c2(self, pose):
        if (pose[0] - 31) ** 2 + (pose[1] - 21) ** 2 <= (10 + self.clearance) ** 2:
            return True
        else:
            return False

    def obs_c3(self, pose):
        if (pose[0] - 71) ** 2 + (pose[1] - 21) ** 2 <= (10 + self.clearance) ** 2:
            return True
        else:
            return False

    def obs_c4(self, pose):
        if (pose[0] - 71) ** 2 + (pose[1] - 81) ** 2 <= (10 + self.clearance) ** 2:
            return True
        else:
            return False

    def obs_s1(self, pose):
        if pose[0] > 23.5 - self.clearance and pose[0] < 38.5 + self.clearance and pose[
            1] < 88.5 + self.clearance and pose[1] > 73.5 - self.clearance:
            return True
        else:
            return False

    def obs_s2(self, pose):
        if pose[0] > 3.5 - self.clearance and pose[0] < 18.5 + self.clearance and pose[
            1] < 58.5 + self.clearance and pose[1] > 43.5 - self.clearance:
            return True
        else:
            return False

    def obs_s3(self, pose):
        if pose[0] > 83.5 - self.clearance and pose[0] < 98.5 + self.clearance and pose[
            1] < 58.5 + self.clearance and pose[1] > 43.5 - self.clearance:
            return True
        else:
            return False

    def outsideMap(self, pose):
        if pose[0] > 1.0 + self.clearance and pose[0] < 101.0 - self.clearance and pose[
            1] < 101.0 - self.clearance and pose[1] > 1.0 + self.clearance:
            return False
        else:
            return True

    def future_state_check(self, pose):
        flag_2 = True
        if self.obs_c1(pose):
            flag_2 = False
        if self.obs_c2(pose):
            flag_2 = False
        if self.obs_c3(pose):
            flag_2 = False
        if self.obs_c4(pose):
            flag_2 = False
        if self.obs_s1(pose):
            flag_2 = False
        if self.obs_s2(pose):
            flag_2 = False
        if self.obs_s3(pose):
            flag_2 = False
        if self.outsideMap(pose):
            flag_2 = False
        return flag_2


    def action_check(self, start, node, step_size):
        actions = []
        action = self.move(start, '1', step_size, node)
        if action is not None:
            actions.append(action)
        action = self.move(start, '2', step_size, node)
        if action is not None:
            actions.append(action)
        action = self.move(start, '3', step_size, node)
        if action is not None:
            actions.append(action)
        action = self.move(start, '4', step_size, node)
        if action is not None:
            actions.append(action)
        action = self.move(start, '5', step_size, node)
        if action is not None:
            actions.append(action)
        action = self.move(start, '6', step_size, node)
        if action is not None:
            actions.append(action)
        action = self.move(start, '7', step_size, node)
        if action is not None:
            actions.append(action)
        action = self.move(start, '8', step_size, node)
        if action is not None:
            actions.append(action)
        return actions

    def move(self, start, val, step_size, node):
        future_node = None
        if val == '1':
            flag_1 = True
            x = self.curr_pose[0]
            y = self.curr_pose[1]
            theta = self.curr_pose[2]
            for i in range(100):
                x, y, theta = self.step(x, y, theta, 0, step_size[1])
                if not self.future_state_check([x, y]):
                    flag_1 = False
            if flag_1:
                future_node = Node(start, [x, y, theta], node.goal, step_size, node, val)
        if val == '2':
            flag_1 = True
            x = self.curr_pose[0]
            y = self.curr_pose[1]
            theta = self.curr_pose[2]
            for i in range(100):
                x, y, theta = self.step(x, y, theta, step_size[0], step_size[1])
                if not self.future_state_check([x, y]):
                    flag_1 = False
            if flag_1:
                future_node = Node(start, [x, y, theta], node.goal, step_size, node, val)
        if val == '3':
            flag_1 = True
            x = self.curr_pose[0]
            y = self.curr_pose[1]
            theta = self.curr_pose[2]
            for i in range(100):
                x, y, theta = self.step(x, y, theta, step_size[1], step_size[1])
                if not self.future_state_check([x, y]):
                    flag_1 = False
            if flag_1:
                future_node = Node(start, [x, y, theta], node.goal, step_size, node, val)
        if val == '4':
            flag_1 = True
            x = self.curr_pose[0]
            y = self.curr_pose[1]
            theta = self.curr_pose[2]
            for i in range(100):
                x, y, theta = self.step(x, y, theta, step_size[1], step_size[0])
                if not self.future_state_check([x, y]):
                    flag_1 = False
            if flag_1:
                future_node = Node(start, [x, y, theta], node.goal, step_size, node, val)
        if val == '5':
            flag_1 = True
            x = self.curr_pose[0]
            y = self.curr_pose[1]
            theta = self.curr_pose[2]
            for i in range(100):
                x, y, theta = self.step(x, y, theta, step_size[1], 0)
                if not self.future_state_check([x, y]):
                    flag_1 = False
            if flag_1:
                future_node = Node(start, [x, y, theta], node.goal, step_size, node, val)
        if val == '6':
            flag_1 = True
            x = self.curr_pose[0]
            y = self.curr_pose[1]
            theta = self.curr_pose[2]
            for i in range(100):
                x, y, theta = self.step(x, y, theta, 0, step_size[0])
                if not self.future_state_check([x, y]):
                    flag_1 = False
            if flag_1:
                future_node = Node(start, [x, y, theta], node.goal, step_size, node, val)
        if val == '7':
            flag_1 = True
            x = self.curr_pose[0]
            y = self.curr_pose[1]
            theta = self.curr_pose[2]
            for i in range(100):
                x, y, theta = self.step(x, y, theta, step_size[0], step_size[0])
                if not self.future_state_check([x, y]):
                    flag_1 = False
            if flag_1:
                future_node = Node(start, [x, y, theta], node.goal, step_size, node, val)
        if val == '8':
            flag_1 = True
            x = self.curr_pose[0]
            y = self.curr_pose[1]
            theta = self.curr_pose[2]
            for i in range(100):
                x, y, theta = self.step(x, y, theta, step_size[0], 0)
                if not self.future_state_check([x, y]):
                    flag_1 = False
            if flag_1:
                future_node = Node(start, [x, y, theta], node.goal, step_size, node, val)
        return future_node

    def step(self, x, y, theta, rpm1, rpm2):
        r = 0.033
        l = 0.160
        rpm1 /= 10
        rpm2 /= 10
        x += r / 2 * (rpm1 + rpm2) * cos(radians(theta))
        y += r / 2 * (rpm1 + rpm2) * sin(radians(theta))
        theta += r / l * (rpm1 - rpm2)
        theta = self.angleCheck(theta)
        return x, y, theta

    def angleCheck(self, angle):
        if angle >= 360:
            angle -= 360
        if angle < 0:
            angle = 360 + angle
        return angle


