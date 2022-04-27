from math import sqrt, cos, sin, radians, atan2, floor

class Node:
    def __init__(self, start, env, goal, step_length, parent=None, action=None, sub_nodes=None):
        self.env = env
        self.parent = parent
        self.goal = goal
        self.action = action
        self.sub_nodes = sub_nodes

        if parent is not None:
            step = sqrt((env[0] - parent.env[0]) ** 2 + (env[1] - parent.env[1]) ** 2)
            self.g = parent.g + step
        else:
            self.g = 0

        self.cost = self.g + sqrt((env[0] - goal[0]) ** 2 + (env[1] - goal[1]) ** 2) + (
                (env[2] - floor(atan2((goal[1] - start[1]), (goal[0] - start[0])))) / 30) 

    def path(self):
        node, p = self, []
        while node:
            p.append(node)
            node = node.parent
        return reversed(p)

    def actions(self):
        if self.action is None:
            return self.env.possibleMoves()
        else:
            return self.env.possibleMoves(self.action)


class Robot:
    def __init__(self, currentPosition, clearance):
        self.currentPosition = currentPosition
        self.clearance = clearance

    def insideCircle1(self, position):
        if (position[0] - 51) ** 2 + (position[1] - 51) ** 2 <= (10 + self.clearance) ** 2:
            return True
        else:
            return False

    def insideCircle2(self, position):
        if (position[0] - 31) ** 2 + (position[1] - 21) ** 2 <= (10 + self.clearance) ** 2:
            return True
        else:
            return False

    def insideCircle3(self, position):
        if (position[0] - 71) ** 2 + (position[1] - 21) ** 2 <= (10 + self.clearance) ** 2:
            return True
        else:
            return False

    def insideCircle4(self, position):
        if (position[0] - 71) ** 2 + (position[1] - 81) ** 2 <= (10 + self.clearance) ** 2:
            return True
        else:
            return False

    def insideSquare1(self, position):
        if position[0] > 23.5 - self.clearance and position[0] < 38.5 + self.clearance and position[
            1] < 88.5 + self.clearance and position[1] > 73.5 - self.clearance:
            return True
        else:
            return False

    def insideSquare2(self, position):
        if position[0] > 3.5 - self.clearance and position[0] < 18.5 + self.clearance and position[
            1] < 58.5 + self.clearance and position[1] > 43.5 - self.clearance:
            return True
        else:
            return False

    def insideSquare3(self, position):
        if position[0] > 83.5 - self.clearance and position[0] < 98.5 + self.clearance and position[
            1] < 58.5 + self.clearance and position[1] > 43.5 - self.clearance:
            return True
        else:
            return False

    def outsideMap(self, position):
        if position[0] > 1.0 + self.clearance and position[0] < 101.0 - self.clearance and position[
            1] < 101.0 - self.clearance and position[1] > 1.0 + self.clearance:
            return False
        else:
            return True

    def possiblePostion(self, position):
        possiblity = True
        if self.insideCircle1(position):
            possiblity = False
        if self.insideCircle2(position):
            possiblity = False
        if self.insideCircle3(position):
            possiblity = False
        if self.insideCircle4(position):
            possiblity = False
        if self.insideSquare1(position):
            possiblity = False
        if self.insideSquare2(position):
            possiblity = False
        if self.insideSquare3(position):
            possiblity = False
        if self.outsideMap(position):
            possiblity = False
        return possiblity

    def possibleMoves(self, start, node, step_length):
        actions = []
        temp = self.move(start, '1', step_length, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '2', step_length, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '3', step_length, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '4', step_length, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '5', step_length, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '6', step_length, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '7', step_length, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '8', step_length, node)
        if temp is not None:
            actions.append(temp)
        return actions
    def move(self, start, val, step_length, node):
        temp = None
        sub_nodes = []
        if val == '1':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, 0, step_length[1])
                sub_nodes.append((x, y))
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
                    break
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, step_length, node, val, sub_nodes)
        if val == '2':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, step_length[0], step_length[1])
                sub_nodes.append((x, y))
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
                    break
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, step_length, node, val, sub_nodes)
        if val == '3':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, step_length[1], step_length[1])
                sub_nodes.append((x, y))
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
                    break
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, step_length, node, val, sub_nodes)
        if val == '4':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, step_length[1], step_length[0])
                sub_nodes.append((x, y))
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
                    break
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, step_length, node, val, sub_nodes)
        if val == '5':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, step_length[1], 0)
                sub_nodes.append((x, y))
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
                    break
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, step_length, node, val, sub_nodes)
        if val == '6':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, 0, step_length[0])
                sub_nodes.append((x, y))
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
                    break
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, step_length, node, val, sub_nodes)
        if val == '7':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, step_length[0], step_length[0])
                sub_nodes.append((x, y))
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
                    break
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, step_length, node, val, sub_nodes)
        if val == '8':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, step_length[0], 0)
                sub_nodes.append((x, y))
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
                    break
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, step_length, node, val, sub_nodes)
        return temp

    def increment(self, x, y, theta, rpm1, rpm2):
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
