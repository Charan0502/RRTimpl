import math
import random
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRTStar:
    def __init__(self, start, goal, obstacle_list, x_limit, y_limit, search_radius=50, step_size=5, max_iterations=1000):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacle_list = obstacle_list
        self.x_limit = x_limit
        self.y_limit = y_limit
        self.search_radius = search_radius
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.node_list = []

    def planning(self):
        self.node_list.append(self.start)
        for i in range(self.max_iterations):
            random_node = self.generate_random_node()
            nearest_node = self.find_nearest_node(random_node)
            new_node = self.steer(nearest_node, random_node)
            if self.is_valid_node(new_node):
                near_nodes = self.find_near_nodes(new_node)
                min_cost_node = self.choose_parent(new_node, near_nodes)
                self.node_list.append(new_node)
                self.rewire(new_node, near_nodes)
                self.propagate_cost_to_leaves(min_cost_node)
                if self.is_goal_reachable(new_node):
                    return self.generate_path()
        return None

    def generate_random_node(self):
        random_node = Node(random.uniform(0, self.x_limit), random.uniform(0, self.y_limit))

        return random_node

    def find_nearest_node(self, random_node):
        distances = [(node.x - random_node.x) ** 2 + (node.y - random_node.y) ** 2 for node in self.node_list]
        nearest_node = self.node_list[distances.index(min(distances))]
        return nearest_node

    def steer(self, from_node, to_node):
        distance = math.sqrt((from_node.x - to_node.x) ** 2 + (from_node.y - to_node.y) ** 2)
        if distance <= self.step_size:
            return to_node
        else:
            theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            return Node(new_x, new_y)

    def is_valid_node(self, node):
        if node.x < 0 or node.x > self.x_limit or node.y < 0 or node.y > self.y_limit:
            return False
        for (ox, oy, radius) in self.obstacle_list:
            if (node.x - ox) ** 2 + (node.y - oy) ** 2 <= radius ** 2:
                return False
        return True

    def find_near_nodes(self, node):
        nnode = len(self.node_list) + 1
        r = self.search_radius * math.sqrt((math.log(nnode) / nnode))
        near_nodes = [n for n in self.node_list if self.calculate_distance(n, node) <= r]
        return near_nodes

    def choose_parent(self, node, near_nodes):
        if not near_nodes:
            return node
        costs= [self.calculate_cost(n) + self.calculate_distance(n, node) for n in near_nodes]
        min_cost = min(costs)
        min_cost_index = costs.index(min_cost)
        node.parent = near_nodes[min_cost_index]
        return near_nodes[min_cost_index]

    def rewire(self, node, near_nodes):
        for near_node in near_nodes:
            near_cost = self.calculate_cost(node) + self.calculate_distance(node, near_node)
            if near_cost < self.calculate_cost(near_node):
                near_node.parent = node

    def propagate_cost_to_leaves(self, node):
        for n in self.node_list:
            if n.parent == node:
                n.cost = self.calculate_cost(n)
                self.propagate_cost_to_leaves(n)

    def calculate_cost(self, node):
        cost = 0.0
        while node.parent:
            cost += self.calculate_distance(node, node.parent)
            node = node.parent
        return cost

    def calculate_distance(self, from_node, to_node):
        return math.sqrt((from_node.x - to_node.x) ** 2 + (from_node.y - to_node.y) ** 2)

    def is_goal_reachable(self, node):
        return self.calculate_distance(node, self.goal) <= self.step_size

    def generate_path(self):
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[-1]
        while node.parent:
            path.append([node.x, node.y])
            node = node.parent
        path.append([self.start.x, self.start.y])
        path.reverse()
        return path

def main():
    start = (10, 10)
    goal = (90, 90)
    obstacle_list = [(40, 40, 10), (60, 60, 15), (70, 20, 8), (20, 50, 12)]
    x_limit = 100
    y_limit = 100
    rrt_star = RRTStar(start, goal, obstacle_list, x_limit, y_limit)

    path = rrt_star.planning()

    if path is None:
        print("No path found!")
    else:
        print("Path found!")
        plt.figure()
        for (ox, oy, radius) in obstacle_list:
            circle = plt.Circle((ox, oy), radius, fc='r')
            plt.gca().add_patch(circle)
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-b')
        plt.plot(start[0], start[1], 'ro')
        plt.plot(goal[0], goal[1], 'go')
        plt.axis('equal')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('RRT* Path Planning')
        plt.show()

if __name__=="__main__":
    print("abc")
    main()


