import random
import numpy as np
import math
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from time import time
import matplotlib.pyplot as plt
import scipy.stats as stats

class MainRRT:
    def __init__(self, start_1, goal, cube_len):
        self.side_len = cube_len
        self.segments = []
        self.end = self.gen_end_point(goal[0], goal[1], goal[2])
        print(self.end)
        self.start = start_1
        self.visited_nodes = [['q0', self.start, 'None']]
        self.gen_tree()
        self.path_nodes, self.path_segments = [], []  # nodes/segments along the path
        self.find_path()
        #print("Initialized?")

    def gen_end_point(self, x, y, z):
        
        
        lower, upper = -2, 2
        mu, sigma = 0, 2
        
        rand_x = -1 
        rand_y = -1
        rand_z = -1
        while rand_x > self.side_len or rand_x < 0:
            x_1 = x + np.round(stats.truncnorm((lower - mu) / sigma, (upper - mu) / sigma, loc=mu, scale=sigma).rvs(10))
            rand_x = random.choice(x_1)
        while rand_y > self.side_len or rand_y < 0: 
            y_1 = y + np.round(stats.truncnorm((lower - mu) / sigma, (upper - mu) / sigma, loc=mu, scale=sigma).rvs(10))
            rand_y = random.choice(y_1)
        while rand_z > self.side_len or rand_z < 0:
            z_1 = z + np.round(stats.truncnorm((lower - mu) / sigma, (upper - mu) / sigma, loc=mu, scale=sigma).rvs(10))
            rand_z = random.choice(z_1)
        
        return rand_x, rand_y, rand_z
    

    def round_up(self, x):
        return round(x)

    def distance_cal(self, p1, p2):
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2)

    def closest_node(self, point):
        d_list = [self.distance_cal(point, node[1]) for node in self.visited_nodes]
        return min(range(len(d_list)), key=d_list.__getitem__)

    def gen_node(self):
        node_name = "q{}".format(len(self.visited_nodes))
        point_ok = False

        while not point_ok:
            p_coords = (self.side_len*random.random(), self.side_len*random.random(), self.side_len*random.random())
            parent = self.visited_nodes[self.closest_node(p_coords)]

            # x- and y-distances to random point from parent node
            d_x = p_coords[0] - parent[1][0]
            d_y = p_coords[1] - parent[1][1]
            d_z = p_coords[2] - parent[1][2]

            # magnitude of vector to closest node
            vec_mag = math.sqrt((d_x**2) + (d_y**2) + (d_z**2))

            # get new node coordinates by adding unit vector components to parent coordinates
            node = (self.round_up(parent[1][0] + d_x/vec_mag),
                    self.round_up(parent[1][1] + d_y/vec_mag),
                    self.round_up(parent[1][2] + d_z/vec_mag))

            # if newly created node
            for mm in range(len(self.visited_nodes)):
                if self.visited_nodes[mm][1] == node:
                    #print('pass')
                    pass
                else:
                    point_ok = True

        self.visited_nodes.append([node_name, node, parent[0]])
        self.segments.append([parent[1], node])

    def gen_end_seg(self):
        self.segments.append([self.visited_nodes[-1][1], self.end])

    def gen_tree(self):
        done = False

        while not done:
            self.gen_node()
            if self.visited_nodes[-1][1] == self.end:
                done = True

        self.gen_end_seg()

        self.visited_nodes.append(["q{}".format(len(self.visited_nodes)), self.end, self.visited_nodes[-1][0]])

    def find_path(self):
        current = self.visited_nodes[-1]  # set end as current node
        self.path_nodes.append(current[1])  # append end coordinates to list of path nodes

        for _, j in reversed(list(enumerate(self.visited_nodes))):
            if current[2] == j[0]:
                self.path_nodes.insert(0, j[1])
                self.path_segments.insert(0, (j[1], current[1]))
                current = j


if __name__ == "__main__":
    # list of plotting colors
    # [start, end, points, path]
    count = 0
    COLORS = ['#6AB71F', '#FF5733', '#4DAAEA', '#C0120A']
    start = (0, 20, 0)
    end = (40, 0, 40)
    cube_len = 50
    
    for _ in range(5):
        # call and generate RRT
        print(count)
        print("Starting RRT")
        start_time = time()
        RRT = MainRRT(start, end, cube_len)
        end_time = time()-start_time
        print(RRT.path_nodes)
        count+=1

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(list(zip(*RRT.path_nodes))[0], list(zip(*RRT.path_nodes))[1], list(zip(*RRT.path_nodes))[2])
    plt.show()

