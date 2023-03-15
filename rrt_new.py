import random
import math
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from point_cloud import get_point_cloud
from time import time

class MainRRT:
    def __init__(self, start_1, goal, cube_len):
        self.side_len = cube_len
        self.segments = []
        self.end = goal
        self.start = start_1
        self.visited_nodes = [['q0', self.start, 'None']]
        self.gen_tree()
        self.path_nodes, self.path_segments = [], []  # nodes/segments along the path
        self.find_path()

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
                    print('pass')
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
    COLORS = ['#6AB71F', '#FF5733', '#4DAAEA', '#C0120A']
    start = (0, 0, 0)
    end = (15, 15, 0)
    cube_len = 20
    # call and generate RRT
    start_time = time()
    RRT = MainRRT(start, end, cube_len)
    end_time = time()-start_time
    print(end_time)

    #print(*RRT.path_segments, sep="\n")

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(list(zip(*RRT.path_nodes))[0], list(zip(*RRT.path_nodes))[1], list(zip(*RRT.path_nodes))[2])
    plt.show()

