# -*- coding: utf-8 -*-

from random import randint
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import numpy as np
import random
import sys
import itertools
import pickle


class Workspace(object):
    """
    define the workspace where robots reside
    """
    def __init__(self):
        # dimension of the workspace
        # self.length = int(sys.argv[1])
        # self.width = int(sys.argv[1])
        # n = int(sys.argv[2])
        self.length = 10   # length
        self.width = 10   # width
        # n = 4
        self.type_num = {1: 4, 2: 4}   # single-task robot
        self.workspace = (self.length, self.width)
        self.num_of_regions = 10
        self.num_of_obstacles = 10
        self.occupied = []
        self.regions = {'l{0}'.format(i+1): j for i, j in enumerate(self.allocate_region_dars())}
        self.obstacles = {'o{0}'.format(i+1): j for i, j in enumerate(self.allocate_obstacle_dars())}
        self.type_robot_location = self.initialize()
        # region and corresponding locations
        self.label_location = {'r{0}'.format(i + 1): j for i, j in enumerate(list(self.type_robot_location.values()))}
        # region where robots reside
        self.type_robot_label = dict(zip(self.type_robot_location.keys(), self.label_location.keys()))
        # atomic proposition
        self.atomic_prop = self.get_atomic_prop()

        self.graph_workspace = nx.Graph()
        self.build_graph()

        self.p2p = self.point_to_point_path()  # label2label path

    def reachable(self, location, obstacles):
        next_location = []
        # left
        if location[0]-1 > 0 and (location[0]-1, location[1]) not in obstacles:
            next_location.append((location, (location[0]-1, location[1])))
        # right
        if location[0]+1 < self.width and (location[0]+1, location[1]) not in obstacles:
            next_location.append((location, (location[0]+1, location[1])))
        # up
        if location[1]+1 < self.length and (location[0], location[1]+1) not in obstacles:
            next_location.append((location, (location[0], location[1]+1)))
        # down
        if location[1]-1 > 0 and (location[0], location[1]-1) not in obstacles:
            next_location.append((location, (location[0], location[1]-1)))
        return next_location

    def build_graph(self):
        obstacles = list(itertools.chain(*self.obstacles.values()))
        for i in range(self.width):
            for j in range(self.length):
                if (i, j) not in obstacles:
                    self.graph_workspace.add_edges_from(self.reachable((i, j), obstacles))

    def get_atomic_prop(self):
        atomic_prop = dict()
        for type_robot, location in self.type_robot_location.items():
            for region, cells in self.regions.items():
                if location in cells:
                    if (region, type_robot[0]) not in atomic_prop.keys():
                        atomic_prop[(region, type_robot[0])] = 1
                    else:
                        atomic_prop[(region, type_robot[0])] += 1
        return atomic_prop

    def point_to_point_path(self):
        key_region = list(self.regions.keys())
        key_init = list(self.label_location.keys())

        p2p = dict()
        for l1 in range(len(self.regions)):
            for l2 in range(l1, len(self.regions)):
                min_length = np.inf
                for source in self.regions[key_region[l1]]:
                    for target in self.regions[key_region[l2]]:
                        length, _ = nx.algorithms.single_source_dijkstra(self.graph_workspace, source=source,
                                                                         target=target)
                        if length < min_length:
                            min_length = length
                p2p[(key_region[l1], key_region[l2])] = min_length
                p2p[(key_region[l2], key_region[l1])] = min_length
        # with open('data/p2p_large_workspace', 'wb') as filehandle:
        #     pickle.dump(p2p, filehandle)
        # with open('data/p2p_large_workspace', 'rb') as filehandle:
        #     p2p = pickle.load(filehandle)
        for r1 in range(len(self.label_location)):
            for l1 in range(len(self.regions)):
                min_length = np.inf
                for target in self.regions[key_region[l1]]:
                    length, _ = nx.algorithms.single_source_dijkstra(self.graph_workspace,
                                                                    source=self.label_location[key_init[r1]],
                                                                    target=target)
                    if length < min_length:
                        min_length = length
                p2p[(key_init[r1], key_region[l1])] = min_length
                p2p[(key_region[l1], key_init[r1])] = min_length

        for r1 in range(len(self.label_location)):
            for r2 in range(r1, len(self.label_location)):
                length, path = nx.algorithms.single_source_dijkstra(self.graph_workspace,
                                                                    source=self.label_location[key_init[r1]],
                                                                    target=self.label_location[key_init[r2]])
                p2p[(key_init[r1], key_init[r2])] = length
                p2p[(key_init[r2], key_init[r1])] = length

        return p2p

    def plot_workspace(self):
        ax = plt.figure(1).gca()
        ax.set_xlim((0, self.width))
        ax.set_ylim((0, self.length))
        plt.xticks(np.arange(0, self.width + 1, 1.0))
        plt.yticks(np.arange(0, self.length + 1, 1.0))
        # self.plot_workspace_helper(ax, self.regions, 'region')
        self.plot_workspace_helper(ax, self.obstacles, 'obstacle')
        for index, i in self.type_robot_location.items():
            plt.plot(i[0] + 0.5, i[1] + 0.5, 'o')
            ax.text(i[0] + 0.5, i[1] + 0.5, r'${}$'.format(index), fontsize=10)

    def plot_workspace_helper(self, ax, obj, obj_label):
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid(b=True, which='major', color='k', linestyle='--')
        for key in obj:
            color = 'b' if obj_label != 'region' else 'c'
            for grid in obj[key]:
                x_ = grid[0]
                y_ = grid[1]
                x = []
                y = []
                patches = []
                for point in [(x_, y_), (x_ + 1, y_), (x_ + 1, y_ + 1), (x_, y_ + 1)]:
                    x.append(point[0])
                    y.append(point[1])
                polygon = Polygon(np.column_stack((x, y)), True)
                patches.append(polygon)
                p = PatchCollection(patches, facecolors=color, edgecolors=color, alpha=0.4)
                ax.add_collection(p)
            ax.text(np.mean(x) - 0.2, np.mean(y) - 0.2, r'${}_{{{}}}$'.format(key[0], key[1:]), fontsize=8)

    def path_plot(self, robot_path):
        """
        plot the path
        :param path: found path
        :param workspace: workspace
        :param number_of_robots:
        :return: figure
        """

        for robot, path in robot_path.items():
            # prefix path
            if len(path) == 1:
                continue
            x_pre = np.asarray([point[0] + 0.5 for point in path])
            y_pre = np.asarray([point[1] + 0.5 for point in path])
            plt.quiver(x_pre[:-1], y_pre[:-1], x_pre[1:] - x_pre[:-1], y_pre[1:] - y_pre[:-1],
                       color="#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)]),
                       scale_units='xy', angles='xy', scale=1, label='prefix path')

            plt.savefig('img/path.png', bbox_inches='tight', dpi=600)

    def initialize(self):
        type_robot_location = dict()
        x0 = self.regions['l1'].copy()
        # x0.remove((1, 4))
        for robot_type in self.type_num.keys():
            for num in range(self.type_num[robot_type]):
                while True:
                    candidate = random.sample(x0, 1)[0]
                    if candidate not in type_robot_location.values():
                        type_robot_location[(robot_type, num)] = candidate
                        x0.remove(candidate)
                        break
        return type_robot_location

    def allocate_region_dars(self):
        # (x, y) --> (y-1, 30-x)
        # dars
        # regions = []
        # regions.append(list(itertools.product(range(0, 6), range(0, 17))))  # x0 l1
        # regions.append(list(itertools.product(range(0, 11), range(19, 30))))  # b1 l2
        # regions.append(list(itertools.product(range(18, 30), range(16, 30))))  # b2 l3
        # regions.append(list(itertools.product(range(18, 30), range(0, 14))))  # b3 l4
        # regions.append(list(itertools.product(range(16, 18), range(0, 3))))  # g1 l5
        # regions.append(list(itertools.product(range(18, 21), range(14, 16))))  # g1 l6

        # small regions
        # regions = []
        # regions.append(list(itertools.product(range(0, 5), range(0, 5))))  # x0
        # regions.append(list(itertools.product(range(0, 5), range(25, 30))))  # b1
        # regions.append(list(itertools.product(range(25, 30), range(25, 30))))  # b2
        # regions.append(list(itertools.product(range(25, 30), range(0, 5))))  # b3

        # # small workspace
        regions = []
        regions.append(list(itertools.product(range(0, 2), range(0, 5))))  # x0 l1
        regions.append(list(itertools.product(range(0, 3), range(8, 10))))  # b1 l2
        regions.append(list(itertools.product(range(8, 10), range(7, 10))))  # b2 l3
        regions.append(list(itertools.product(range(8, 10), range(0, 2))))  # b3 l4
        regions.append(list(itertools.product(range(5, 6), range(0, 2))))  # g1 l5
        regions.append(list(itertools.product(range(6, 8), range(4, 5))))  # g2 l6

        return regions

    def allocate_obstacle_dars(self):
        # 30 by 30
        # obstacles = []
        # obstacles.append(list(itertools.product(range(0, 8), range(17, 19))))  # o1
        # obstacles.append(list(itertools.product(range(6, 8), range(0, 12))))
        # obstacles.append(list(itertools.product(range(11, 13), range(18, 30))))  # o2
        # obstacles.append(list(itertools.product(range(16, 18), range(3, 27))))  # o3
        # obstacles.append(list(itertools.product(range(21, 30), range(14, 16))))  # o4

        # small workspace
        obstacles = []
        obstacles.append(list(itertools.product(range(0, 2), range(5, 6))))  # o1
        # obstacles.append(list(itertools.product(range(2, 2), range(0, 4))))  #
        obstacles.append(list(itertools.product(range(3, 4), range(7, 10))))  # o2
        obstacles.append(list(itertools.product(range(5, 6), range(2, 8))))  # o3
        obstacles.append(list(itertools.product(range(8, 10), range(4, 5))))  # o4

        return obstacles

    # def allocate_region_dars(self):
    #     # (x, y) --> (y-1, 30-x)
    #     # ijrr
    #     # # small workspace
    #     regions = []
    #     regions.append(list(itertools.product(range(5, 8), range(0, 2))))  # x0 l1
    #     regions.append(list(itertools.product(range(6, 8), range(5, 8))) + [(7,4)])  # b1 l2
    #     regions.append(list(itertools.product(range(0, 2), range(0, 4))))  # b2 l3
    #     regions.append(list(itertools.product(range(0, 3), range(6, 7))))  # b3 l4
    #     return regions
    #
    # def allocate_obstacle_dars(self):
    #
    #     # small workspace
    #     obstacles = []
    #     # obstacles.append(list(itertools.product(range(3, 4), range(2, 6))) + [(4, 2)])  # o1
    #     obstacles.append(list(itertools.product(range(3, 4), range(0, 6))))  # o1
    #
    #     return obstacles
    #
    # def initialize(self):
    #     type_robot_location = {(1, 0): (6, 0), (1, 1): (6, 1), (1, 2): (7, 1),
    #                            (2, 0): (5, 0), (2, 1): (5, 1)}
    #     return type_robot_location

    def update_after_prefix(self, loop=False):
        # region and corresponding locations
        self.label_location = {'r{0}'.format(i + 1): j for i, j in enumerate(list(self.type_robot_location.values()))}
        # if robots return to their initial locations
        self.regions.update({label: [region] for label, region in self.label_location.items()})

        # region where robots reside
        self.type_robot_label = dict(zip(self.type_robot_location.keys(), self.label_location.keys()))
        # atomic proposition
        self.atomic_prop = self.get_atomic_prop()

        key_region = list(self.regions.keys())
        key_init = list(self.label_location.keys())

        for r1 in range(len(self.label_location)):
            for l1 in range(len(self.regions)):
                min_length = np.inf
                for target in self.regions[key_region[l1]]:
                    length, _ = nx.algorithms.single_source_dijkstra(self.graph_workspace,
                                                                     source=self.label_location[key_init[r1]],
                                                                     target=target)
                    if length < min_length:
                        min_length = length
                self.p2p[(key_init[r1], key_region[l1])] = min_length
                self.p2p[(key_region[l1], key_init[r1])] = min_length

        # robots return to their initial locations
        if loop:
            for r1 in range(len(self.label_location)):
                for r2 in range(r1, len(self.label_location)):
                    length, path = nx.algorithms.single_source_dijkstra(self.graph_workspace,
                                                                        source=self.label_location[key_init[r1]],
                                                                        target=self.label_location[key_init[r2]])
                    self.p2p[(key_init[r1], key_init[r2])] = length
                    self.p2p[(key_init[r2], key_init[r1])] = length

    def longest_time(self, init, target):
        """
        the longest time to return to initial locations
        """
        horizon = 0
        for robot in init.keys():
            length, _ = nx.algorithms.single_source_dijkstra(self.graph_workspace,
                                                             source=init[robot],
                                                             target=target[robot])
            if length > horizon:
                horizon = length
        return horizon
