# -*- coding: utf-8 -*-

from random import randint
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import numpy as np
import random
import sys


class Workspace(object):
    """
    define the workspace where robots reside
    """
    def __init__(self):
        # dimension of the workspace
        # self.length = int(sys.argv[1])
        # self.width = int(sys.argv[1])
        # n = int(sys.argv[2])
        self.length = 10     # length
        self.width = 10      # width
        self.n = 5  # nt(sys.argv[1])
        self.type_num = {1: self.n, 2: self.n, 3: self.n, 4: self.n, 5: self.n}   # single-task robot
        # self.type_num = {1: self.n, 2: self.n}   # single-task robot

        self.workspace = (self.length, self.width)
        self.num_of_regions = 10
        self.num_of_obstacles = 10
        self.occupied = []
        self.regions = {'l{0}'.format(i+1): j for i, j in enumerate(self.allocate(self.num_of_regions))}
        self.obstacles = {'o{0}'.format(i+1): j for i, j in enumerate(self.allocate(self.num_of_obstacles))}
        self.type_robot_location = self.initialize()
        # region and corresponding locations
        self.label_location = {'r{0}'.format(i + 1): j for i, j in enumerate(list(self.type_robot_location.values()))}
        # region where robots reside
        self.type_robot_label = dict(zip(self.type_robot_location.keys(), self.label_location.keys()))
        # self.regions = {'l1': (1, 0), 'l2': (6, 3), 'l3': (2, 5)}
        # self.obstacles = {'o1': (0, 5), 'o2': (0, 0), 'o3': (4, 2)}
        # self.type_robot_location = {(1, 0): (6, 8), (1, 1): (3, 1), (2, 0): (4, 8)}
        # self.type_robot_label = {(1, 0): 'r1', (1, 1): 'r2', (2, 0): 'r3'}
        # self.label_location = {'r1': (6, 8), 'r2': (3, 1), 'r3': (4, 8)}

        self.graph_workspace = nx.Graph()
        self.build_graph()

        self.p2p, self.p2p_path = self.point_to_point_path()  # label2label path

    def initialize(self):
        type_robot_location = dict()
        obstacles = list(self.obstacles.values())
        for robot_type in self.type_num.keys():
            for num in range(self.type_num[robot_type]):
                while True:
                    candidate = (randint(0, self.width-1), randint(0, self.length-1))
                    if candidate not in obstacles:
                        type_robot_location[(robot_type, num)] = candidate
                        break
        return type_robot_location

    def allocate(self, num):
        obj = []
        for i in range(num):
            while True:
                candidate = (randint(0, self.width-1), randint(0, self.length-1))
                if candidate in self.occupied:
                    continue
                else:
                    obj.append(candidate)
                    break
            self.occupied.append(candidate)
        return obj

    def reachable(self, location):
        next_location = []
        obstacles = list(self.obstacles.values())
        # left
        if location[0]-1 >= 0 and (location[0]-1, location[1]) not in obstacles:
            next_location.append((location, (location[0]-1, location[1])))
        # right
        if location[0]+1 < self.width and (location[0]+1, location[1]) not in obstacles:
            next_location.append((location, (location[0]+1, location[1])))
        # up
        if location[1]+1 < self.length and (location[0], location[1]+1) not in obstacles:
            next_location.append((location, (location[0], location[1]+1)))
        # down
        if location[1]-1 >= 0 and (location[0], location[1]-1) not in obstacles:
            next_location.append((location, (location[0], location[1]-1)))
        return next_location

    def build_graph(self):
        obstacles = list(self.obstacles.values())
        for i in range(self.width):
            for j in range(self.length):
                if (i, j) not in obstacles:
                    self.graph_workspace.add_edges_from(self.reachable((i, j)))

    def point_to_point_path(self):
        p2p = dict()
        p2p_path = dict()
        key_region = list(self.regions.keys())
        key_init = list(self.label_location.keys())
        for l1 in range(len(self.regions)):
            for l2 in range(l1, len(self.regions)):
                length, path = nx.algorithms.single_source_dijkstra(self.graph_workspace,
                                                                    source=self.regions[key_region[l1]],
                                                                    target=self.regions[key_region[l2]])
                p2p[(key_region[l1], key_region[l2])] = length
                p2p_path[(key_region[l1], key_region[l2])] = path
                p2p[(key_region[l2], key_region[l1])] = length
                p2p_path[(key_region[l2], key_region[l1])] = path[::-1]

        for r1 in range(len(self.label_location)):
            for l1 in range(len(self.regions)):
                length, path = nx.algorithms.single_source_dijkstra(self.graph_workspace,
                                                                    source=self.label_location[key_init[r1]],
                                                                    target=self.regions[key_region[l1]])
                p2p[(key_init[r1], key_region[l1])] = length
                p2p_path[(key_init[r1], key_region[l1])] = path
                p2p[(key_region[l1], key_init[r1])] = length

        for r1 in range(len(self.label_location)):
            for r2 in range(r1, len(self.label_location)):
                length, path = nx.algorithms.single_source_dijkstra(self.graph_workspace,
                                                                    source=self.label_location[key_init[r1]],
                                                                    target=self.label_location[key_init[r2]])
                p2p[(key_init[r1], key_init[r2])] = length
                p2p_path[(key_init[r1], key_init[r2])] = path
                p2p[(key_init[r2], key_init[r1])] = length
                p2p_path[(key_init[r2], key_init[r1])] = path

        return p2p, p2p_path

    def plot_workspace(self):
        ax = plt.figure(1).gca()
        ax.set_xlim((0, self.width))
        ax.set_ylim((0, self.length))
        plt.xticks(np.arange(0, self.width + 1, 1.0))
        plt.yticks(np.arange(0, self.length + 1, 1.0))
        self.plot_workspace_helper(ax, self.regions, 'region')
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
            color = '0.75' if obj_label != 'region' else 'c'
            x = []
            y = []
            x_ = obj[key][0]
            y_ = obj[key][1]
            patches = []
            for point in [(x_, y_), (x_+1, y_), (x_+1, y_+1), (x_, y_+1)]:
                x.append(point[0])
                y.append(point[1])
            polygon = Polygon(np.column_stack((x, y)), True)
            patches.append(polygon)
            p = PatchCollection(patches, facecolors=color, edgecolors=color)
            ax.add_collection(p)
            ax.text(np.mean(x)-0.2, np.mean(y)-0.2, r'${}_{{{}}}$'.format(key[0], key[1:]), fontsize=16)

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