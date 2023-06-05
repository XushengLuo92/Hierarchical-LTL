import networkx as nx
from gurobipy import *
from workspace_dars import Workspace
import matplotlib.pyplot as plt
import itertools
from vis import vis
import datetime
import numpy as np


def multi_agent_path_planning(workspace, T, robot_team_initial_target, neg_symbols_edge, neg_symbols_vertex):
    # build the time_expanded graph
    time_expanded_graph = nx.DiGraph()
    # s = datetime.datetime.now()
    loc2node, gadget, robot_index, edge_index_dict, index_edge_dict = build_time_expanded_graph(
        workspace, T, robot_team_initial_target, time_expanded_graph)
    # print((datetime.datetime.now() - s).total_seconds())
    # ILP formulation
    m = Model()
    return ILP(m, time_expanded_graph, robot_team_initial_target, loc2node, gadget, robot_index, edge_index_dict,
               index_edge_dict, neg_symbols_vertex, neg_symbols_edge, workspace, T)


def build_time_expanded_graph(workspace, T, robot_team_initial_target, time_expanded_graph):
    # add nodes from G
    loc2node = dict()
    node_index = 0
    for loc in workspace.graph_workspace.nodes():
        time_expanded_graph.add_nodes_from(range(node_index, node_index + T + 1), loc=loc)
        loc2node[loc] = range(node_index, node_index + T + 1)
        node_index = node_index + T + 1

    edge_index = []
    # index the robot set
    robot_index = list(robot_team_initial_target.keys())
    gadget = []
    for u, v in workspace.graph_workspace.edges():
        for t in range(0, T):
            edge = [(loc2node[u][t], loc2node[v][t+1]), (loc2node[v][t], loc2node[u][t + 1])]
            time_expanded_graph.add_edges_from(edge, cost=1)
            edge_index += edge
            gadget.append(edge)
    # green edges
    for loc in loc2node.keys():
        time_expanded_graph.add_edges_from([(loc2node[loc][i], loc2node[loc][i + 1]) for i in range(T)], cost=0)
        edge_index += [(loc2node[loc][i], loc2node[loc][i + 1]) for i in range(T)]
    # index the edge set
    edge_index_dict = {edge: index for index, edge in enumerate(edge_index)}
    index_edge_dict = {index: edge for index, edge in enumerate(edge_index)}

    return loc2node, gadget, robot_index, edge_index_dict, index_edge_dict


def ILP(m, time_expanded_graph, robot_team_initial_target, loc2node, gadget, robot_index, edge_index_dict, index_edge_dict,
        neg_symbols_vertex, neg_symbols_edge, workspace, T):
    # create variables
    x_vars = {}
    x_vars.update(m.addVars(list(range(len(robot_index))), list(range(time_expanded_graph.number_of_edges())),
                            vtype=GRB.BINARY))
    m.update()

    # (8)
    for j in range(time_expanded_graph.number_of_edges()):
        m.addConstr(quicksum(x_vars[(i, j)] for i in range(len(robot_index))) <= 1)
    # (9) and (11)
    # initial location (source)
    for i, robot in enumerate(robot_index):
        initial_node = loc2node[robot_team_initial_target[robot][0]][0]
        m.addConstr(quicksum(x_vars[i, edge_index_dict[j]] for j in time_expanded_graph.edges(initial_node)) == 1)
        initial_nodes = [nodes[0] for nodes in loc2node.values()]
        initial_nodes.remove(initial_node)
        m.addConstr(quicksum(x_vars[i, edge_index_dict[j]] for node in initial_nodes
                             for j in time_expanded_graph.edges(node)) == 0)

    # target location (sink)
    for i, robot in enumerate(robot_index):
        try:
            # target may be occupied by a robot
            target_node = [loc2node[loc][-1] for loc in workspace.regions[robot_team_initial_target[robot][1]]
                           if loc in loc2node.keys()]
        # target is not a region
        except KeyError:
            target_node = [loc2node[robot_team_initial_target[robot][1]][-1]]

        m.addConstr(quicksum(x_vars[i, edge_index_dict[(j, t)]] for t in target_node
                             for j in time_expanded_graph.predecessors(t)) == 1)

    # (10)
    # exclude the initial and terminal nodes
    exclude = [nodes[0] for nodes in loc2node.values()] + [nodes[-1] for nodes in loc2node.values()]
    for node in time_expanded_graph.nodes():
        if node not in exclude:
            for i in range(len(robot_index)):
                m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, node)])] for p in time_expanded_graph.predecessors(node))
                            == quicksum(x_vars[(i, edge_index_dict[(node, s)])] for s in time_expanded_graph.successors(node)))

    # (17) head-on collision constraint for a single gadget
    for edge_pair in gadget:
        m.addConstr(
            quicksum(x_vars[i, edge_index_dict[edge]] for i in range(len(robot_index)) for edge in edge_pair) <= 1)

    # (18) the meet collision constraint
    for node in time_expanded_graph.nodes():
        m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, node)])] for i in range(len(robot_index))
                             for p in time_expanded_graph.predecessors(node)) <= 1)
    # avoid negative literals in vertex label
    for symbol in neg_symbols_vertex:
        index_robot = [i for i, r in enumerate(robot_index) if r[0] == symbol[1]]
        for t in range(1, T):
            m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, loc2node[loc][t])])] for i in index_robot
                                 for loc in workspace.regions[symbol[0]]
                                 for p in time_expanded_graph.predecessors(loc2node[loc][t])) <= symbol[2]-1)
    # avoid negative literals in edge label
    for symbol in neg_symbols_edge:
        index_robot = [i for i, r in enumerate(robot_index) if r[0] == symbol[1]]
        m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, loc2node[loc][T])])] for i in index_robot
                             for loc in workspace.regions[symbol[0]]
                             for p in time_expanded_graph.predecessors(loc2node[loc][T])) <= symbol[2]-1)

    m.update()
    # expr = LinExpr([1]*len(robot_index), [x_vars[(i, i)] for i in range(len(robot_index))])
    expr = LinExpr([time_expanded_graph.edges[index_edge_dict[index[1]]]['cost'] for index in x_vars.keys()],
                   list(x_vars.values()))

    m.setObjective(expr, GRB.MINIMIZE)
    m.update()
    m.Params.OutputFlag = 0

    m.optimize()
    if m.status == GRB.Status.OPTIMAL:
        # print('Optimal objective: %g' % m.objVal)
        pass
    elif m.status == GRB.Status.INF_OR_UNBD:
        print('Model is infeasible or unbounded')
        return None
    elif m.status == GRB.Status.INFEASIBLE:
        print('Model is infeasible')
        return None
    elif m.status == GRB.Status.UNBOUNDED:
        print('Model is unbounded')
        return None
    else:
        print('Optimization ended with status %d' % m.status)
        exit(0)

    paths = extract_paths(x_vars, time_expanded_graph, robot_team_initial_target, loc2node, robot_index, index_edge_dict,
                          edge_index_dict, workspace)
    return paths


def extract_paths(x_vars, time_expanded_graph, robot_team_initial_target, loc2node, robot_index, index_edge_dict,
                  edge_index_dict, workspace):
    # print("extracting paths")
    paths = {robot: [] for robot in robot_index}
    for index, robot in enumerate(robot_index):
        # the initial location
        node = loc2node[robot_team_initial_target[robot][0]][0]
        try:
            target_node = [loc2node[loc][-1] for loc in workspace.regions[robot_team_initial_target[robot][1]]
                           if loc in loc2node.keys()]
        except KeyError:
            target_node = [loc2node[robot_team_initial_target[robot][1]][-1]]
        while node not in target_node:
            # the location that node corresponds to
            paths[robot].append(time_expanded_graph.nodes[node]['loc'])
            # find the next transition
            for edge in time_expanded_graph.edges(node):
                if x_vars[(index, edge_index_dict[edge])].x == 1:
                    node = edge[1]
                    break
        # add the terminal location
        paths[robot].append(time_expanded_graph.nodes[node]['loc'])

    return paths


def mapp(workspace, acpt_run, robot_waypoint, robot_time, order):
    """
    Generalized multi-robot path planning
    """
    start = datetime.datetime.now()
    if order == 'sequential':
        robot_path = {type_robot: [location] for type_robot, location in workspace.type_robot_location.items()}

        for index, clock in enumerate(acpt_run):
            # initial and target locations for some robots
            robot_team_initial_target = {robot: (robot_path[robot][-1], target) for target, robots in clock[-1].items()
                                         for robot in robots}
            # starting horizon
            if index != 0:
                horizon = clock[2] - acpt_run[index-1][2]
            else:
                horizon = clock[2]
            for T in range(horizon+2, horizon + 100, 4):
                paths = multi_agent_path_planning(workspace, T, robot_team_initial_target, acpt_run[clock][3],
                                                       acpt_run[clock][4])
                if paths:
                    # update
                    for robot, path in paths.items():
                        robot_path[robot] += path[1:]
                    for robot in robot_path.keys():
                        if robot not in paths.keys():
                            robot_path[robot] += [robot_path[robot][-1]]*T
                    break

        end = datetime.datetime.now()
        print((end - start).total_seconds())

        vis(workspace, robot_path, {robot: [len(path)] * 2 for robot, path in robot_path.items()}, [])
    elif order == 'simultaneous':
        robot_path = {type_robot: [location] for type_robot, location in workspace.type_robot_location.items()}
        robot_progress = {robot: -1 for robot in robot_waypoint.keys()}
        clock = 0
        overall_progress = 0
        while clock < len(acpt_run):
            print(acpt_run[clock])
            # initial and target locations of robots that execute the subtask
            # acpt_run[clock] = ([1, succ, time_point[step], exe_robot_edge, neg_literal_edge,
            #                   vertex_period, exe_robot_vertex, neg_literal_vertex])

            next_progress = acpt_run[clock][2]
            # edge
            robot_team_initial_target = {robot: (robot_path[robot][-1], target) for target, robots in
                                         acpt_run[clock][3].items() for robot in robots}
            # vertex, the label of the reaching vertex
            if 'accept' not in acpt_run[clock][1]:
                robot_team_initial_target.update({robot: (robot_path[robot][-1], target) for target, robots in
                                                  acpt_run[clock+1][6].items() for robot in robots})
            # update robot_team_initial_target by considering simultaneity
            remove_edge = update(workspace, robot_team_initial_target, robot_waypoint, robot_time, robot_path,
                                 robot_progress, next_progress)
            # # treat all robots that do not execute the current subtask as obstacles
            # remove_edge = dict()
            # for robot, path in robot_path.items():
            #     if robot not in robot_team_initial_target.keys():
            #         remove_edge[robot] = list(workspace.graph_workspace.edges(path[-1]))
            #         workspace.graph_workspace.remove_node(path[-1])

            # find the collision-avoidance paths for involved robots
            horizon = next_progress - overall_progress
            for T in range(horizon, horizon + 100, 2):
                mapp_paths = multi_agent_path_planning(workspace, T, robot_team_initial_target, acpt_run[clock][4],
                                                       acpt_run[clock][7])
                if mapp_paths:
                    # update
                    for robot, path in mapp_paths.items():
                        robot_path[robot] += path[1:]
                    for robot in robot_path.keys():
                        if robot not in mapp_paths.keys():
                            robot_path[robot] += [robot_path[robot][-1]] * T
                    break

            # update key time points for each robot
            for robot, time in robot_time.items():
                if robot_progress[robot] + 1 < len(robot_time[robot]):
                    robot_time[robot][robot_progress[robot]+1:] = [t+T-horizon
                                                                   for t in robot_time[robot][robot_progress[robot]+1:]]

            # update the overall progress
            overall_progress += T
            # update the individual progress
            for robots in acpt_run[clock][3].values():
                for robot in robots:
                    robot_progress[robot] += 1
            # update the overall plan
            for subseq_clock in range(clock, len(acpt_run)):
                acpt_run[subseq_clock][2] = acpt_run[subseq_clock][2] + T - horizon
            # restore removed edges
            workspace.graph_workspace.add_edges_from(itertools.chain(remove_edge))
            # plt.show()
            clock += 1

        vis(workspace, robot_path, {robot: [len(path)] * 2 for robot, path in robot_path.items()}, [])


def update(workspace, robot_team_initial_target, robot_waypoint, robot_time, robot_path, robot_progress, next_progress):
    # find robots that need to move
    robot_future_time_min_length_path_target = dict()
    for robot in robot_progress.keys():
        if robot not in robot_team_initial_target.keys():
            # time difference between the next progress (edge) and the key time point for the considered robot
            if robot_progress[robot] + 1 < len(robot_time[robot]):
                future_time = robot_time[robot][robot_progress[robot]+1] - next_progress
            else:
                continue
            # calculate the shortest path from the current location to assigned region
            min_length = np.inf
            path = []
            min_target = None
            for target in workspace.regions[robot_waypoint[robot][robot_progress[robot] + 1]]:
                length, p = nx.algorithms.single_source_dijkstra(workspace.graph_workspace,
                                                                 source=robot_path[robot][-1],
                                                                 target=target)
                if min_length > length:
                    min_length = length
                    path = p
                    min_target = target
            if future_time < min_length:
                robot_future_time_min_length_path_target[robot] = [future_time, min_length, path, min_target]
    # treat robots that do not move as obstacles
    assigned_target = [path[-1] for robot, path in robot_path.items() if robot not in robot_team_initial_target.keys()
                       and robot not in robot_future_time_min_length_path_target.keys()]
    # treat all robots that do not execute the current subtask as obstacles
    remove_edge = []
    for obs in assigned_target:
        remove_edge += list(workspace.graph_workspace.edges(obs))
        workspace.graph_workspace.remove_node(obs)

    for robot in robot_future_time_min_length_path_target.keys():
        # determine the target point at the next progress (time)
        future_time, min_length, path, min_target = robot_future_time_min_length_path_target[robot]
        target = path[min_length-future_time]
        # find the intermediate target for mapp
        if target in assigned_target:
            l, p = nx.algorithms.single_source_dijkstra(workspace.graph_workspace,
                                                        source=robot_path[robot][-1],
                                                        target=min_target)
            # if target is already selected, select previous one
            index = l - 1 - future_time
            while target in assigned_target:
                target = p[index]
                try:
                    index -= 1
                except KeyError:
                    break

        if target != robot_path[robot][-1]:
            assigned_target.append(target)
            robot_team_initial_target[robot] = (robot_path[robot][-1], target)

    return remove_edge

# def update(workspace, robot_team_initial_target, robot_waypoint, robot_time, robot_path, robot_progress, next_progress):
#     # overestimate, suppose all robots are fixed (obstacles)
#     assigned_target = []  # [path[-1] for robot, path in robot_path.items() if robot not in robot_team_initial_target.keys()]
#     for robot in robot_progress.keys():
#         if robot not in robot_team_initial_target.keys():
#             # time difference between the next progress (edge) and the key time point for the considered robot
#             if robot_progress[robot] + 1 < len(robot_time[robot]):
#                 future_time = robot_time[robot][robot_progress[robot]+1] - next_progress
#             else:
#                 continue
#             # calculate the shortest path from the current location to assigned region
#             min_length = np.inf
#             path = []
#             min_target = None
#             for target in workspace.regions[robot_waypoint[robot][robot_progress[robot] + 1]]:
#                 length, p = nx.algorithms.single_source_dijkstra(workspace.graph_workspace,
#                                                                  source=robot_path[robot][-1],
#                                                                  target=target)
#                 if min_length > length:
#                     min_length = length
#                     path = p
#                     min_target = target
#
#             # determine the target point at the next progress (time)
#             if future_time < min_length:
#                 target = path[min_length-future_time]
#                 remove_edge = []
#                 while target in assigned_target:
#                     remove_edge += workspace.graph_workspace.edges(target)
#                     workspace.graph_workspace.remove_node(target)
#                     min_length, path = nx.algorithms.single_source_dijkstra(workspace.graph_workspace,
#                                                                      source=robot_path[robot][-1],
#                                                                      target=min_target)
#                     target = path[min_length - 1 - future_time]
#                 assigned_target.append(target)
#                 workspace.graph_workspace.add_edges_from(remove_edge)
#
#                 robot_team_initial_target[robot] = (robot_path[robot][-1], target)