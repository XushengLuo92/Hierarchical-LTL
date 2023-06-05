import networkx as nx
from gurobipy import *
from workspace_dars import Workspace
import matplotlib.pyplot as plt
import itertools
from vis import vis
import datetime


# def multi_agent_path_planning(workspace, T, robot_team_initial_target):
#     # build the time_expanded graph
#     time_expanded_graph = nx.DiGraph()
#     loc2node, loc_pair2node, robot_index, edge_index_dict, index_edge_dict, robot_edge = build_time_expanded_graph(
#         workspace, T, robot_team_initial_target, time_expanded_graph)
#     # ILP formulation
#     m = Model()
#     return ILP(m, time_expanded_graph, robot_index, edge_index_dict, index_edge_dict, robot_edge)
#
#
# def build_time_expanded_graph(workspace, T, robot_team_initial_target, time_expanded_graph):
#     # add nodes from G
#     loc2node = dict()
#     node_index = 0
#     for loc in workspace.graph_workspace.nodes():
#         time_expanded_graph.add_nodes_from(range(node_index, node_index + 2 * T + 1, 2), loc=loc, id=True)
#         # id help extract paths, corresponding to 1', 2' in fig. 5
#         time_expanded_graph.add_nodes_from(range(node_index+1, node_index + 2 * T + 1, 2), loc=loc, id=False)
#         loc2node[loc] = range(node_index, node_index + 2 * T + 1)
#         node_index = node_index + 2 * T + 1
#
#     edge_index = []
#     # index the robot set
#     robot_index = list(robot_team_initial_target.keys())
#     robot_edge = {}
#     # add loopback arcs
#     for robot, initial_target in robot_team_initial_target.items():
#         initial = initial_target[0]
#         target = initial_target[1]
#         time_expanded_graph.add_edges_from([(loc2node[l][-1], loc2node[initial][0])
#                                             for l in workspace.regions[target]], cost=0)
#         edge_index += [(loc2node[l][-1], loc2node[initial][0]) for l in workspace.regions[target]]
#         # robot: the set of edges connecting target and initial locs
#         robot_edge[robot] = [(loc2node[l][-1], loc2node[initial][0]) for l in workspace.regions[target]]
#
#     # add nodes of gadgets
#     loc_pair2node = dict()
#     node_index = time_expanded_graph.number_of_nodes()
#     for u, v in workspace.graph_workspace.edges():
#         for t in range(0, 2 * T, 2):
#             time_expanded_graph.add_nodes_from(range(node_index, node_index + 2), id=False)
#             edge = [(loc2node[u][t], node_index), (loc2node[v][t], node_index),
#                     (node_index + 1, loc2node[u][t + 1]), (node_index + 1, loc2node[v][t + 1])]
#             time_expanded_graph.add_edges_from(edge, cost=0)
#             edge_index += edge
#             edge = [(node_index, node_index + 1)]
#             time_expanded_graph.add_edges_from(edge, cost=1)
#             edge_index += edge
#             loc_pair2node[(u, v)] = [node_index, node_index + 1]
#             node_index += 2
#
#     # green and blue edges
#     for loc in loc2node.keys():
#         time_expanded_graph.add_edges_from([(loc2node[loc][i], loc2node[loc][i + 1]) for i in range(2 * T)], cost=0)
#         edge_index += [(loc2node[loc][i], loc2node[loc][i + 1]) for i in range(2 * T)]
#     # index the edge set
#     edge_index_dict = {edge: index for index, edge in enumerate(edge_index)}
#     index_edge_dict = {index: edge for index, edge in enumerate(edge_index)}
#     # index the robot: edge
#     for robot, edges in robot_edge.items():
#         robot_edge[robot] = [edge_index_dict[e] for e in edges]
#     return loc2node, loc_pair2node, robot_index, edge_index_dict, index_edge_dict, robot_edge
#
#
# def ILP(m, time_expanded_graph, robot_index, edge_index_dict, index_edge_dict, robot_edge):
#     # create variables
#     x_vars = {}
#     x_vars.update(m.addVars(list(range(len(robot_index))), list(range(time_expanded_graph.number_of_edges())),
#                             vtype=GRB.BINARY))
#     m.update()
#
#     # (8)
#     for j in range(time_expanded_graph.number_of_edges()):
#         m.addConstr(quicksum(x_vars[(i, j)] for i in range(len(robot_index))) <= 1)
#     # (9) and (11)
#     for i in range(len(robot_index)):
#         m.addConstr(quicksum(x_vars[i, j] for j in robot_edge[robot_index[i]]) == 1)
#
#         # for j in
#         #     if j != i:
#         #         m.addConstr(x_vars[(i, j)] == 0)
#         #     else:
#         #         m.addConstr(x_vars[(i, j)] == 1)
#
#     # (10)
#     for node in time_expanded_graph.nodes():
#         for i in range(len(robot_index)):
#             m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, node)])] for p in time_expanded_graph.predecessors(node))
#                         == quicksum(x_vars[(i, edge_index_dict[(node, s)])] for s in time_expanded_graph.successors(node)))
#     m.update()
#     # expr = LinExpr([1]*len(robot_index), [x_vars[(i, i)] for i in range(len(robot_index))])
#     expr = LinExpr([time_expanded_graph.edges[index_edge_dict[index[1]]]['cost'] for index in x_vars.keys()],
#                    list(x_vars.values()))
#
#     m.setObjective(expr, GRB.MINIMIZE)
#     m.update()
#     m.Params.OutputFlag = 0
#
#     m.optimize()
#     if m.status == GRB.Status.OPTIMAL:
#         # print('Optimal objective: %g' % m.objVal)
#         pass
#     elif m.status == GRB.Status.INF_OR_UNBD:
#         print('Model is infeasible or unbounded')
#         return None
#     elif m.status == GRB.Status.INFEASIBLE:
#         print('Model is infeasible')
#         return None
#     elif m.status == GRB.Status.UNBOUNDED:
#         print('Model is unbounded')
#         return None
#     else:
#         print('Optimization ended with status %d' % m.status)
#         exit(0)
#     obj = sum([x_vars[(i, j)].x for i in range(len(robot_index)) for j in robot_edge[robot_index[i]]])
#     if obj == len(robot_index):
#         paths = extract_paths(x_vars, time_expanded_graph, robot_index, index_edge_dict, edge_index_dict, robot_edge)
#         return paths
#
#     return None
#
#
# def extract_paths(x_vars, time_expanded_graph, robot_index, index_edge_dict, edge_index_dict, robot_edge):
#     # print("extracting paths")
#     paths = {robot: [] for robot in robot_index}
#     for index, robot in enumerate(robot_index):
#         # the initial location, from loopback
#         node = index_edge_dict[robot_edge[robot][0]][1]
#         # terminal location
#         for e in robot_edge[robot]:
#             if x_vars[(index, e)].x == 1:
#                 terminal = index_edge_dict[e][0]
#         while node != terminal:
#             # the location that node corresponds to
#             if time_expanded_graph.nodes[node]['id']:
#                 if time_expanded_graph.nodes[node]['loc'] == (7, 8):
#                     print('d')
#                 paths[robot].append(time_expanded_graph.nodes[node]['loc'])
#             # find the next transition
#             for edge in time_expanded_graph.edges(node):
#                 if x_vars[(index, edge_index_dict[edge])].x == 1:
#                     node = edge[1]
#                     break
#         # add the terminal location
#         paths[robot].append(time_expanded_graph.nodes[terminal]['loc'])
#
#     return paths

def multi_agent_path_planning(workspace, T, robot_team_initial_target, neg_symbols):
    # build the time_expanded graph
    time_expanded_graph = nx.DiGraph()
    s = datetime.datetime.now()
    loc2node, loc_pair2node, robot_index, edge_index_dict, index_edge_dict = build_time_expanded_graph(
        workspace, T, robot_team_initial_target, time_expanded_graph)
    print((datetime.datetime.now() - s).total_seconds())
    # ILP formulation
    m = Model()
    return ILP(m, time_expanded_graph, robot_team_initial_target, loc2node, robot_index, edge_index_dict,
               index_edge_dict, neg_symbols, workspace)


def build_time_expanded_graph(workspace, T, robot_team_initial_target, time_expanded_graph):
    # add nodes from G
    loc2node = dict()
    node_index = 0
    for loc in workspace.graph_workspace.nodes():
        time_expanded_graph.add_nodes_from(range(node_index, node_index + 2 * T + 1, 2), loc=loc, id=True)
        # id help extract paths, corresponding to 1', 2' in fig. 5
        time_expanded_graph.add_nodes_from(range(node_index+1, node_index + 2 * T + 1, 2), loc=loc, id=False)
        loc2node[loc] = range(node_index, node_index + 2 * T + 1)
        node_index = node_index + 2 * T + 1

    edge_index = []
    # index the robot set
    robot_index = list(robot_team_initial_target.keys())
    # add nodes of gadgets
    loc_pair2node = dict()
    node_index = time_expanded_graph.number_of_nodes()
    for u, v in workspace.graph_workspace.edges():
        for t in range(0, 2 * T, 2):
            time_expanded_graph.add_nodes_from(range(node_index, node_index + 2), id=False)
            edge = [(loc2node[u][t], node_index), (loc2node[v][t], node_index),
                    (node_index + 1, loc2node[u][t + 1]), (node_index + 1, loc2node[v][t + 1])]
            time_expanded_graph.add_edges_from(edge, cost=0)
            edge_index += edge
            edge = [(node_index, node_index + 1)]
            time_expanded_graph.add_edges_from(edge, cost=1)
            edge_index += edge
            loc_pair2node[(u, v)] = [node_index, node_index + 1]
            node_index += 2

    # green and blue edges
    for loc in loc2node.keys():
        time_expanded_graph.add_edges_from([(loc2node[loc][i], loc2node[loc][i + 1]) for i in range(2 * T)], cost=0)
        edge_index += [(loc2node[loc][i], loc2node[loc][i + 1]) for i in range(2 * T)]
    # index the edge set
    edge_index_dict = {edge: index for index, edge in enumerate(edge_index)}
    index_edge_dict = {index: edge for index, edge in enumerate(edge_index)}

    return loc2node, loc_pair2node, robot_index, edge_index_dict, index_edge_dict


def ILP(m, time_expanded_graph, robot_team_initial_target, loc2node, robot_index, edge_index_dict, index_edge_dict,
        neg_symbols, workspace):
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
        target_node = [loc2node[loc][-1] for loc in workspace.regions[robot_team_initial_target[robot][1]]]
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

    # avoid negative literals
    for symbol in neg_symbols:
        index_robot = [i for i, r in enumerate(robot_index) if r[0] == symbol[1]]
        for t in range(2, 2*T+1, 2):
            m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, loc2node[loc][t])])] for i in index_robot
                                 for loc in workspace.regions[symbol[0]]
                                 for p in time_expanded_graph.predecessors(loc2node[loc][t])) <= symbol[2]-1)

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
        target_node = [loc2node[loc][-1] for loc in workspace.regions[robot_team_initial_target[robot][1]]]
        while node not in target_node:
            # the location that node corresponds to
            if time_expanded_graph.nodes[node]['id']:
                paths[robot].append(time_expanded_graph.nodes[node]['loc'])
            # find the next transition
            for edge in time_expanded_graph.edges(node):
                if x_vars[(index, edge_index_dict[edge])].x == 1:
                    node = edge[1]
                    break
        # add the terminal location
        paths[robot].append(time_expanded_graph.nodes[node]['loc'])

    return paths


def mapp(workspace, acpt_run):

#   acpt_run = (1, 'T1_S5', 1, 24, [], {'l2': [(1, 0), (1, 1), (1, 2), (1, 3)]})
#                (1, 'T2_S7', 2, 45, [], {'l3': [(1, 1), (1, 2)]})
#                (1, 'accept_all', 3, 49, [], {'l4': [(1, 1), (1, 2)]})

    # workspace.regions = {'r1': list(itertools.product(range(3), range(3))),
    #                      'r2': list(itertools.product(range(7, 10), range(7, 10))),
    #                      'r3': list(itertools.product(range(7, 9), range(7, 9)))}

    # workspace.obstacles = {obs: [point] for obs, point in workspace.obstacles.items()}
    # remove obstacles from regions
    # obs = list(itertools.chain(*workspace.obstacles.values()))
    # for robot, region in workspace.regions.items():
    #     workspace.regions[robot] = [point for point in region if point not in obs]

    # robot_team_initial_target = {(1, 0): (robot_path[(1, 0)], 'l2'),
    #                              (1, 1): (workspace.type_robot_location[(1, 1)], 'l3'),
    #                              (2, 1): (workspace.type_robot_location[(2, 1)], 'l2'),
    #                              (2, 2): (workspace.type_robot_location[(2, 2)], 'l2')}

    # Manhattan_dist = [workspace.p2p[(initial_target[1], 'r{0}'.
        #                                  format(1 + list(workspace.type_robot_location.keys()).index(robot)))]
        #                   for robot, initial_target in robot_team_initial_target.items()]
    # neg_symbols = [['r3', 1, 1], ['r3', 2, 1], ['r3', 3, 1]]
    start = datetime.datetime.now()
    robot_path = {type_robot: [location] for type_robot, location in workspace.type_robot_location.items()}

    for index, stage in enumerate(acpt_run):
        print(stage)
        # initial and target locations for some robots
        robot_team_initial_target = {robot: (robot_path[robot][-1], target) for target, robots in stage[-1].items()
                                     for robot in robots}
        # negative symbols
        neg_symbols = []
        # starting horizon
        if index != 0:
            horizon = stage[3] - acpt_run[index-1][3]
        else:
            horizon = stage[3]
        for T in range(horizon+5, horizon + 100, 10):
            paths = multi_agent_path_planning(workspace, T, robot_team_initial_target, neg_symbols)
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

    # for robot, path in paths.items():
    #     print(robot, path)

    # workspace.plot_workspace()
    # workspace.path_plot(paths)

    vis(workspace, robot_path, {robot: [len(path)] * 2 for robot, path in robot_path.items()}, [])

    plt.show()
