import networkx as nx
from gurobipy import *
from workspace_dars import Workspace
import matplotlib.pyplot as plt
import itertools
from vis import vis
import datetime
import numpy as np


def multi_agent_path_planning(workspace, T, robot_init_target, robot_target_region,
                              robot_move, neg_clause, robot_init):
    # build the time_expanded graph
    time_expanded_graph = nx.DiGraph()
    # s = datetime.datetime.now()
    loc2node, gadget, robot_index, edge_index_dict, index_edge_dict = build_time_expanded_graph(
        workspace, T, robot_move, time_expanded_graph)
    # print((datetime.datetime.now() - s).total_seconds())
    # ILP formulation
    m = Model()
    return ILP(m, time_expanded_graph, robot_init_target, robot_target_region,
               loc2node, gadget, robot_index, edge_index_dict, index_edge_dict, neg_clause, workspace, T, robot_init)


def build_time_expanded_graph(workspace, T, robot_move, time_expanded_graph):
    """
    build the time-expanded graph
    """
    # decide the horizon according to the existence of self-loop
    horizon = T
    # add nodes from G
    loc2node = dict()
    node_index = 0
    for loc in workspace.graph_workspace.nodes():
        time_expanded_graph.add_nodes_from(range(node_index, node_index + horizon + 1), loc=loc)
        loc2node[loc] = range(node_index, node_index + horizon + 1)
        node_index = node_index + horizon + 1

    edge_index = []
    # index the robot set
    robot_index = list(robot_move)
    gadget = []
    # gadget
    for u, v in workspace.graph_workspace.edges():
        for t in range(0, horizon):
            edge = [(loc2node[u][t], loc2node[v][t + 1]), (loc2node[v][t], loc2node[u][t + 1])]
            time_expanded_graph.add_edges_from(edge, cost=1)
            edge_index += edge
            gadget.append(edge)
    # same-node edges
    for loc in loc2node.keys():
        time_expanded_graph.add_edges_from([(loc2node[loc][i], loc2node[loc][i + 1]) for i in range(horizon)], cost=0)
        edge_index += [(loc2node[loc][i], loc2node[loc][i + 1]) for i in range(horizon)]
    # index the edge set
    edge_index_dict = {edge: index for index, edge in enumerate(edge_index)}
    index_edge_dict = {index: edge for index, edge in enumerate(edge_index)}

    return loc2node, gadget, robot_index, edge_index_dict, index_edge_dict


def ILP(m, time_expanded_graph, robot_init_target, robot_target_region, loc2node,
        gadget, robot_index, edge_index_dict, index_edge_dict, neg_clause, workspace, T, robot_init,):
    """
    build the ILP to solve GMMPP
    """
    # create variables
    x_vars = {}
    x_vars.update(m.addVars(list(range(len(robot_index))), list(range(time_expanded_graph.number_of_edges())),
                            vtype=GRB.BINARY))
    m.update()

    # (24) routing constraint: each edge can be traversed by at most one robot at the given time
    for j in range(time_expanded_graph.number_of_edges()):
        m.addConstr(quicksum(x_vars[(i, j)] for i in range(len(robot_index))) <= 1)

    # (26a) and (26b) robot has to depart from its initial location.
    for i, robot in enumerate(robot_index):
        # find the label_type (edge, vertex1, vertex2) that robot corresponds to
        initial_node = loc2node[robot_init[robot]][0]
        m.addConstr(quicksum(x_vars[i, edge_index_dict[j]] for j in time_expanded_graph.edges(initial_node)) == 1)
        initial_nodes = [nodes[0] for nodes in loc2node.values()]
        initial_nodes.remove(initial_node)
        m.addConstr(quicksum(x_vars[i, edge_index_dict[j]] for node in initial_nodes
                             for j in time_expanded_graph.edges(node)) == 0)

    # target location (sink) (29)
    # 1. target location for robots, at time T
    for robot, initial_target in robot_init_target.items():
        target_node = [loc2node[initial_target[1]][T]]
        i = robot_index.index(robot)
        m.addConstr(quicksum(x_vars[i, edge_index_dict[(p, t)]] for t in target_node
                             for p in time_expanded_graph.predecessors(t)) == 1)

    # 2. target location for the current vertex, at time 1,....T
    for robot, initial_target in robot_target_region['vertex1'].items():
        i = robot_index.index(robot)
        for tt in range(1, T+1):
            target_node = [loc2node[loc][tt] for loc in workspace.regions[initial_target[1]]
                           if loc in loc2node.keys()]
            m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, t)])]
                                 for t in target_node for p in time_expanded_graph.predecessors(t)) == 1)

    # (25) flow conservation: exclude the initial and terminal nodes
    exclude = [nodes[0] for nodes in loc2node.values()] + [nodes[-1] for nodes in loc2node.values()]
    for node in time_expanded_graph.nodes():
        if node not in exclude:
            for i in range(len(robot_index)):
                m.addConstr(
                    quicksum(x_vars[(i, edge_index_dict[(p, node)])] for p in time_expanded_graph.predecessors(node))
                    == quicksum(x_vars[(i, edge_index_dict[(node, s)])] for s in time_expanded_graph.successors(node)))

    # (27) head-on collision constraint for a single gadget
    for edge_pair in gadget:
        m.addConstr(
            quicksum(x_vars[i, edge_index_dict[edge]] for i in range(len(robot_index)) for edge in edge_pair) <= 1)

    # (28) the meet collision constraint
    for node in time_expanded_graph.nodes():
        m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, node)])] for i in range(len(robot_index))
                             for p in time_expanded_graph.predecessors(node)) <= 1)

    # (30) avoid negative literals
    # 2. avoid negative literals in the current vertex at time 1, ... T
    for neg_lit in neg_clause['vertex1']:
        index_robot = [i for i, r in enumerate(robot_index) if r[0] == neg_lit[1]]
        for tt in range(1, T+1):
            target_node = [loc2node[loc][tt] for loc in workspace.regions[neg_lit[0]]
                           if loc in loc2node.keys()]
            m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, t)])] for i in index_robot
                                 for t in target_node for p in time_expanded_graph.predecessors(t)) <= neg_lit[2] - 1)

    m.update()
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

    paths = extract_paths(x_vars, time_expanded_graph, loc2node, robot_index, edge_index_dict, T, robot_init,
                          )
    return paths


def extract_paths(x_vars, time_expanded_graph, loc2node, robot_index, edge_index_dict, T, robot_init,
                  ):
    """
    extract the path from the GMMPP
    """
    horizon = T
    paths = {robot: [] for robot in robot_index}
    for index, robot in enumerate(robot_index):
        # the initial location
        node = loc2node[robot_init[robot]][0]
        for i in range(horizon + 1):
            # the location that node corresponds to
            paths[robot].append(time_expanded_graph.nodes[node]['loc'])
            # find the next transition
            for edge in time_expanded_graph.edges(node):
                if x_vars[(index, edge_index_dict[edge])].x == 1:
                    node = edge[1]
                    break

    return paths


def mapp_for_loop(exe_robot_next_vertex, final_loc_pre, final_loc_suf,
                  essential_clause_next_clause, neg_clause_next_vertex, workspace):
    """
    find the paths back to the initial locations
    """
    robot_init_target = {robot: (final_loc_suf[robot], final_loc_pre[robot]) for robot in final_loc_pre.keys()}

    robot_target_region = dict()
    robot_target_region['vertex1'] = dict()
    # save matches of robots among those that satisfy of the vertex label
    for cl in exe_robot_next_vertex.keys():
        # robots stay at target regions always
        robot_target_region['vertex1'].update({robot: (final_loc_suf[robot], essential_clause_next_clause[cl[1]][0])
                                               for robot in exe_robot_next_vertex[cl]})

    # negative clause
    neg_clause = dict()
    neg_clause['vertex1'] = neg_clause_next_vertex

    horizon = 0

    for init, target in robot_init_target.values():
        length, _ = nx.algorithms.single_source_dijkstra(workspace.graph_workspace, source=init, target=target)
        horizon = length if length > horizon else horizon

    for T in range(horizon, horizon + 100, 1):
        mapp_paths = multi_agent_path_planning(workspace, T, robot_init_target, robot_target_region,
                                               list(final_loc_suf.keys()), neg_clause, final_loc_suf)
        if mapp_paths:
            return mapp_paths
