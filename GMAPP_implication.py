import networkx as nx
from gurobipy import *
from workspace_dars import Workspace
import matplotlib.pyplot as plt
import itertools
from vis import vis
import datetime
import numpy as np


def multi_agent_path_planning(workspace, T, robot_team_initial_target, robot_move, neg_clause, robot_init,
                              is_single_subtask):
    # build the time_expanded graph
    time_expanded_graph = nx.DiGraph()
    # s = datetime.datetime.now()
    loc2node, gadget, robot_index, edge_index_dict, index_edge_dict = build_time_expanded_graph(
        workspace, T, robot_move, time_expanded_graph, is_single_subtask)
    # print((datetime.datetime.now() - s).total_seconds())
    # ILP formulation
    m = Model()
    return ILP(m, time_expanded_graph, robot_team_initial_target, loc2node, gadget, robot_index, edge_index_dict,
               index_edge_dict, neg_clause, workspace, T, robot_init, is_single_subtask)


def build_time_expanded_graph(workspace, T, robot_move, time_expanded_graph, is_single_subtask):
    """
    build the time-expanded graph
    """
    # decide the horizon according to the existence of self-loop
    if is_single_subtask:
        horizon = T + 1
    else:
        horizon = T + 2
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
            edge = [(loc2node[u][t], loc2node[v][t+1]), (loc2node[v][t], loc2node[u][t + 1])]
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


def ILP(m, time_expanded_graph, robot_team_initial_target, loc2node, gadget, robot_index, edge_index_dict, index_edge_dict,
        neg_clause, workspace, T, robot_init, is_single_subtask):
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
    # 1. target location for the edge, at time T
    for robot, initial_target in robot_team_initial_target['edge'].items():
        target_node = [loc2node[loc][T] for loc in workspace.regions[initial_target[1]]
                       if loc in loc2node.keys()]
        i = robot_index.index(robot)
        m.addConstr(quicksum(x_vars[i, edge_index_dict[(p, t)]] for t in target_node
                             for p in time_expanded_graph.predecessors(t)) == 1)
    # 2. target location for the other edges, at time T
    for robot, initial_target in robot_team_initial_target['other_edges'].items():
        target_node = [loc2node[initial_target[1]][T]]
        i = robot_index.index(robot)
        m.addConstr(quicksum(x_vars[i, edge_index_dict[(p, t)]] for t in target_node
                             for p in time_expanded_graph.predecessors(t)) == 1)
    # 3. target location for the next vertex/next edge, at time T+1
    for robot, initial_target in robot_team_initial_target['vertex2'].items():
        target_node = [loc2node[loc][T+1] for loc in workspace.regions[initial_target[1]]
                       if loc in loc2node.keys()]
        i = robot_index.index(robot)
        m.addConstr(quicksum(x_vars[i, edge_index_dict[(p, t)]] for t in target_node
                             for p in time_expanded_graph.predecessors(t)) == 1)
    # 4. target location for the current vertex, at time 1,....T-1
    for robot, initial_target in robot_team_initial_target['vertex1'].items():
        i = robot_index.index(robot)
        for tt in range(1, T):
            target_node = [loc2node[loc][tt] for loc in workspace.regions[initial_target[1]]
                           if loc in loc2node.keys()]
            m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, t)])]
                                 for t in target_node for p in time_expanded_graph.predecessors(t)) == 1)
    # 5. target location for the next next vertex,  at time T+2
    if not is_single_subtask:
        for robot, initial_target in robot_team_initial_target['next_artificial'].items():
            target_node = [loc2node[loc][T+2] for loc in workspace.regions[initial_target[1]]
                           if loc in loc2node.keys()]
            i = robot_index.index(robot)
            m.addConstr(quicksum(x_vars[i, edge_index_dict[(p, t)]] for t in target_node
                                 for p in time_expanded_graph.predecessors(t)) == 1)

    # (25) flow conservation: exclude the initial and terminal nodes
    exclude = [nodes[0] for nodes in loc2node.values()] + [nodes[-1] for nodes in loc2node.values()]
    for node in time_expanded_graph.nodes():
        if node not in exclude:
            for i in range(len(robot_index)):
                m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, node)])] for p in time_expanded_graph.predecessors(node))
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
    # 1. negative literals in the edge label at time T
    for neg_lit in neg_clause['edge']:
        index_robot = [i for i, r in enumerate(robot_index) if r[0] == neg_lit[1]]
        target_node = [loc2node[loc][T] for loc in workspace.regions[neg_lit[0]]
                       if loc in loc2node.keys()]
        m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, t)])] for i in index_robot
                             for t in target_node for p in time_expanded_graph.predecessors(t)) <= neg_lit[2]-1)

    # 2. avoid negative literals in the current vertex at time 1, ... T-1
    for neg_lit in neg_clause['vertex1']:
        index_robot = [i for i, r in enumerate(robot_index) if r[0] == neg_lit[1]]
        for tt in range(1, T):
            target_node = [loc2node[loc][tt] for loc in workspace.regions[neg_lit[0]]
                           if loc in loc2node.keys()]
            m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, t)])] for i in index_robot
                                 for t in target_node for p in time_expanded_graph.predecessors(t)) <= neg_lit[2]-1)

    # 3. avoid negative literals in the next vertex/next edge, at time T+1
    for neg_lit in neg_clause['vertex2']:
        index_robot = [i for i, r in enumerate(robot_index) if r[0] == neg_lit[1]]
        target_node = [loc2node[loc][T+1] for loc in workspace.regions[neg_lit[0]]
                       if loc in loc2node.keys()]
        m.addConstr(quicksum(x_vars[(i, edge_index_dict[(p, t)])] for i in index_robot
                             for t in target_node for p in time_expanded_graph.predecessors(t)) <= neg_lit[2]-1)

    # 4. avoid negative literals in the next next vertex, at time T+2
    if not is_single_subtask:
        for neg_lit in neg_clause['next_artificial']:
            index_robot = [i for i, r in enumerate(robot_index) if r[0] == neg_lit[1]]
            target_node = [loc2node[loc][T+2] for loc in workspace.regions[neg_lit[0]]
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
                          is_single_subtask)
    return paths


def extract_paths(x_vars, time_expanded_graph, loc2node, robot_index, edge_index_dict, T, robot_init,
                  is_single_subtask):
    """
    extract the path from the GMMPP
    """
    if is_single_subtask:
        horizon = T + 1
    else:
        horizon = T + 2
    paths = {robot: [] for robot in robot_index}
    for index, robot in enumerate(robot_index):
        # the initial location
        node = loc2node[robot_init[robot]][0]
        for i in range(horizon+1):
            # the location that node corresponds to
            paths[robot].append(time_expanded_graph.nodes[node]['loc'])
            # find the next transition
            for edge in time_expanded_graph.edges(node):
                if x_vars[(index, edge_index_dict[edge])].x == 1:
                    node = edge[1]
                    break

    return paths


def mapp(workspace, buchi, acpt_run, robot_waypoint, robot_time, is_nonempty_self_loop, order, part='prefix'):
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
            for T in range(horizon+2, horizon + 100, 1):
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

        # vis(workspace, robot_path, {robot: [len(path)] * 2 for robot, path in robot_path.items()}, [])
    elif order == 'simultaneous':
        robot_path = {type_robot: [location] for type_robot, location in workspace.type_robot_location.items()}
        robot_progress = {robot: -1 for robot in robot_waypoint.keys()}
        clock = -1
        # with self-loop, stop at the second-to-last subtask since the last one is true
        if is_nonempty_self_loop:
            effective_length = len(acpt_run)-2
        # without self-loop, stop at the third-to-last subtask since the the third-to-last and second-to-last will be
        # considered together and the last one is true
        elif part == 'suffix':
            effective_length = len(acpt_run)-3
        else:
            effective_length = len(acpt_run)-1

        while clock < effective_length:
            print(acpt_run[clock+1]['subtask'])

            # check if the initial robot locations satisfy the first subtask or the current subtask is '1'
            # if yes, update the local progress of robots that participate and global clock
            if acpt_run[clock+1]['subtask'] in buchi.sat_init_edge:  # or acpt_run[clock+1]['essential_clause_edge'] == '1':
                for robot in acpt_run[clock+1]['essential_robot_edge'].values():
                    robot_progress[robot] += 1
                clock += 1
                continue

            is_single_subtask = True
            # determine the target for robots: (1) edge (2) vertex of the current subtask (3) vertex of the next subtask
            robot_team_initial_target = dict()
            robot_team_initial_target['edge'] = {robot: (robot_path[robot][-1], target) for target, robots in
                                                 acpt_run[clock+1]['essential_robot_edge'].items() for robot in robots}
            robot_team_initial_target['vertex1'] = {robot: (robot_path[robot][-1], target) for target, robots in
                                                    acpt_run[clock+1]['essential_robot_vertex'].items() for robot in robots}
            # 1. with self-loop, 2. without self-loop but not the third-to-last subtask:
            # activiate the vertex of the next subtask
            if is_nonempty_self_loop or clock + 1 < effective_length:
                robot_team_initial_target['vertex2'] = {robot: (robot_path[robot][-1], target) for target, robots in
                                                        acpt_run[clock+2]['essential_robot_vertex'].items()
                                                        for robot in robots}
            # without self-loop and the third-to-last subtask: activate the edge of the next subtask (accept -> next)
            # and the vertex of the next-next subtask (next -> next')
            elif not is_nonempty_self_loop and clock + 1 == effective_length and part == 'suffix':
                robot_team_initial_target['vertex2'] = {robot: (robot_path[robot][-1], target) for target, robots in
                                                        acpt_run[clock + 2]['essential_robot_edge'].items()
                                                        for robot in robots}
                robot_team_initial_target['next_artificial'] = {robot: (robot_path[robot][-1], target)
                                                                for target, robots in
                                                                acpt_run[clock + 3]['essential_robot_vertex'].items()
                                                                for robot in robots}
                is_single_subtask = False

            # determine the running and terminal constraints
            neg_clause = dict()
            neg_clause['edge'] = acpt_run[clock+1]['neg_edge']
            neg_clause['vertex1'] = acpt_run[clock+1]['neg_vertex']
            if is_nonempty_self_loop or clock + 1 < effective_length:
                neg_clause['vertex2'] = acpt_run[clock+2]['neg_vertex']
            elif not is_nonempty_self_loop and clock + 1 == effective_length and part == 'suffix':
                neg_clause['vertex2'] = acpt_run[clock + 2]['neg_edge']
                neg_clause['next_artificial'] = acpt_run[clock + 3]['neg_vertex']

            # update robot_team_initial_target by considering simultaneity
            next_time = acpt_run[clock+1]['time_element'][0]  # the completion of the current subtask

            # robots that need to move according to positive literals
            partial_or_full = 'f'
            robot_move = set(robot for robot_initial_target in robot_team_initial_target.values()
                             for robot in robot_initial_target.keys())
            remove_edge = update_robot_env(workspace, robot_team_initial_target, robot_move, robot_waypoint, robot_time,
                                           robot_path, robot_progress, next_time, neg_clause, partial_or_full)
            if partial_or_full == 'f':
                robot_move = list(robot_path.keys())

            # ------ find the collision-avoidance paths for involved robots ---------
            # the completion time of last subtask
            if clock == -1:
                past_time = 0
            else:
                past_time = acpt_run[clock]['time_element'][0]
            # expected horizon according to high-level plan
            horizon = next_time - past_time
            robot_init = {robot: path[-1] for robot, path in robot_path.items()}
            for T in range(horizon, horizon + 100, 1):
                mapp_paths = multi_agent_path_planning(workspace, T, robot_team_initial_target, robot_move, neg_clause,
                                                       robot_init, is_single_subtask)
                if mapp_paths:
                    # 1. with self-loop, 2. without self-loop but not the third-to-last subtask:
                    # activiate the vertex of the next subtask
                    if is_nonempty_self_loop or clock + 1 < effective_length:
                        # update the path: second to the last to one
                        for robot, path in mapp_paths.items():
                            robot_path[robot] += path[1:-1]
                        for robot in robot_path.keys():
                            if robot not in mapp_paths.keys():
                                robot_path[robot] += [robot_path[robot][-1]] * T
                        break
                    # without self-loop and the third-to-last subtask: activate the edge of the next subtask
                    # (accept -> next) and the vertex of the next-next subtask (next -> next')
                    elif not is_nonempty_self_loop and clock + 1 == effective_length and part == 'suffix':
                        # update the path: second to the last
                        for robot, path in mapp_paths.items():
                            robot_path[robot] += path[1:]
                        for robot in robot_path.keys():
                            if robot not in mapp_paths.keys():
                                robot_path[robot] += [robot_path[robot][-1]] * (T+1)
                        break

            # update key time points for each robot
            if T > horizon:
                for robot, time in robot_time.items():
                    if robot_progress[robot] + 1 < len(robot_time[robot]):
                        robot_time[robot][robot_progress[robot]+1:] = [t+T-horizon
                                                                       for t in robot_time[robot][robot_progress[robot]+1:]]

            # update the overall progress
            past_time += T
            # update the individual progress
            for robots in acpt_run[clock+1]['essential_robot_edge'].values():
                for robot in robots:
                    robot_progress[robot] += 1
            # update the overall plan
            for subseq_clock in range(clock+1, len(acpt_run)):
                acpt_run[subseq_clock]['time_element'][0] = acpt_run[subseq_clock]['time_element'][0] + T - horizon
            # restore removed edges
            workspace.graph_workspace.add_edges_from(itertools.chain(remove_edge))
            # plt.show()
            clock += 1

        end = datetime.datetime.now()
        print('GMMPP: ', (end - start).total_seconds())


        # vis(workspace, robot_path, {robot: [len(path)] * 2 for robot, path in robot_path.items()}, [])
        # vis(workspace, robot_path, {robot: [len(path)]*2 for robot, path in robot_path.items()}, task.ap)
        return robot_path


def update_robot_env(workspace, robot_team_initial_target, robot_move, robot_waypoint, robot_time, robot_path,
                     robot_progress, next_time, neg_clause, partial_or_full):
    """
    find robots that need to move simultaneous and those that remain idle
    """
    # find robots that need to move
    robot_future_time_min_length_path_target = dict()
    for robot in robot_progress.keys():
        if robot not in robot_move:
            # time difference between the next progress (edge) and the key time point for the considered robot
            if robot_progress[robot] + 1 < len(robot_time[robot]):
                future_time = robot_time[robot][robot_progress[robot]+1] - next_time
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
            # robots need to move since the remaining time is not enough
            if future_time < min_length:
                robot_future_time_min_length_path_target[robot] = [future_time, min_length, path, min_target]

    # those robots that are be involved in the negative literals
    robot_team_initial_target['constraint'] = {(neg_lit[1], robot): (robot_path[(neg_lit[1], robot)][-1], None)
                                               for clause in neg_clause.values() for neg_lit in clause
                                               for robot in range(workspace.type_num[neg_lit[1]])}
    robot_move.update(set(robot_team_initial_target['constraint'].keys()))

    # treat robots that do not move as obstacles
    if partial_or_full == 'p':
        new_obstacles = [path[-1] for robot, path in robot_path.items() if robot not in robot_move
                         and robot not in robot_future_time_min_length_path_target.keys()]
    elif partial_or_full == 'f':
        new_obstacles = []

    # treat all robots that do not execute the current subtask as obstacles
    remove_edge = []
    for obs in new_obstacles:
        remove_edge += list(workspace.graph_workspace.edges(obs))
        workspace.graph_workspace.remove_node(obs)

    # determine the target point at the next time
    other_edge = dict()
    for robot in robot_future_time_min_length_path_target.keys():
        future_time, min_length, path, min_target = robot_future_time_min_length_path_target[robot]
        target = path[min_length-future_time]
        # find the intermediate target for mapp
        if target in new_obstacles:
            l, p = nx.algorithms.single_source_dijkstra(workspace.graph_workspace,
                                                        source=robot_path[robot][-1],
                                                        target=min_target)
            # if target is already selected, select previous one
            index = l - 1 - future_time
            while target in new_obstacles:
                target = p[index]
                try:
                    index -= 1
                except KeyError:
                    break

        if target != robot_path[robot][-1]:
            # the free cell is occupied by some robot
            new_obstacles.append(target)
            other_edge[robot] = (robot_path[robot][-1], target)
            robot_move.add(robot)
    # update the set of robots that need to move
    robot_team_initial_target['other_edges'] = other_edge

    return remove_edge
