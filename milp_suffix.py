from gurobipy import *
import math


def construct_milp_constraint(ts, type_num, poset, pruned_subgraph, element2edge,
                              element_component_clause_literal_node, strict_poset_relation, init_type_robot_node,
                              incomparable_element, larger_element, terminator, robot2eccl, robot2robots, workspace,
                              factor):

    M = 1e5
    epsilon = 1/factor  # edge and previous edge

    m = Model()
    # create variables
    x_vars, t_vars, c_vars, t_edge_vars, b_element_vars = create_variables(m, ts, poset, pruned_subgraph, element2edge,
                                                                           type_num, factor)
    # create initial constraints
    initial_constraints(m, x_vars, t_vars, ts, init_type_robot_node, type_num, terminator)

    # network and schedule constraints
    network_schedule_constraints(m, ts, x_vars, t_vars, init_type_robot_node, incomparable_element, larger_element,
                                 type_num, M, epsilon)

    # using same robot
    for robot, eccls in robot2eccl.items():
        clause = [list(group) for key, group in itertools.groupby(eccls, operator.itemgetter(0, 1))]
        num_vertex = len(element_component_clause_literal_node[eccls[0]])
        for c in clause:
            for l in c:
                for vertex in range(num_vertex):
                    w = element_component_clause_literal_node[l][vertex]
                    m.addConstr(quicksum(x_vars[(p, w, robot2robots[robot][vertex])] for p in ts.predecessors(w))
                                + M * (c_vars[l[:-1]] - 1) <= 1)

                    m.addConstr(1 <= quicksum(x_vars[(p, w, robot2robots[robot][vertex])] for p in ts.predecessors(w))
                                + M * (1 - c_vars[l[:-1]]))

    m.update()

    # focus on logical relation
    for element in poset:
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']

        if edge_label != '1':
            edge_constraints(m, ts, x_vars, t_vars, c_vars, t_edge_vars, element, self_loop_label, edge_label,
                             strict_poset_relation, pruned_subgraph, element2edge,
                             element_component_clause_literal_node, type_num, M, epsilon)
        else:
            continue

        if self_loop_label and self_loop_label != '1':
            self_loop_constraints(m, ts, x_vars, t_vars, c_vars, t_edge_vars, b_element_vars, element, self_loop_label,
                                  incomparable_element, strict_poset_relation, pruned_subgraph, element2edge,
                                  element_component_clause_literal_node, type_num, M, epsilon)

    expr = LinExpr([0.7*ts.edges[tuple(index[:2])]['weight'] for index in x_vars.keys()], list(x_vars.values()))
    expr.add(LinExpr([0.3] * len([key for key in t_vars.keys() if key[2] == 1]),
                     [value for key, value in t_vars.items() if key[2] == 1]))
    m.setObjective(expr, GRB.MINIMIZE)
    # m.Params.OutputFlag = 0
    m.update()
    m.optimize()
    if m.status == GRB.Status.OPTIMAL:
        print('Optimal objective: %g' % m.objVal)
    elif m.status == GRB.Status.INF_OR_UNBD:
        print('Model is infeasible or unbounded')
        exit(0)
    elif m.status == GRB.Status.INFEASIBLE:
        print('Model is infeasible')
        exit(0)
    elif m.status == GRB.Status.UNBOUNDED:
        print('Model is unbounded')
        exit(0)
    else:
        print('Optimization ended with status %d' % m.status)
        exit(0)

    print('# of variables: {0}'.format(m.NumVars))
    print('# of constraints: {0}'.format(m.NumConstrs))

    return get_waypoint(x_vars, t_vars, ts, init_type_robot_node, terminator, workspace, factor)


def create_variables(m, ts, poset, pruned_subgraph, element2edge, type_num, factor):
    # clause variable, (element, 0|1, clause_index)
    c_vars = {}
    for element in poset:
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']
        if self_loop_label and self_loop_label != '1':
            c_vars.update(m.addVars([element], [0], list(range(len(self_loop_label))), vtype=GRB.BINARY))
        if edge_label != '1':
            c_vars.update(m.addVars([element], [1], list(range(len(edge_label))), vtype=GRB.BINARY))

    m.update()

    # time variable (node, robot_index, -|+)
    t_vars = {}
    v_type = GRB.INTEGER if factor == 1 else GRB.CONTINUOUS
    for node in ts.nodes():
        # vars for self loop
        if ts.nodes[node]['location_type_component_element'][2] == 0:
            t_vars.update(m.addVars([node], list(range(type_num[ts.nodes[node]['location_type_component_element'][1]])), [0, 1],
                                    vtype=v_type))
        # vars for edge
        else:
            t_vars.update(m.addVars([node], list(range(type_num[ts.nodes[node]['location_type_component_element'][1]])), [1],
                                    vtype=v_type))
    m.update()
    # routing variable (node_i, node_j, robot_index)
    x_vars = {}
    for edge in ts.edges():
        x_vars.update(m.addVars([edge[0]], [edge[1]],
                                list(range(type_num[ts.nodes[edge[1]]['location_type_component_element'][1]])),
                                vtype=GRB.BINARY))
    m.update()
    # edge time variable (element)
    t_edge_vars = {}
    for element in poset:
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']
        if edge_label != '1':
            t_edge_vars.update(m.addVars([element], vtype=v_type))
    # binary relation between edge time (element, element)
    b_element_vars = {}
    for element in poset:
        for another_element in poset:
            if element != another_element:
                b_element_vars.update(m.addVars([element], [another_element], vtype=GRB.BINARY))

    return x_vars, t_vars, c_vars, t_edge_vars, b_element_vars


def initial_constraints(m, x_vars, t_vars, ts, init_type_robot_node, type_num, terminator):
    # create initial constraints
    # nodes for initial locations -- eq (2)
    for type_robot, node in init_type_robot_node.items():
        m.addConstr(1 >= quicksum(x_vars[(node, s, type_robot[1])] for s in ts.successors(node)))
        m.addConstr(0 == quicksum(x_vars[(node, s, k)] for s in ts.successors(node) for k in
                                  range(type_num[ts.nodes[s]['location_type_component_element'][1]])
                                  if k != type_robot[1]))
        # initial time -- eq (3) starting from the same time base
        for k in range(type_num[type_robot[0]]):
            m.addConstr(t_vars[(node, k, 1)] == terminator[type_robot])
    m.update()


def one_clause_true(m, c_vars, element, component, label):
    m.addConstr(quicksum(c_vars[(element, component, c)] for c in range(len(label))) == 1)


def literal_clause(m, x_vars, c_vars, element, component, label, ts, type_num, clause_nodes, c):
    # encode the relation between clause and its literals -- eq (6)

    expr_literal = quicksum(x_vars[(p, i, k)] for literal_nodes in clause_nodes for i in literal_nodes for p in ts.predecessors(i)
                            for k in range(type_num[ts.nodes[i]['location_type_component_element'][1]]))
    mult = sum([l[2] for l in label[c]])
    m.addConstr(expr_literal/mult == c_vars[(element, component, c)])


def network_schedule_constraints(m, ts, x_vars, t_vars, init_type_robot_node, incomparable_element, larger_element,
                                 type_num, M, epsilon):
    # focus on nodes
    for i in ts.nodes():
        if i not in init_type_robot_node.values() and list(ts.predecessors(i)):  # the predecessor of i
            # each (replica) location represented by node is visited by at most one robot of type k -- eq (1)
            m.addConstr(quicksum(x_vars[(p, i, k)] for p in ts.predecessors(i)
                                 for k in range(type_num[ts.nodes[i]['location_type_component_element'][1]])) <= 1)

            for k in range(type_num[ts.nodes[i]['location_type_component_element'][1]]):
                # routing network that the visitation of a robot to a location i
                # is the precondition for its departure from that location -- eq (2)
                m.addConstr(quicksum(x_vars[(p, i, k)] for p in ts.predecessors(i)) >=
                            quicksum(x_vars[(i, s, k)] for s in ts.successors(i)))
                # t_ik = 0 if location i is not visited by the robot k -- eq (4)
                m.addConstr(t_vars[(i, k, 1)] <= M * quicksum(x_vars[(p, i, k)] for p in ts.predecessors(i)))
                if ts.nodes[i]['location_type_component_element'][2] == 0:
                    m.addConstr(t_vars[(i, k, 0)] <= M * quicksum(x_vars[(p, i, k)] for p in ts.predecessors(i)))

                # The time of robot k visiting p should be larger than that of the same robot
                # visiting i if there is a plan from p to i -- eq (3)
                i_element = ts.nodes[i]['location_type_component_element'][3]
                for p in ts.predecessors(i):
                    p_element = ts.nodes[p]['location_type_component_element'][3]
                    # incomparable elements, using epsilon to avoid loop
                    if p_element in incomparable_element[i_element]:
                        if ts.nodes[i]['location_type_component_element'][2] == 0:
                            m.addConstr(
                                t_vars[(p, k, 1)] + (epsilon + ts.edges[(p, i)]['weight']) * x_vars[(p, i, k)] <=
                                t_vars[(i, k, 0)] + M * (1 - x_vars[(p, i, k)]))
                        else:
                            m.addConstr(
                                t_vars[(p, k, 1)] + (epsilon + ts.edges[(p, i)]['weight']) * x_vars[(p, i, k)] <=
                                t_vars[(i, k, 1)] + M * (1 - x_vars[(p, i, k)]))
                    # precedent elements, include initial element -1
                    elif p_element in larger_element[i_element] + [-1]:
                        if ts.nodes[i]['location_type_component_element'][2] == 0:
                            m.addConstr(t_vars[(p, k, 1)] + ts.edges[(p, i)]['weight'] * x_vars[(p, i, k)] <=
                                        t_vars[(i, k, 0)] + M * (1 - x_vars[(p, i, k)]))
                        else:
                            m.addConstr(t_vars[(p, k, 1)] + ts.edges[(p, i)]['weight'] * x_vars[(p, i, k)] <=
                                        t_vars[(i, k, 1)] + M * (1 - x_vars[(p, i, k)]))

    m.update()


def edge_constraints(m, ts, x_vars, t_vars, c_vars, t_edge_vars, element, self_loop_label, edge_label,
                     strict_poset_relation, pruned_subgraph, element2edge,
                     element_component_clause_literal_node, type_num, M, epsilon):
    # one and only one clause is true -- eq (5)
    one_clause_true(m, c_vars, element, 1, edge_label)
    for c in range(len(edge_label)):
        # the nodes corresponding to each clause
        clause_nodes = []  # each item is a set of literal nodes
        for l in range(len(edge_label[c])):
            clause_nodes.append(element_component_clause_literal_node[(element, 1, c, l)])

        # encode the relation between clause and its literals -- eq (6)
        literal_clause(m, x_vars, c_vars, element, 1, edge_label, ts, type_num, clause_nodes, c)

        # encode the synchronization constraints in terms of timing -- eq (7)
        for l in clause_nodes:
            for i in l:
                m.addConstr(quicksum(t_vars[(clause_nodes[0][0], k, 1)]
                                     for k in range(
                    type_num[ts.nodes[clause_nodes[0][0]]['location_type_component_element'][1]])) ==
                            quicksum(t_vars[(i, k, 1)] for k in
                                     range(type_num[ts.nodes[i]['location_type_component_element'][1]])))

        # timing constraints between a node and its outgoing edge -- eq (9)-(10)
        if self_loop_label and self_loop_label != '1':
            for c_self_loop in range(len(self_loop_label)):
                for l_self_loop in range(len(self_loop_label[c_self_loop])):
                    for i in element_component_clause_literal_node[(element, 0, c_self_loop, l_self_loop)]:
                        m.addConstr(quicksum(t_vars[(i, k, 0)] for k in
                                             range(type_num[ts.nodes[i]['location_type_component_element'][1]]))
                                    <= t_edge_vars[element])
                        m.addConstr(t_edge_vars[element] <= quicksum(t_vars[(i, k, 1)]
                                                                     for k in range(
                            type_num[ts.nodes[i]['location_type_component_element'][1]]))
                                    + M * (1 - c_vars[(element, 0, c_self_loop)]))

                        # m.addConstr(quicksum(t_vars[(i, k, 0)]
                        #                      for k in
                        #                      range(type_num[ts.nodes[i]['location_type_component_element'][1]]))
                        #             + M * (c_vars[(element, 0, c_self_loop)] - 1) <=
                        #             quicksum(t_vars[(clause_nodes[0][0], k, 1)]
                        #                      for k in range(
                        #                 type_num[ts.nodes[clause_nodes[0][0]]['location_type_component_element'][1]]))
                        #             + M * (1 - c_vars[(element, 1, c)]))  # any node in a clause of edge label
                        #
                        # m.addConstr(quicksum(t_vars[(clause_nodes[0][0], k, 1)]
                        #                      for k in range(
                        #     type_num[ts.nodes[clause_nodes[0][0]]['location_type_component_element'][1]]))
                        #             + M * (c_vars[(element, 1, c)] - 1) <=
                        #             quicksum(t_vars[(i, k, 1)]
                        #                      for k in
                        #                      range(type_num[ts.nodes[i]['location_type_component_element'][1]]))
                        #             + M * (1 - c_vars[(element, 0, c_self_loop)]))

        # precedence timing constraints, between edge and previous edge -- eq (12)
        for order in strict_poset_relation:
            if order[1] == element:
                m.addConstr(t_edge_vars[order[0]] + epsilon <= t_edge_vars[element])
                # larger_edge_label = pruned_subgraph.edges[element2edge[order[0]]]['label']
                # for c_edge in range(len(larger_edge_label)):
                #     i = element_component_clause_literal_node[(order[0], 1, c_edge, 0)][0]  # any node
                #     m.addConstr(quicksum(t_vars[(i, k, 1)]
                #                          for k in range(type_num[ts.nodes[i]['location_type_component_element'][1]]))
                #                 + M * (c_vars[(order[0], 1, c_edge)] - 1) + epsilon <=
                #                 quicksum(t_vars[(clause_nodes[0][0], k, 1)]
                #                          for k in range(
                #                     type_num[ts.nodes[clause_nodes[0][0]]['location_type_component_element'][1]]))
                #                 + M * (1 - c_vars[(element, 1, c)]))
    m.update()


def self_loop_constraints(m, ts, x_vars, t_vars, c_vars, t_edge_vars, b_element_vars, element, self_loop_label,
                          incomparable_element, strict_poset_relation, pruned_subgraph, element2edge,
                          element_component_clause_literal_node, type_num, M, epsilon):
    # one and only one clause is true, -- eq (5)
    one_clause_true(m, c_vars, element, 0, self_loop_label)

    for c in range(len(self_loop_label)):
        # the nodes corresponding to each clause
        clause_nodes = []
        for l in range(len(self_loop_label[c])):
            clause_nodes.append(element_component_clause_literal_node[(element, 0, c, l)])

        # encode the relation between clause and its literals -- eq (6)
        literal_clause(m, x_vars, c_vars, element, 0, self_loop_label, ts, type_num, clause_nodes, c)

        # # encode the synchronization constraints -- eq (8)
        # for l in clause_nodes:
        #     for i in l:
        #         m.addConstr(quicksum(x_vars[(p, clause_nodes[0][0], k)]
        #                              for p in ts.predecessors(clause_nodes[0][0])
        #                              for k in range(
        #             type_num[ts.nodes[clause_nodes[0][0]]['location_type_component_element'][1]])) ==
        #                     quicksum(x_vars[(p, i, k)]
        #                              for p in ts.predecessors(i)
        #                              for k in range(type_num[ts.nodes[i]['location_type_component_element'][1]])))
        # timing constraints between a node and its incoming edge

        # timing constraints between a node and its incoming edge -- eq (19-20)
        strict_incmp = [order[0] for order in strict_poset_relation if order[1] == element]
        strict_incmp += [e for e in incomparable_element[element]
                         if pruned_subgraph.edges[element2edge[e]]['label'] != '1']
        for l in clause_nodes:
            for j in l:
                for e in strict_incmp:
                    m.addConstr(quicksum(t_vars[(j, k, 0)] for k in range(type_num[ts.nodes[j]
                    ['location_type_component_element'][1]])) <=
                                t_edge_vars[e] + M * (len(strict_incmp) - 1 -
                                                      quicksum(b_element_vars[o] for o in strict_incmp if o != e)))

        for order in strict_poset_relation:
            if order[1] == element:
                larger_edge_label = pruned_subgraph.edges[element2edge[order[0]]]['label']
                for c_edge in range(len(larger_edge_label)):
                    i = element_component_clause_literal_node[(order[0], 1, c_edge, 0)][0]
                    for l in clause_nodes:
                        for j in l:
                            m.addConstr(quicksum(t_vars[(j, k, 0)]
                                                 for k in
                                                 range(type_num[ts.nodes[j]['location_type_component_element'][1]]))
                                        + M * (c_vars[(element, 0, c)] - 1) <=
                                        quicksum(t_vars[(i, k, 1)]
                                                 for k in
                                                 range(type_num[ts.nodes[i]['location_type_component_element'][1]]))
                                        + M * (1 - c_vars[(order[0], 1, c_edge)]))
    m.update()


def get_waypoint(x_vars, t_vars, ts, init_type_robot_node, terminator, workspace, factor):
    robot_waypoint = dict()
    robot_waypoint_plus = dict()
    robot_time = dict()
    for type_robot, node in init_type_robot_node.items():
        path = [node]
        time = [round(terminator[type_robot] * factor)]
        is_found = True
        while is_found:
            pre = path[-1]
            for s in ts.succ[pre]:
                if round(x_vars[(pre, s, type_robot[1])].x) == 1:
                    try:
                        time.append(round(t_vars[(s, type_robot[1], 0)].x * factor))
                        path.append(s)
                    except KeyError:
                        pass
                    time.append(round(t_vars[(s, type_robot[1], 1)].x * factor))
                    path.append(s)
                    break
            if path[-1] == pre:
                is_found = False

        robot_time[type_robot] = time
        # stay put
        if len(path) == 1:
            robot_waypoint_plus[type_robot] = [ts.nodes[path[0]]['location_type_component_element'][0]]
            robot_waypoint[type_robot] = [ts.nodes[path[0]]['location_type_component_element'][0]]

        # robot that moves
        else:
            robot_waypoint_plus[type_robot] = ['-' + ts.nodes[point]['location_type_component_element'][0] for point in path[:2]]\
                                              + ['+' + ts.nodes[point]['location_type_component_element'][0] for point in path[2:]]\
                                              + ['+' + ts.nodes[path[1]]['location_type_component_element'][0]]

            robot_waypoint[type_robot] = [ts.nodes[point]['location_type_component_element'][0] for point in path] \
                                         + [ts.nodes[path[1]]['location_type_component_element'][0]]

    t_max = max([robot_time[type_robot][-1] - robot_time[type_robot][1] + workspace.p2p[(waypoints[-2], waypoints[1])]
                 for type_robot, waypoints in robot_waypoint.items() if len(waypoints) > 1])
    for type_robot, waypoints in robot_waypoint.items():
        if len(waypoints) > 1:
            robot_time[type_robot] = robot_time[type_robot] + [robot_time[type_robot][1] + t_max]

    return robot_waypoint, robot_waypoint_plus, robot_time


def get_path(robot_waypoint, robot_time, workspace):
    robot_path = dict()
    for robot, waypoint in robot_waypoint.items():
        path = [workspace.type_robot_label[robot]]
        for i in range(len(waypoint)-1):
            if waypoint[i] == waypoint[i+1]:
                path += (round(robot_time[robot][i+1]) - round(robot_time[robot][i])) * [workspace.regions[waypoint[i]]]
            else:
                p = workspace.p2p_path[(waypoint[i], waypoint[i+1])]
                path = path + (round(robot_time[robot][i+1]) - round(robot_time[robot][i]) + 1 - len(p)) * [p[0]] + p[1:]
        robot_path[robot] = path

    return robot_path
