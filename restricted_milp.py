from gurobipy import *
import math
from restricted_post_processing import run
from termcolor import colored, cprint
import itertools

print_red_on_cyan = lambda x: cprint(x, 'blue', 'on_red')


def construct_milp_constraint(ts, type_num, reduced_task_network, task_hierarchy, task_element_component_clause_literal_node, \
    init_type_robot_node, strict_larger_task_element, incomparable_task_element, 
    larger_task_element, maximal_task_element, robot2teccl, composite_subtasks, pairwise_or_relation_composite_subtasks, show=True):
    M = 1e5
    epsilon = 1  # edge and previous edge
    m = Model()
    # create variables
    task_vars, x_vars, t_vars, c_vars, t_edge_vars, b_element_vars, b_immediate_element_vars, b_maximal_element_vars \
        = create_variables(m, ts, reduced_task_network, task_hierarchy, type_num, maximal_task_element, composite_subtasks)

    # # create initial constraints
    initial_constraints(m, x_vars, t_vars, ts, init_type_robot_node, type_num)
    
    task_constraints(m, task_vars, composite_subtasks)

    # network and schedule constraints
    network_schedule_constraints(m, ts, x_vars, t_vars, init_type_robot_node, incomparable_task_element, larger_task_element,
                                 type_num, M, epsilon)

    # edge time constraints -- eq (12)
    for (task, element) in reduced_task_network:
        pruned_subgraph = task_hierarchy[task].buchi_graph
        element2edge = task_hierarchy[task].element2edge
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']
        # must have vertices
        if edge_label != '1':
            m.addConstr(quicksum(t_vars[(task_element_component_clause_literal_node[(task, element, 1, c, 0)][0], k, 1)]
                                 for c in range(len(edge_label)) for k in range(
                type_num[ts.nodes[task_element_component_clause_literal_node[(task, element, 1, c, 0)][0]]
                ['location_type_component_task_element'][1]])) == t_edge_vars[(task, element)])

    m.update()

    # binary relation between edge time -- eq (19)
    # no matter the edge label is '1' or not

    for pair in itertools.combinations(reduced_task_network, 2):
        # no subtasks are completed at the same time -- eq (19a)
        if (pair[0][0], pair[1][0]) not in pairwise_or_relation_composite_subtasks and (pair[1][0], pair[0][0]) not in pairwise_or_relation_composite_subtasks:
            m.addConstr(b_element_vars[pair[0]+ pair[1]] + b_element_vars[pair[1] + pair[0]] == 1)

    for task_element in reduced_task_network:
        for another_task_element in reduced_task_network:
            if task_element != another_task_element:
                # # no subtasks are completed at the same time -- eq (19a)
                # m.addConstr(b_element_vars[(element, another_element)] + b_element_vars[(another_element, element)] == 1)
                # -- eq (19b)
                if (task_element[0], another_task_element[0]) not in pairwise_or_relation_composite_subtasks and \
                    (another_task_element[0], task_element[0]) not in pairwise_or_relation_composite_subtasks:
                    m.addConstr(M * (b_element_vars[task_element + another_task_element] - 1) <=
                                t_edge_vars[task_element] - t_edge_vars[another_task_element])

                    m.addConstr(t_edge_vars[task_element] - t_edge_vars[another_task_element] <=
                                M * b_element_vars[task_element + another_task_element] - epsilon)
    m.update()

    # using same robot (26a) and (26b)
    for robot, teccls in robot2teccl.items():
        clause = [list(group) for key, group in itertools.groupby(teccls, operator.itemgetter(0, 1))]
        num_vertex = len(task_element_component_clause_literal_node[teccls[0]])
        num_robot = type_num[ts.nodes[task_element_component_clause_literal_node[teccls[0]][0]]
        ['location_type_component_task_element'][1]]
        for l_base in clause[0]:
            for c in clause[1:]:
                for l in c:
                    for vertex in range(num_vertex):
                        v = task_element_component_clause_literal_node[l_base][vertex]
                        w = task_element_component_clause_literal_node[l][vertex]
                        for k in range(num_robot):
                            m.addConstr(quicksum(x_vars[(p, w, k)] for p in ts.predecessors(w))
                                        + M * (c_vars[l[:-1]] - 1) <=
                                        quicksum(x_vars[(p, v, k)] for p in ts.predecessors(v))
                                        + M * (1 - c_vars[l_base[:-1]]))

                            m.addConstr(quicksum(x_vars[(p, v, k)] for p in ts.predecessors(v))
                                        + M * (c_vars[l_base[:-1]] - 1) <=
                                        quicksum(x_vars[(p, w, k)] for p in ts.predecessors(w))
                                        + M * (1 - c_vars[l[:-1]]))

    m.update()
    # focus on label
    for (task, element) in reduced_task_network:
        pruned_subgraph = task_hierarchy[task].buchi_graph
        element2edge = task_hierarchy[task].element2edge
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']

        edge_constraints(m, ts, task_vars, x_vars, t_vars, c_vars, t_edge_vars, b_element_vars, task, element, self_loop_label,
                         edge_label, task_element_component_clause_literal_node, type_num, M, epsilon,
                         pruned_subgraph, strict_larger_task_element, incomparable_task_element)

    #     if self_loop_label and self_loop_label != '1':
    #         self_loop_constraints(m, ts, x_vars, t_vars, c_vars, t_edge_vars, b_element_vars, b_immediate_element_vars,
    #                               element, self_loop_label, strict_larger_task_element, incomparable_task_element, poset_relation,
    #                               task_element_component_clause_literal_node, type_num, M, buchi, pruned_subgraph,
    #                               element2edge)

        # activation of the next subtask
        activate_next(m, ts, task_vars, x_vars, t_vars, c_vars, t_edge_vars, b_element_vars, b_immediate_element_vars,
                      task, element, self_loop_label, strict_larger_task_element, incomparable_task_element, reduced_task_network,
                      task_element_component_clause_literal_node, type_num, M, task_hierarchy, composite_subtasks)
    # activation of the first subtask
    activation_first(m, ts, x_vars, t_vars, c_vars, b_element_vars,
                     task_element_component_clause_literal_node, type_num, M, task_hierarchy,
                     maximal_task_element, b_maximal_element_vars, init_type_robot_node)

    expr = LinExpr([0.7 * ts.edges[tuple(index[:2])]['weight'] for index in x_vars.keys()], list(x_vars.values()))
    expr.add(LinExpr([0.3] * len([key for key in t_edge_vars.keys()]),
                     [value for key, value in t_edge_vars.items()]))
    m.setObjective(expr, GRB.MINIMIZE)
    if not show:
        m.Params.OutputFlag = 0
    # m.Params.MIPGap = 0.1
    m.update()
    if show:
        print('# of variables: {0}'.format(m.NumVars))
        print('# of constraints: {0}'.format(m.NumConstrs))

    m.optimize()

    if m.status == GRB.Status.OPTIMAL:
        if show:
            print('Optimal objective: %g' % m.objVal)
    elif m.status == GRB.Status.INF_OR_UNBD:
        print('Model is infeasible or unbounded')
    elif m.status == GRB.Status.INFEASIBLE:
        if show:
            print_red_on_cyan('Model is infeasible')
    elif m.status == GRB.Status.UNBOUNDED:
        print('Model is unbounded')
    else:
        print('Optimization ended with status %d' % m.status)
    if m.status != GRB.Status.OPTIMAL:
        return None, None, None, None, None, None, None, None

    goal = 0
    for index in x_vars.keys():
        goal += ts.edges[tuple(index[:2])]['weight'] * x_vars[index].x
    if show:
        print('obj:%g' % goal)

    id2robots = dict()

    get_same_robot(id2robots, robot2teccl, x_vars, task_element_component_clause_literal_node, type_num, ts)

    # obtain the time axis
    time_task_element_axis = get_axis(t_edge_vars)

    robot_waypoint, robot_time, robot_label, robot_waypoint_axis, robot_time_axis \
        = get_waypoint(x_vars, t_vars, ts, init_type_robot_node, time_task_element_axis)

    # extract the run
    
    # acpt_run = run(task_hierarchy, time_task_element_axis, composite_subtasks, \
    #     {'x': x_vars, 'c': c_vars, 't': t_edge_vars}, task_element_component_clause_literal_node, ts, type_num,
    #     dict())
    acpt_run = dict()
    for (task, var) in t_edge_vars.items():
        print("{0}: {1}".format(task, var.x))
    return robot_waypoint, robot_time, id2robots, robot_label, robot_waypoint_axis, robot_time_axis, \
           time_task_element_axis, acpt_run


def create_variables(m, ts, task_network, task_hierarchy, type_num, maximal_element, composite_subtasks):
    # task variable 
    task_vars = {}
    for task in composite_subtasks.keys():
        task_vars.update(m.addVars([task], vtype=GRB.BINARY))
    m.update()

    # clause variable, (element, 0|1, clause_index)
    c_vars = {}
    for (task, element) in task_network:
        pruned_subgraph = task_hierarchy[task].buchi_graph
        element2edge = task_hierarchy[task].element2edge
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']
        if self_loop_label and self_loop_label != '1':
            c_vars.update(m.addVars([(task, element)], [0], list(range(len(self_loop_label))), vtype=GRB.BINARY))
        if edge_label != '1':
            c_vars.update(m.addVars([(task, element)], [1], list(range(len(edge_label))), vtype=GRB.BINARY))

    m.update()

    # time variable (node, robot_index, -|+)
    t_vars = {}
    for node in ts.nodes():
        # vars for self loop
        if ts.nodes[node]['location_type_component_task_element'][2] == 0:
            t_vars.update(
                m.addVars([node], list(range(type_num[ts.nodes[node]['location_type_component_task_element'][1]])), [0, 1],
                          vtype=GRB.INTEGER))
        # vars for edge
        else:
            t_vars.update(
                m.addVars([node], list(range(type_num[ts.nodes[node]['location_type_component_task_element'][1]])), [1],
                          vtype=GRB.INTEGER))
    m.update()

    # routing variable (node_i, node_j, robot_index)
    x_vars = {}
    for edge in ts.edges():
        x_vars.update(m.addVars([edge[0]], [edge[1]],
                                list(range(type_num[ts.nodes[edge[1]]['location_type_component_task_element'][1]])),
                                vtype=GRB.BINARY))
    m.update()

    # edge time variable (element)
    # even if the edge label is '1'
    t_edge_vars = {}
    for task_element in task_network:
        t_edge_vars.update(m.addVars([task_element], vtype=GRB.INTEGER))
    m.update()

    # binary relation between edge time (element, element)
    b_element_vars = {}
    for task_element in task_network:
        for another_task_element in task_network:
            if task_element != another_task_element:
                b_element_vars.update(m.addVars([task_element], [another_task_element], vtype=GRB.BINARY))
    m.update()

    # binary relation between edge time (element, element) to indicate whether two subtasks are consecutive
    b_immediate_element_vars = {}
    for task_element in task_network:
        for another_task_element in task_network:
            if task_element != another_task_element:
                b_immediate_element_vars.update(m.addVars([task_element], [another_task_element], vtype=GRB.BINARY))
    m.update()

    b_maximal_element_vars = {}
    for task_element in maximal_element:
        b_maximal_element_vars.update(m.addVars([task_element], vtype=GRB.INTEGER))
    m.update()

    return task_vars, x_vars, t_vars, c_vars, t_edge_vars, b_element_vars, b_immediate_element_vars, b_maximal_element_vars


def initial_constraints(m, x_vars, t_vars, ts, init_type_robot_node, type_num):
    # create initial constraints
    # nodes for initial locations -- eq (5)
    for type_robot, node in init_type_robot_node.items():
        m.addConstr(1 >= quicksum(x_vars[(node, s, type_robot[1])] for s in ts.successors(node)))  # 5a
        m.addConstr(0 == quicksum(x_vars[(node, s, k)] for s in ts.successors(node) for k in
                                  range(type_num[ts.nodes[s]['location_type_component_task_element'][1]])  # 5b
                                  if k != type_robot[1]))
        # initial time -- eq (7)
        for k in range(type_num[type_robot[0]]):
            m.addConstr(t_vars[(node, k, 1)] == 0)
    m.update()

def task_constraints(m, task_vars, composite_subtasks):
    for (_, or_subtasks) in composite_subtasks.items():
        for subtasks in or_subtasks.or_composite_subtasks:
            print(subtasks)
            m.addConstr(quicksum([task_vars[subtask] for subtask in subtasks]) == 1)
    for task in composite_subtasks.keys():
        exist_or_relation = False        
        for (_, or_subtasks) in composite_subtasks.items():
            for subtasks in or_subtasks.or_composite_subtasks:
                if task in subtasks:
                    exist_or_relation = True
                    break
            if exist_or_relation:
                break
        if not exist_or_relation:
            print(task)
            m.addConstr(task_vars[task] == 1)
            
    m.update()

def one_clause_true(m, task_vars, c_vars, task, element, component, label, strict_larger_task_element, incomparable_task_element, buchi):
    """
    only one clause is true
    """
    strict_incmp = strict_larger_task_element[(task, element)] + incomparable_task_element[(task, element)]
    z = len(strict_incmp) - 1
    # first subtask, the vertex label is not satisfied by the initial robot locations
    if z == -1 and component == 0 and not buchi.sat_vertex:
        m.addConstr(quicksum(c_vars[(task, element, component, c)] for c in range(len(label))) == 0)
        m.update()
        return
    m.addConstr(quicksum(c_vars[(task, element, component, c)] for c in range(len(label))) == task_vars[task])
    m.update()


def literal_clause(m, x_vars, c_vars, task, element, component, label, ts, type_num, clause_nodes, c):
    # encode the relation between clause and its literals -- eq (10)
    expr_literal = quicksum(
        x_vars[(p, i, k)] for literal_nodes in clause_nodes for i in literal_nodes for p in ts.predecessors(i)
        for k in range(type_num[ts.nodes[i]['location_type_component_task_element'][1]]))
    mult = sum([l[2] for l in label[c]])
    m.addConstr(expr_literal / mult == c_vars[(task, element, component, c)])
    m.update()


def network_schedule_constraints(m, ts, x_vars, t_vars, init_type_robot_node, incomparable_task_element, larger_task_element,
                                 type_num, M, epsilon):
    # focus on nodes
    for i in ts.nodes():
        if i not in init_type_robot_node.values() and list(ts.predecessors(i)):  # the predecessor of i
            # each (replica) location represented by node is visited by at most one robot of type k -- eq (3)
            m.addConstr(quicksum(x_vars[(p, i, k)] for p in ts.predecessors(i)
                                 for k in range(type_num[ts.nodes[i]['location_type_component_task_element'][1]])) <= 1)

            for k in range(type_num[ts.nodes[i]['location_type_component_task_element'][1]]):
                # routing network that the visitation of a robot to a location i
                # is the precondition for its departure from that location -- eq (4)
                m.addConstr(quicksum(x_vars[(p, i, k)] for p in ts.predecessors(i)) >=
                            quicksum(x_vars[(i, s, k)] for s in ts.successors(i)))
                # t_ik = 0 if location i is not visited by the robot k -- eq (6)
                m.addConstr(t_vars[(i, k, 1)] <= M * quicksum(x_vars[(p, i, k)] for p in ts.predecessors(i)))
                if ts.nodes[i]['location_type_component_task_element'][2] == 0:
                    m.addConstr(t_vars[(i, k, 0)] <= M * quicksum(x_vars[(p, i, k)] for p in ts.predecessors(i)))

                # The time of robot k visiting p should be larger than that of the same robot
                # visiting i if there is a plan from p to i -- eq (8)
                i_task_element = tuple(ts.nodes[i]['location_type_component_task_element'][3:])
                for p in ts.predecessors(i):
                    p_task_element = tuple(ts.nodes[p]['location_type_component_task_element'][3:])
                    # incomparable elements, using epsilon to avoid loop  (8b)
                    if p_task_element in incomparable_task_element[i_task_element]:
                        if ts.nodes[i]['location_type_component_task_element'][2] == 0:
                            m.addConstr(
                                t_vars[(p, k, 1)] + (epsilon + ts.edges[(p, i)]['weight']) * x_vars[(p, i, k)] <=
                                t_vars[(i, k, 0)] + M * (1 - x_vars[(p, i, k)]))
                        else:
                            m.addConstr(
                                t_vars[(p, k, 1)] + (epsilon + ts.edges[(p, i)]['weight']) * x_vars[(p, i, k)] <=
                                t_vars[(i, k, 1)] + M * (1 - x_vars[(p, i, k)]))
                    # precedent elements, include initial element -1   (8a)
                    elif p_task_element in larger_task_element[i_task_element] + [(-1, )]:
                        if ts.nodes[i]['location_type_component_task_element'][2] == 0:
                            m.addConstr(
                                t_vars[(p, k, 1)] + ts.edges[(p, i)]['weight'] * x_vars[(p, i, k)] <=
                                t_vars[(i, k, 0)] + M * (1 - x_vars[(p, i, k)]))
                        else:
                            m.addConstr(
                                t_vars[(p, k, 1)] + ts.edges[(p, i)]['weight'] * x_vars[(p, i, k)] <=
                                t_vars[(i, k, 1)] + M * (1 - x_vars[(p, i, k)]))
                    # same element, i corresponds to the edge, p corresponds to the vertex (8a)
                    elif p_task_element == i_task_element:
                        m.addConstr(
                            t_vars[(p, k, 1)] + ts.edges[(p, i)]['weight'] * x_vars[(p, i, k)] <=
                            t_vars[(i, k, 1)] + M * (1 - x_vars[(p, i, k)]))

    m.update()


def edge_constraints(m, ts, task_vars, x_vars, t_vars, c_vars, t_edge_vars, b_element_vars, task, element, self_loop_label, edge_label,
                     task_element_component_clause_literal_node, type_num, M, epsilon, buchi,
                     strict_larger_task_element, incomparable_task_element):
    # one and only one clause is true -- eq (9)
    if edge_label != '1':

        one_clause_true(m, task_vars, c_vars, task, element, 1, edge_label, strict_larger_task_element, incomparable_task_element, buchi)

        for c in range(len(edge_label)):
            # the nodes corresponding to each clause
            clause_nodes = []  # each item is a set of literal nodes
            for l in range(len(edge_label[c])):
                clause_nodes.append(task_element_component_clause_literal_node[(task, element, 1, c, l)])

            # encode the relation between clause and its literals -- eq (10)
            literal_clause(m, x_vars, c_vars, task, element, 1, edge_label, ts, type_num, clause_nodes, c)

            # encode the synchronization constraints in terms of timing -- eq (11)
            for l in clause_nodes:
                for i in l:
                    m.addConstr(quicksum(t_vars[(clause_nodes[0][0], k, 1)]
                                         for k in range(
                        type_num[ts.nodes[clause_nodes[0][0]]['location_type_component_task_element'][1]])) ==
                                quicksum(t_vars[(i, k, 1)] for k in
                                         range(type_num[ts.nodes[i]['location_type_component_task_element'][1]])))

    m.update()

    # timing constraints between a node and its outgoing edge -- eq (13)
    strict_incmp = strict_larger_task_element[(task, element)] + incomparable_task_element[(task, element)]
    z = len(strict_incmp) - 1
    # has a self-loop with positive literals: if the first subtask, then initial locations must satisfy the self-loop
    # else if not the first subtask, no addtional constraints
    if self_loop_label and ((self_loop_label != '1' and z != -1) or
                                (self_loop_label != '1' and z == -1 and buchi.sat_vertex)):
        for c_self_loop in range(len(self_loop_label)):
            for l_self_loop in range(len(self_loop_label[c_self_loop])):
                for i in task_element_component_clause_literal_node[(element, 0, c_self_loop, l_self_loop)]:
                    m.addConstr(quicksum(t_vars[(i, k, 0)] for k in
                                         range(type_num[ts.nodes[i]['location_type_component_element'][1]]))
                                <= t_edge_vars[element])

                    m.addConstr(t_edge_vars[element] <= quicksum(t_vars[(i, k, 1)]
                                                                 for k in range(
                        type_num[ts.nodes[i]['location_type_component_element'][1]]))
                                + 1 + M * (1 - c_vars[(element, 0, c_self_loop)]))
    # the vertex label is false, or the initial robot locations satisfy the not true vertex label of initial vertex,
    # the edge label becomes true at 0, --- (14)
    if z == -1 and (not self_loop_label or (self_loop_label != '1' and not buchi.sat_vertex)):
        m.addConstr(t_edge_vars[element] == 0)

    # precedence timing constraints, between edge and previous edge, with vertex label, covered by -- eq (15)
    for another_task_element in strict_larger_task_element[(task, element)]:
        m.addConstr(t_edge_vars[another_task_element] + epsilon <= t_edge_vars[(task, element)] + M * (1 - task_vars[task]))

    m.update()


def self_loop_constraints(m, ts, task_vars, x_vars, t_vars, c_vars, t_edge_vars, b_element_vars, b_immediate_element_vars,
                          element, self_loop_label,
                          strict_larger_element, incomparable_element, poset_relation,
                          element_component_clause_literal_node, type_num, M, buchi, pruned_subgraph, element2edge):
    # one and only one clause is true, -- eq (9)
    one_clause_true(m, task_vars, c_vars, element, 0, self_loop_label, strict_larger_element, incomparable_element, buchi)

    for c in range(len(self_loop_label)):
        # the nodes corresponding to each clause
        clause_nodes = []
        for l in range(len(self_loop_label[c])):
            clause_nodes.append(element_component_clause_literal_node[(element, 0, c, l)])

        # encode the relation between clause and its literals -- eq (10)
        literal_clause(m, x_vars, c_vars, element, 0, self_loop_label, ts, type_num, clause_nodes, c)
    m.update()


def activate_next(m, ts, task_vars, x_vars, t_vars, c_vars, t_edge_vars, b_element_vars, b_immediate_element_vars,
                  task, element, self_loop_label,
                  strict_larger_task_element, incomparable_task_element, reduced_task_network,
                  task_element_component_clause_literal_node, type_num, M, task_hierarchy, composite_subtasks):
    # only one subtask
    if not b_immediate_element_vars:
        return
    task_element = (task, element)
    # subtasks that can immediately follow the current subtask
    strict_smaller_task_element = [order[1] for order in reduced_task_network.edges() if order[0] == task_element]
    # subtask can not be the last one
    if strict_smaller_task_element:
        # there exists a subtask that occurs immediately after it -- eq (16)
        m.addConstr(quicksum(b_immediate_element_vars[task_element + another_task_element]
                             for another_task_element in strict_smaller_task_element + incomparable_task_element[task_element]) == task_vars[task])
    # subtask can be the last one -- eq (20)
    else:
        z = len(incomparable_task_element[task_element])
        m.addConstr(quicksum(b_immediate_element_vars[task_element + another_task_element]
                             for another_task_element in incomparable_task_element[task_element]) <= 1)
        m.addConstr(z - quicksum(b_element_vars[task_element + another_task_element]
                                 for another_task_element in incomparable_task_element[task_element]) -
                    M * quicksum(b_immediate_element_vars[task_element + another_task_element]
                                 for another_task_element in incomparable_task_element[task_element]) <= 0)

        m.addConstr(quicksum(b_immediate_element_vars[task_element + another_task_element]
                             for another_task_element in incomparable_task_element[task_element]) - M *
                    (z - quicksum(b_element_vars[task_element + another_task_element]
                                  for another_task_element in incomparable_task_element[task_element])) <= 0)
    m.update()

    # if it can not be the first subtask to be completed, then it has to immediately follow one subtask -- eq (21)
    if strict_larger_task_element[task_element]:
        m.addConstr(quicksum(b_immediate_element_vars[another_task_element + task_element]
                             for another_task_element in
                             strict_larger_task_element[task_element] + incomparable_task_element[task_element]) == task_vars[task])
    else:
        # subtask can be the first one -- eq (22)
        m.addConstr(quicksum(b_immediate_element_vars[another_task_element + task_element]
                             for another_task_element in incomparable_task_element[task_element]) <= 1)

        m.addConstr(quicksum(b_element_vars[task_element + another_task_element]
                             for another_task_element in incomparable_task_element[task_element]) -
                    M * quicksum(b_immediate_element_vars[another_task_element + task_element]
                                 for another_task_element in incomparable_task_element[task_element]) <= 0)
        m.addConstr(quicksum(b_immediate_element_vars[another_task_element + task_element]
                             for another_task_element in incomparable_task_element[task_element]) -
                    M * quicksum(b_element_vars[task_element + another_task_element]
                                 for another_task_element in incomparable_task_element[task_element]) <= 0)

    # if occurs immediately after subtask, then it should be completed after the current subtask -- eq (17)
    # If subtask e is not the last subtask, there should be a subtask  immediately after e
    for another_task_element in strict_smaller_task_element + incomparable_task_element[task_element]:
        m.addConstr(t_edge_vars[task_element] + 1 <= t_edge_vars[another_task_element] +
                    M * (1 - b_immediate_element_vars[task_element + another_task_element]))

    #     # at most one time instant later than the completion of the current subtask -- eq (18)
    #     another_pruned_subgraph = task_hierarchy[another_task_element[0]].buchi_graph
    #     another_element2edge = task_hierarchy[another_task_element[0]].element2edge
    #     self_loop_label_another_ele = another_pruned_subgraph.nodes[another_element2edge[another_task_element[1]][0]]['label']
    #     if self_loop_label_another_ele and self_loop_label_another_ele != '1':
    #         for c_self_loop in range(len(self_loop_label_another_ele)):
    #             for l_self_loop in range(len(self_loop_label_another_ele[c_self_loop])):
    #                 for i in task_element_component_clause_literal_node[another_task_element + (0, c_self_loop, l_self_loop)]:
    #                     m.addConstr(quicksum(t_vars[(i, k, 0)] for k in
    #                                          range(type_num[ts.nodes[i]['location_type_component_task_element'][1]]))
    #                                 <= t_edge_vars[task_element] + 1
    #                                 + M * (1 - b_immediate_element_vars[task_element + another_task_element]))

    # m.update()


def activation_first(m, ts, x_vars, t_vars, c_vars, b_element_vars,
                     task_element_component_clause_literal_node, type_num, M, task_hierarchy,
                     maximal_task_element, b_maximal_element_vars, init_type_robot_node):
    #  the first completed subtask has a self-loop then the vertex label should be activated at time 0 -- eq (23)
    sat_vertex = False
    if sat_vertex:
        for (task, element) in maximal_task_element:
            pruned_subgraph = task_hierarchy[task].buchi_graph
            element2edge = task_hierarchy[task].element2edge
            self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
            if self_loop_label and self_loop_label != '1':
                for c in range(len(self_loop_label)):
                    for l in range(len(self_loop_label[c])):
                        for j in task_element_component_clause_literal_node[(task, element, 0, c, l)]:
                            m.addConstr(quicksum(t_vars[(j, k, 0)] for k in range(type_num[ts.nodes[j]
                            ['location_type_component_task_element'][1]])) <=
                                        M * (quicksum(b_element_vars[(task, element) + another_task_element]
                                                        for another_task_element in maximal_task_element if
                                                        another_task_element != (task, element)) +
                                                1 - c_vars[(element, 0, c)]))

    m.update()
    # -- eq (24)
    for task_element in maximal_task_element:
        m.addConstr(quicksum(b_element_vars[task_element + another_task_element] for another_task_element in maximal_task_element
                             if another_task_element != task_element) - M * (1 - b_maximal_element_vars[task_element]) <= 0)
        m.addConstr((1 - b_maximal_element_vars[task_element]) - M * quicksum(b_element_vars[task_element + another_task_element]
                                                                         for another_task_element in maximal_task_element
                                                                         if another_task_element != task_element) <= 0)
    m.update()

    # the constraints regarding the categories of leaving vertices -- eq (25)
    for (task, element) in maximal_task_element:
        pruned_subgraph = task_hierarchy[task].buchi_graph
        element2edge = task_hierarchy[task].element2edge
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        if self_loop_label and self_loop_label != '1':
            for c in range(len(self_loop_label)):
                clause_nodes = []
                for l in range(len(self_loop_label[c])):
                    clause_nodes.append(task_element_component_clause_literal_node[(task, element, 0, c, l)])
            # -- eq (25a)
            if sat_vertex:
                m.addConstr(quicksum(x_vars[(p, i, k)] for literal_nodes in clause_nodes for i in literal_nodes
                                     for k in range(type_num[ts.nodes[i]['location_type_component_task_element'][1]])
                                     for p in ts.predecessors(i) if p not in init_type_robot_node.values()) <=
                            M * (1 - b_maximal_element_vars[(task, element)]))
            # -- eq (25b)
            m.addConstr(quicksum(x_vars[(p, i, k)] for literal_nodes in clause_nodes for i in literal_nodes
                                 for k in range(type_num[ts.nodes[i]['location_type_component_task_element'][1]])
                                 for p in ts.predecessors(i) if p in init_type_robot_node.values()) <=
                        M * b_maximal_element_vars[(task, element)])

        m.update()


def get_waypoint(x_vars, t_vars, ts, init_type_robot_node, time_task_element_type_robot_axis):
    """
    extract the high-level plan
    """
    robot_waypoint = dict()
    robot_time = dict()
    robot_label = dict()

    robot_waypoint_axis = dict()
    robot_time_axis = dict()
    for type_robot, node in init_type_robot_node.items():
        path = [node]
        time = [0]
        label = [0]
        is_found = True
        while is_found:
            pre = path[-1]
            for s in ts.succ[pre]:
                if round(x_vars[(pre, s, type_robot[1])].x) == 1:
                    try:
                        time.append(round(t_vars[(s, type_robot[1], 0)].x))
                        path.append(s)
                        label.append(ts.nodes[s]['location_type_component_task_element'][2])
                    except KeyError:
                        pass
                    time.append(round(t_vars[(s, type_robot[1], 1)].x))
                    path.append(s)
                    # literal, clause, component, element
                    label.append(ts.nodes[s]['location_type_component_task_element'][2])
                    break
            if path[-1] == pre:
                is_found = False
        # one-to-one correspondence between the waypoint and time
        robot_waypoint[type_robot] = [ts.nodes[point]['location_type_component_task_element'][0] for point in path]
        robot_time[type_robot] = time
        robot_label[type_robot] = label
        # one-to-one correspondence between the waypoint and time that aligns with the time axis
        robot_waypoint_axis[type_robot] = [ts.nodes[point]['location_type_component_task_element'][0] for point in path
                                           if ts.nodes[point]['location_type_component_task_element'][2] == 1
                                           and len(ts.nodes[point]['location_type_component_task_element']) > 4]

        robot_time_axis[type_robot] = [time_task_element[0]
                                       for point in path for time_task_element in time_task_element_type_robot_axis
                                       if ts.nodes[point]['location_type_component_task_element'][2] == 1
                                       and time_task_element[1] == tuple(ts.nodes[point]['location_type_component_task_element'][3:])]
        
        robot_task_element = [time_task_element[1]
                                       for point in path for time_task_element in time_task_element_type_robot_axis
                                       if ts.nodes[point]['location_type_component_task_element'][2] == 1
                                       and time_task_element[1] == tuple(ts.nodes[point]['location_type_component_task_element'][3:])]
        for time_task_element_type_robot in time_task_element_type_robot_axis:
            if time_task_element_type_robot[1] in robot_task_element:
                time_task_element_type_robot.append(type_robot)
        
    return robot_waypoint, robot_time, robot_label, robot_waypoint_axis, robot_time_axis


def get_same_robot(id2robots, robot2teccl, x_vars, task_element_component_clause_literal_node, type_num, ts):
    for robot, teccls in robot2teccl.items():
        robots = []
        num_vertex = len(task_element_component_clause_literal_node[teccls[0]])
        num_robot = type_num[ts.nodes[tuple(task_element_component_clause_literal_node[teccls[0]][0:2])]
        ['location_type_component_task_element'][1]]
        # detemine the teccl that the corresponding vertices are visited
        vertices = None
        for teccl in teccls:
            vertices = task_element_component_clause_literal_node[teccl]
            if sum([x_vars[(p, vertices[0], k)].x for p in ts.predecessors(vertices[0]) for k in range(num_robot)]) > 0:
                break
        for vertex in range(num_vertex):
            v = vertices[vertex]
            for k in range(num_robot):
                if sum([x_vars[(p, v, k)].x for p in ts.predecessors(v)]) == 1:
                    robots.append(k)
                    break
        id2robots[robot] = robots


def get_axis(t_edge_vars):
    time_axis = [[round(t_edge_vars[edge_t].x), edge_t] for edge_t in t_edge_vars.keys()]
    time_axis.sort()

    offset = 0
    value = time_axis[0][0]
    for i in range(1, len(time_axis)):
        # skip for subtasks for which time is 0.0 due to there eixsts subtasks that have or relation to them
        if (time_axis[i][0] == 0.0):
            continue
        if time_axis[i][0] == value:
            offset += 1
            time_axis[i][0] = time_axis[i][0] + offset
        else:
            value = time_axis[i][0]
            time_axis[i][0] = time_axis[i][0] + offset
    return time_axis
