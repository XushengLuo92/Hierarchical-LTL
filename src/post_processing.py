import networkx as nx
import random
import itertools
import sympy
from gurobipy import *


def isEquivalent(expr1, expr2):
    return sympy.simplify_logic((expr1 & (~expr2)) | ((~expr1) & expr2)) == False

# ------------- get accepted runs using the waypoint only ----------------
def run_with_t_edge(task_hierarchy, time_task_element_axis, var, task_element_component_clause_literal_node, ts, type_num,
        type_robot_label, last_subtask=None, loop=False, show=False):
    """
    the accepting run incurred by the path
    """
   
    # ind the index of the first non-zero element
    first_non_zero_index = next((index for index, item in enumerate(time_task_element_axis) if item[0] != 0.0), None)
    
    acpt_run = []
    # iterate until the accepting state is reached
    for i in range(first_non_zero_index, len(time_task_element_axis)):
        if show:
            print(time_task_element_axis[i])

        # Determine the set of identical time instants
        time_instant_task_element = time_task_element_axis[i]
        if acpt_run:
            pre_neg_edge = acpt_run[-1]['neg_edge']
        else:
            pre_neg_edge = []
        # loop over each successor to see whether progress can be made

        # print((node, succ), graph.edges[(node, succ)]['formula'])
        # whether the collection of paths at clock satisfies the edge label
        # neg_literal: negative clause that needs to be addressed
        # exe_robot: set of robots that takes the subtask with nonzero id

        task = time_instant_task_element[1][0]
        element = time_instant_task_element[1][1]
        buchi_graph = task_hierarchy[task].buchi_graph
        element2edge = task_hierarchy[task].element2edge
     
        essential_clause_edge, neg_clause_edge, exe_robots_edge \
                    = determine_essentials(time_instant_task_element, var, buchi_graph.edges[element2edge[element]]['label'],
                                           buchi_graph.edges[element2edge[element]]['neg_label'], 1,
                                           task_element_component_clause_literal_node, ts, type_num,
                                           type_robot_label, last_subtask, buchi_graph, [], loop)
            
        essential_clause_vertex, neg_clause_vertex, exe_robots_vertex \
            = determine_essentials(time_instant_task_element, var, buchi_graph.nodes[element2edge[element][0]]['label'],
                                    buchi_graph.nodes[element2edge[element][0]]['neg_label'], 0,
                                    task_element_component_clause_literal_node, ts, type_num, dict(),
                                    last_subtask, buchi_graph, pre_neg_edge, loop)

        acpt_run.append({'subtask': time_instant_task_element[1], 'time_element': time_instant_task_element[0],
                            'essential_robot_edge': exe_robots_edge,
                            'essential_clause_edge': essential_clause_edge, 'neg_edge': neg_clause_edge,
                            'essential_robot_vertex': exe_robots_vertex,
                            'neg_vertex': neg_clause_vertex})
    return acpt_run



# ------------- get accepted runs using the waypoint only ----------------
def run(task_hierarchy, time_task_element_axis, composite_subtasks, var, task_element_component_clause_literal_node, ts, type_num,
        type_robot_label, last_subtask=None, loop=False):
    """
    the accepting run incurred by the path
    """
    sorted_t_edge = list(sorted(var['t'].items(), key=lambda item: item[1].x))
    initial = next((item for item in sorted_t_edge if item[1].x != 0.0), None)
    ap_up_to_now = {}
    initial = {}
    for (task, hierarchy) in task_hierarchy.items():
        buchi =  hierarchy.buchi_graph
        init_state = buchi.graph['init']
        initial[task] = init_state
    
    frontier = [[initial, -1, {}]]
    # iterate until the accepting state is reached
    while True:
        # if show:
        #     print([f[0] for f in frontier])
        taskwise_node, clock, taskwise_acpt_run = frontier.pop()
        print('taskwise_node: ', taskwise_node)
        print('taskwise_acpt_run: ', taskwise_acpt_run)

        # Determine the set of identical time instants
        time_instant_task_element = time_task_element_axis[clock + 1]
        # if taskwise_acpt_run_:
        #     pre_neg_edge = taskwise_acpt_run_[-1]['neg_edge']
        # else:
        #     pre_neg_edge = []
        # loop over each successor to see whether progress can be made
        task = time_instant_task_element[1][0]
        element = time_instant_task_element[1][1]
        buchi_graph = task_hierarchy[task].buchi_graph
        element2edge = task_hierarchy[task].element2edge
        found_equivalent_subtask = False
        for succ in buchi_graph.succ[taskwise_node[task]]:
            # equivalent subtask
            if buchi_graph.edges[element2edge[element]]['formula'] == buchi_graph.edges[(taskwise_node[task], succ)]['formula'] and \
                            buchi_graph.nodes[element2edge[element][0]]['formula'] == buchi_graph.nodes[taskwise_node[task]]['formula']:
                found_equivalent_subtask = True
                # if isEquivalent(graph.edges[element2edge[instant_element[1]]]['formula'], graph.edges[(node, succ)]['formula']) and \
                #         isEquivalent(graph.nodes[element2edge[instant_element[1]][0]]['formula'], graph.nodes[node]['formula']):

                # print((node, succ), graph.edges[(node, succ)]['formula'])
                # whether the collection of paths at clock satisfies the edge label
                # neg_literal: negative clause that needs to be addressed
                # exe_robot: set of robots that takes the subtask with nonzero id

                essential_clause_edge, neg_clause_edge, exe_robots_edge \
                    = determine_essentials(time_instant_task_element, var, buchi_graph.edges[(taskwise_node[task], succ)]['label'],
                                           buchi_graph.edges[(taskwise_node[task], succ)]['neg_label'], 1,
                                           task_element_component_clause_literal_node, ts, type_num,
                                           type_robot_label, last_subtask, buchi_graph, [], loop)
                true_ap = ['_'.join(str(item) for item in sublist) for sublist in essential_clause_edge]
                for ap in true_ap:
                    ap_up_to_now[sympy.symbols(ap)] = True
                # essential_clause_vertex, neg_clause_vertex, exe_robots_vertex \
                #     = determine_essentials(instant_element, var, graph.nodes[node]['label'],
                #                            graph.nodes[node]['neg_label'], 0,
                #                            element_component_clause_literal_node, ts, type_num, dict(),
                #                            last_subtask, buchi,
                #                            pre_neg_edge, loop)

                # clock, the exact time when transition occurs
                acpt_run = taskwise_acpt_run.copy()  # copy the history
                info = {'subtask': (taskwise_node[task], succ), 'time_element': time_task_element_axis[clock + 1],
                                 'essential_robot_edge': exe_robots_edge,
                                 'essential_clause_edge': essential_clause_edge, 'neg_edge': neg_clause_edge}
                if task not in acpt_run.keys():
                    acpt_run[task] = [info]
                else:
                    acpt_run[task].append(info)
                                #  'essential_robot_vertex': exe_robots_vertex,
                                #  'neg_vertex': neg_clause_vertex})
                node = taskwise_node.copy()
                node[task] = succ

                # stop when accept is reached
                accept_in_succ = 'accept' in succ
                if task == 'p0' and accept_in_succ:
                    print('taskwise_node: ', node)
                    print('taskwise_acpt_run: ', acpt_run)
                    print('ap_up_to_now:', ap_up_to_now)
                    return acpt_run
                curr_task = task
                while accept_in_succ:
                    ap_up_to_now[sympy.symbols(curr_task + '_1_1_0')] = True
                    found_parent = False
                    for (t, composite_subtask) in composite_subtasks.items():
                        # curr_task is a subtask of considered task
                        if curr_task in composite_subtask.subtask2element.keys():
                            curr_graph = task_hierarchy[t].buchi_graph
                            curr_node = taskwise_node[t]
                            for succ in curr_graph.succ[curr_node]:
                                curr_task_symbol = sympy.symbols(curr_task + '_1_1_0')
                                # Define an assignment
                                assignment = {curr_task_symbol: True}
                                # Evaluate the expression with the assignment
                                satisfies = curr_graph.edges[(curr_node, succ)]['formula'].subs(assignment)
                                if satisfies:
                                    node[t] = succ
                                    if t not in acpt_run.keys():
                                        acpt_run[t] = [{'subtask': (curr_node, succ), 'time_element': time_task_element_axis[clock + 1]}]
                                    else:
                                        acpt_run[t].append({'subtask': (curr_node, succ), 'time_element': time_task_element_axis[clock + 1]})
                                    node[t] = succ
                                    found_parent = True
                                    accept_in_succ = 'accept' in succ
                                    curr_task = t
                                    if t == 'p0' and accept_in_succ:
                                        print('taskwise_node: ', node)
                                        print('taskwise_acpt_run: ', acpt_run)
                                        print('ap_up_to_now:', ap_up_to_now)
                                        return acpt_run
                                    break
                        if found_parent:
                            break
                # clock + 1, after reaching succ, the immediate time clock that should be verified
                frontier.append([node, clock + 1, acpt_run])
                break
        if not found_equivalent_subtask:
            hass_nodes = hierarchy.hass_graphs[0][2]
            for succ in buchi_graph.succ[taskwise_node[task]]:
                if succ == 'T2_S20':
                    continue
                # TODO set of aps that are ture up to now
                # label = buchi_graph.edges[(taskwise_node[task], succ)]['label']
                # sublists = []
                # for sublist in label:
                #     sublist_str = ' & '.join('_'.join(str(item) for item in inner_list) for inner_list in sublist)
                #     sublists.append(f"({sublist_str})")
                # # Combine sublists with pipe operator
                # result = ' | '.join(sublists)
                satisfies = buchi_graph.edges[(taskwise_node[task], succ)]['formula'].subs(ap_up_to_now)
                if  satisfies:
                    acpt_run = taskwise_acpt_run.copy()  # copy the history
                    if task not in acpt_run.keys():
                        acpt_run[task] = [{'subtask': (taskwise_node[task], succ), 'time_element': time_task_element_axis[clock]}]
                    else:
                        acpt_run[task].append({'subtask': (taskwise_node[task], succ), 'time_element': time_task_element_axis[clock]})
                    node = taskwise_node.copy()
                    node[task] = succ
                    frontier.append([node, clock, acpt_run])
                    break

def edge_exist_in_hass_graph(element2edge, hass_nodes, queried_edge):
    exist = False
    for node in hass_nodes:
        edge = element2edge[node]
        exist = edge == queried_edge
        if exist:
            return exist
    return exist
    
    
def determine_essentials(time_instant_task_element, var, label, neg_label, component,
                         task_element_component_clause_literal_node, ts, type_num,
                         type_robot_label, last_subtask, buchi,
                         pre_neg_edge=[],
                         loop=False):
    """
    determine the essential clause and the essential robots
    """
    if label == '1':
        # negative clause that is conjunctive with satisfied positive literals
        if neg_label:
            # neg vertex label: implied by the  previous neg edge label
            if component == 0:
                for clause in neg_label:
                    # initial state with negative clause
                    if not pre_neg_edge:
                        # if buchi.ap_sat_label('1', [clause]):
                        neg_clause = clause
                        break
                    # othereise, subformula
                    if set(clause).issubset(set(pre_neg_edge)):
                        neg_clause = clause
                        break
            else:
                # randomly choose one negative clause for the edge
                neg_clause = neg_label[0]
        else:
            neg_clause = []
        return '1', neg_clause, dict()
    else:
        if label:
            essential_clause = []
            neg_clause = []
            # iterate over all elements with identical completion times
            for c, clause in enumerate(label):
                # determine the clause valued 1
                if var['c'][(time_instant_task_element[1][0], time_instant_task_element[1][1], component, c)].x == 1:
                    essential_clause = clause
                    neg_clause = neg_label[c]
                    break
            if essential_clause:
                exe_robots = {lit[0]: [] for lit in essential_clause}
                for l, lit in enumerate(essential_clause):
                    for i in task_element_component_clause_literal_node[(time_instant_task_element[1][0], time_instant_task_element[1][1], component, c, l)]:
                        for k in range(type_num[ts.nodes[i]['location_type_component_task_element'][1]]):
                            if sum([round(var['x'][(p, i, k)].x) for p in ts.predecessors(i)]) == 1:
                                exe_robots[lit[0]].append((lit[1], k))
                                break
            # extra literal added to the last subtasks in the suffix part
            else:
                # reutrn to initial locations
                if loop:
                    essential_clause = []
                    neg_clause = []
                    exe_robots = {label: [type_robot] for type_robot, label in type_robot_label.items()}
                # return to respective regions
                else:
                    essential_clause = last_subtask['essential_clause_edge']
                    neg_clause = last_subtask['neg_edge']
                    exe_robots = last_subtask['essential_robot_edge']

        # empty label
        else:
            return False, [], dict()

    return essential_clause, neg_clause, exe_robots
