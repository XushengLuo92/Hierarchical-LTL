import networkx as nx
import random
import itertools
import sympy
from gurobipy import *


def isEquivalent(expr1, expr2):
    return sympy.simplify_logic((expr1 & (~expr2)) | ((~expr1) & expr2)) == False


# ------------- get accepted runs using the waypoint only ----------------
def run(graph, time_axis, initial, element2edge, var, element_component_clause_literal_node, ts, type_num,
        is_nonempty_self_loop, type_robot_label, last_subtask=None, loop=False):
    """
    the accepting run incurred by the path
    """

    frontier = [[initial, -1, []]]
    # iterate until the accepting state is reached
    while True:
        print([f[0] for f in frontier])
        node, clock, acpt_run_ = frontier.pop()

        # Determine the set of identical time instants
        instant_element = time_axis[clock + 1]
        if acpt_run_:
            pre_essential_clause_edge = acpt_run_[-1]['essential_clause_edge']
        else:
            pre_essential_clause_edge = []
        # loop over each successor to see whether progress can be made
        for succ in graph.succ[node]:
            # initial vertex is the accepting vertex at the same time, we view these two differently
            # the successor of initial vertex does not include the arificial vertex
            if clock + 1 == 0 and succ == 'artificial':
                continue
            # # the successor of accepting vertex does not include other vertcies than the artificial vertex
            # if clock + 1 != 0 and 'accept' in node and succ != 'artificial':
            #     continue
            # equivalent subtask
            if graph.edges[element2edge[instant_element[1]]]['formula'] == graph.edges[(node, succ)]['formula'] and \
                            graph.nodes[element2edge[instant_element[1]][0]]['formula'] == graph.nodes[node]['formula']:
                # if isEquivalent(graph.edges[element2edge[instant_element[1]]]['formula'], graph.edges[(node, succ)]['formula']) and \
                #         isEquivalent(graph.nodes[element2edge[instant_element[1]][0]]['formula'], graph.nodes[node]['formula']):

                # print((node, succ), graph.edges[(node, succ)]['formula'])
                # whether the collection of paths at clock satisfies the edge label
                # neg_literal: negative clause that needs to be addressed
                # exe_robot: set of robots that takes the subtask with nonzero id

                essential_clause_edge, neg_clause_edge, exe_robots_edge \
                    = determine_essentials(instant_element, var, graph.edges[(node, succ)]['label'],
                                           graph.edges[(node, succ)]['neg_label'], 1,
                                           element_component_clause_literal_node, ts, type_num,
                                           type_robot_label)

                essential_clause_vertex, neg_clause_vertex, exe_robots_vertex \
                    = determine_essentials(instant_element, var, graph.nodes[node]['label'],
                                           graph.nodes[node]['neg_label'], 0,
                                           element_component_clause_literal_node, ts, type_num, dict(),
                                           pre_essential_clause_edge, essential_clause_edge)

                # clock, the exact time when transition occurs
                acpt_run = acpt_run_.copy()  # copy the history
                acpt_run.append({'subtask': (node, succ), 'time_element': time_axis[clock + 1],
                                 'essential_robot_edge': exe_robots_edge,
                                 'essential_clause_edge': essential_clause_edge, 'neg_edge': neg_clause_edge,
                                 'essential_robot_vertex': exe_robots_vertex,
                                 'neg_vertex': neg_clause_vertex})

                # 1: self-loop exists, stop when artificial is reached
                # 2: self-loop does not exist, stop when accept is reached
                if (is_nonempty_self_loop and 'artificial' in succ) or \
                        (not is_nonempty_self_loop and 'accept' in succ):
                    return acpt_run
                # clock + 1, after reaching succ, the immediate time clock that should be verified
                frontier.append([succ, clock + 1, acpt_run])


def determine_essentials(instant_element, var, label, neg_label, component,
                         element_component_clause_literal_node, ts, type_num, type_robot_label,
                         pre_complete_clause_formula=[],
                         cur_complete_clause_formula=[]):
    """
    determine the essential clause and the essential robots
    """
    if label == '1':
        # negative clause that is conjunctive with satisfied positive literals
        if neg_label:
            for clause in neg_label:
                # do not exclude with previous edge label
                if not_exclusion(pre_complete_clause_formula, clause):
                    neg_clause = clause
                # might as well exclude with the current edge label
                if not not_exclusion(cur_complete_clause_formula, clause):
                    neg_clause = clause
                    break
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
                if var['c'][(instant_element[1], component, c)].x == 1:
                    essential_clause = clause
                    neg_clause = neg_label[c]
                    break
            if essential_clause:
                exe_robots = {lit[0]: [] for lit in essential_clause}
                for l, lit in enumerate(essential_clause):
                    for i in element_component_clause_literal_node[(instant_element[1], component, c, l)]:
                        for k in range(type_num[ts.nodes[i]['location_type_component_element'][1]]):
                            if sum([round(var['x'][(p, i, k)].x) for p in ts.predecessors(i)]) == 1:
                                exe_robots[lit[0]].append((lit[1], k))
                                break
            # extra literal
            else:
                essential_clause = []
                neg_clause = []
                exe_robots = {label: [type_robot] for type_robot, label in type_robot_label.items()}

        # empty label
        else:
            return False, [], dict()

    return essential_clause, neg_clause, exe_robots


def not_exclusion(pre_complete_clause_formula, neg_clause):
    """
    whether two clauses are mutual exclusive
    """
    if not pre_complete_clause_formula:
        return True
    else:
        for essential_lit in pre_complete_clause_formula:
            for neg_lit in neg_clause:
                if essential_lit[0] == neg_lit[0] and essential_lit[1] == neg_lit[1] and essential_lit[2] >= neg_lit[2]:
                    return False
        return True


def determine_robots_for_next(instant_element, var, label, neg_label, element_component_clause_literal_node,
                              ts, type_num, pre_complete_clause_formula):
    """
    determine the essential robots for the vertex label of the next vertex, i.e., artificial robot
    """
    if label == '1':
        # negative clause that is conjunctive with satisfied positive literals
        if neg_label:
            for clause in neg_label:
                if not_exclusion(pre_complete_clause_formula, clause):
                    neg_clause = clause
                    break
        else:
            neg_clause = []
        return dict(), '1', neg_clause
    else:
        if label:
            essential_clause = []
            neg_clause = []
            # iterate over all elements with identical completion times
            for c, clause in enumerate(label):
                # determine the clause valued 1
                if var['c'][(instant_element[1], 0, c)].x == 1:
                    essential_clause = clause
                    neg_clause = neg_label[c]
                    break

            exe_robots = {(c, l): [] for l in range(len(essential_clause))}
            for l, lit in enumerate(essential_clause):
                for i in element_component_clause_literal_node[(instant_element[1], 0, c, l)]:
                    for k in range(type_num[ts.nodes[i]['location_type_component_element'][1]]):
                        if sum([round(var['x'][(p, i, k)].x) for p in ts.predecessors(i)]) == 1:
                            exe_robots[(c, l)].append((lit[1], k))
                            break
        # empty label
        else:
            return dict(), '1', []

    return exe_robots, essential_clause, neg_clause
