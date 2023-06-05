import networkx as nx
import random
import itertools
import sympy
from gurobipy import *


def isEquivalent(expr1, expr2):
    return sympy.simplify_logic((expr1 & (~expr2)) | ((~expr1) & expr2)) == False


# ------------- get accepted runs using the waypoint only ----------------
def run(graph, time_axis, initial, element2edge, var, element_component_clause_literal_node, ts, type_num,
        type_robot_label, buchi, show, last_subtask=None, loop=False):
    """
    the accepting run incurred by the path
    """

    frontier = [[initial, -1, []]]
    # iterate until the accepting state is reached
    while True:
        if show:
            print([f[0] for f in frontier])
        node, clock, acpt_run_ = frontier.pop()

        # Determine the set of identical time instants
        instant_element = time_axis[clock + 1]
        if acpt_run_:
            pre_neg_edge = acpt_run_[-1]['neg_edge']
        else:
            pre_neg_edge = []
        # loop over each successor to see whether progress can be made
        for succ in graph.succ[node]:
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
                                           type_robot_label, last_subtask, buchi, [], loop)

                essential_clause_vertex, neg_clause_vertex, exe_robots_vertex \
                    = determine_essentials(instant_element, var, graph.nodes[node]['label'],
                                           graph.nodes[node]['neg_label'], 0,
                                           element_component_clause_literal_node, ts, type_num, dict(),
                                           last_subtask, buchi,
                                           pre_neg_edge, loop)

                # clock, the exact time when transition occurs
                acpt_run = acpt_run_.copy()  # copy the history
                acpt_run.append({'subtask': (node, succ), 'time_element': time_axis[clock + 1],
                                 'essential_robot_edge': exe_robots_edge,
                                 'essential_clause_edge': essential_clause_edge, 'neg_edge': neg_clause_edge,
                                 'essential_robot_vertex': exe_robots_vertex,
                                 'neg_vertex': neg_clause_vertex})

                # stop when accept is reached
                if 'accept' in succ:
                    return acpt_run
                # clock + 1, after reaching succ, the immediate time clock that should be verified
                frontier.append([succ, clock + 1, acpt_run])


def determine_essentials(instant_element, var, label, neg_label, component,
                         element_component_clause_literal_node, ts, type_num,
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
                        if buchi.ap_sat_label('1', [clause]):
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
