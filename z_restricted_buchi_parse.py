# -*- coding: utf-8 -*-
"""
Remove all other initial and accepting vertices given a pair of initial and accepting vertices
"""

import subprocess
import os.path
import re
import networkx as nx
import numpy as np
from networkx.classes.digraph import DiGraph
import datetime
from sympy.logic.boolalg import to_dnf, And, Equivalent


class Buchi(object):
    """
    construct buchi automaton graph
    """

    def __init__(self, task, workspace):
        """
        initialization
        :param task: task specified in LTL
        """
        # task specified in LTL
        self.formula = task.formula
        self.type_num = workspace.type_num
        self.atomic_prop = workspace.atomic_prop
        self.sat_init_edge = []
        self.sat_vertex = False
        self.regions = workspace.regions
        # graph of buchi automaton
        self.buchi_graph = DiGraph(type='buchi', init=[], accept=[])

    def construct_buchi_graph(self):
        """
        parse the output of the program ltl2ba and build the buchi automaton
        """
        # directory of the program ltl2ba
        dirname = os.path.dirname(__file__)
        # output of the program ltl2ba
        output = subprocess.check_output(dirname + "/./ltl2ba -f \"" + self.formula + "\"", shell=True).decode(
            "utf-8")

        # find all states/nodes in the buchi automaton
        state_re = re.compile(r'\n(\w+):\n\t')
        state_group = re.findall(state_re, output)

        # find initial and accepting states
        init = [s for s in state_group if 'init' in s]
        # treat the node accept_init as init node
        accept = [s for s in state_group if 'accept' in s]
        # finish the inilization of the graph of the buchi automaton
        self.buchi_graph.graph['init'] = init
        self.buchi_graph.graph['accept'] = accept

        # for each state/node, find it transition relations
        for state in state_group:
            # add node
            self.buchi_graph.add_node(state, label=[], neg_label=[], formula=to_dnf('0'))
            # loop over all transitions starting from current state
            state_if_fi = re.findall(state + r':\n\tif(.*?)fi', output, re.DOTALL)
            if state_if_fi:
                relation_group = re.findall(r':: (\(.*?\)) -> goto (\w+)\n\t', state_if_fi[0])
                for symbol, next_state in relation_group:
                    symbol = symbol.replace('||', '|').replace('&&', '&').replace('!', '~')
                    # check literals in labels to merge and delete
                    edge_label, neg_label, formula = self.merge_check_feasible(symbol)
                    if edge_label:
                        # update node, do not create edges for selfloop
                        if state == next_state:
                            self.buchi_graph.nodes[state]['label'] = edge_label
                            self.buchi_graph.nodes[state]['neg_label'] = neg_label
                            self.buchi_graph.nodes[state]['formula'] = formula
                            continue
                        # add edge if the label is not false
                        if formula:
                            self.buchi_graph.add_edge(state, next_state, label=edge_label, neg_label=neg_label,
                                                      formula=formula)
                        # print(self.buchi_graph.edges[(state, next_state)])

            else:
                state_skip = re.findall(state + r':\n\tskip\n', output, re.DOTALL)
                if state_skip:
                    self.buchi_graph.nodes[state]['label'] = '1'
                    self.buchi_graph.nodes[state]['neg_label'] = []
                    self.buchi_graph.nodes[state]['formula'] = to_dnf('1')

        # delete vertices without selfloop
        self.delete_node_no_selfloop_except_init_accept()

    def delete_node_no_selfloop_except_init_accept(self):
        """
        delete vertices without selfloop
        """
        remove_node = []
        for node in self.buchi_graph.nodes():
            # if no selfloop
            if not self.buchi_graph.nodes[node]['label']:
                # delete if not init or accept
                if 'init' not in node and 'accept' not in node:
                    remove_node.append(node)

        self.buchi_graph.remove_nodes_from(remove_node)

    def ap_sat_label(self, label, neg_label):
        """
        # whether the atomic propositions satisfy the conjunction of label and neg_label
        """
        # no positive literals
        if label == '1':
            if not neg_label:
                return True
            # satisfy the negative literals
            for clause in neg_label:
                sat = True
                # whether satisfy the clause
                for lit in clause:
                    if (lit[0], lit[1]) in self.atomic_prop.keys():
                        if self.atomic_prop[(lit[0], lit[1])] < lit[2]:
                            continue
                        else:
                            sat = False
                            break
                if sat:
                    return True
            return False

        for index, clause in enumerate(label):
            sat = True
            # satisfy the positive literals
            for lit in clause:
                if (lit[0], lit[1]) in self.atomic_prop.keys():
                    if self.atomic_prop[(lit[0], lit[1])] >= lit[2]:
                        continue
                    else:
                        sat = False
                        break
                else:
                    sat = False
                    break

            if sat:
                # satisfy the corresponding negative literals
                for lit in neg_label[index]:
                    if (lit[0], lit[1]) in self.atomic_prop.keys():
                        if self.atomic_prop[(lit[0], lit[1])] < lit[2]:
                            continue
                        else:
                            sat = False
                            break

        return sat

    def get_init_accept(self):
        """
        search the shortest path from a node to another, i.e., # of transitions in the path, then sort
        """
        init_accept = dict()
        # shortest simple path for each init vertex to accept vertex
        for init in self.buchi_graph.graph['init']:
            init_graph = self.buchi_graph.copy()
            # remove all other initial vertices
            init_graph.remove_nodes_from([node for node in self.buchi_graph.graph['init'] if node != init])
            remove_edge = []
            # no self-loop, initial node: keep the edge that robot initial locations satisfy
            if not init_graph.nodes[init]['label']:
                for n in init_graph.succ[init]:  # node is not in succ of node
                    if not self.ap_sat_label(init_graph.edges[(init, n)]['label'],
                                             init_graph.edges[(init, n)]['neg_label']):
                        remove_edge.append((init, n))
            # has self-loop, initial node: do nothing if the initial robot locations satisfy the vertex label
            # else remove any edge that initial robot locations do not satisfy
            elif not self.ap_sat_label(init_graph.nodes[init]['label'],
                                       init_graph.nodes[init]['neg_label']):

                for n in init_graph.succ[init]:
                    if not self.ap_sat_label(init_graph.edges[(init, n)]['label'],
                                                           init_graph.edges[(init, n)]['neg_label']):
                        remove_edge.append((init, n))

            init_graph.remove_edges_from(remove_edge)

            # iterate over all accepting vertices
            for accept in self.buchi_graph.graph['accept']:
                init_accept_graph = init_graph.copy()
                # remove all other accepting vertices except it is the exact init, 'accept_init'
                init_accept_graph.remove_nodes_from([node for node in self.buchi_graph.graph['accept']
                                                     if node != accept and node != init])
                # shortest simple path
                len1 = np.inf
                if init != accept:
                    try:
                        len1, _ = nx.algorithms.single_source_dijkstra(init_accept_graph,
                                                                         source=init, target=accept)
                    except nx.exception.NetworkXNoPath:
                        len1 = np.inf
                else:
                    for suc in init_accept_graph.succ[init]:
                        try:
                            length, path = nx.algorithms.single_source_dijkstra(init_accept_graph,
                                                                           source=suc, target=accept)
                        except nx.exception.NetworkXNoPath:
                            length, path = np.inf, []
                        if length + 1 < len1 and path:
                            len1 = length + 1
                # save
                init_accept[(init, accept)] = len1

        # shortest simple cycle, including self loop
        accept_accept = dict()
        for accept in self.buchi_graph.graph['accept']:
            # 0 if with self loop or without outgoing edges (co-safe formulae)
            if self.buchi_graph.nodes[accept]['formula']:
                accept_accept[accept] = 0
                continue

            acpt_graph = self.buchi_graph.copy()
            # remove other accepting vertices
            acpt_graph.remove_nodes_from([node for node in self.buchi_graph.graph['accept'] if node != accept])
            # remove initial vertices without selfloop
            acpt_graph.remove_nodes_from([node for node in self.buchi_graph.graph['init']
                                          if node in acpt_graph.nodes and not acpt_graph.nodes[node]['label']
                                          and node != accept])

            # find the shortest path back to itself
            length = np.inf
            for suc in acpt_graph.succ[accept]:
                try:
                    len1, path = nx.algorithms.single_source_dijkstra(self.buchi_graph,
                                                                   source=suc, target=accept)
                except nx.exception.NetworkXNoPath:
                    len1, path = np.inf, []
                if len1 + 1 < length and path:
                    length = len1 + 1
            accept_accept[accept] = length

        # select initial to accept
        init_acpt = {pair: length + accept_accept[pair[1]] for pair, length in init_accept.items()
                     if length != np.inf and accept_accept[pair[1]] != np.inf}
        return sorted(init_acpt.items(), key=lambda x: x[1])

    def get_next_vertex(self, accept_state):
        """
        get the next vertex after the selected accepting state
        """
        next_vertex = {}
        for suc in self.buchi_graph.succ[accept_state]:
            formula = self.buchi_graph.edges[(accept_state, suc)]['formula']
            if formula == to_dnf('1'):
                next_vertex[suc] = 0
            else:
                length = len([letter for clause in formula.__str__().split('|') for letter in clause.split('&')])
                next_vertex[suc] = length

        return sorted(next_vertex.items(), key=lambda x: x[1])

    def merge_check_feasible(self, symbol):
        """
        merge and check feasible transitions
        :return: '1', [], to_dnf('1'): if the label is (1)
                 '1', only_neg_clause, '&'.join(['~' + '_'.join([str(l) for l in lit]) for lit in neg_literals]):
                    if there is at least one clause with only negative literals, return all pure negative literals
                 pos_clause, neg_clause, formula: otherwise, one-to-one correpondence with pos_clause, neg_clause
                 [], [], False: if pos_clause is []
        """
        # [[[literal], .. ], [[literal], .. ], [[literal], .. ]]
        #       clause            clause            clause

        pos_clause = []
        neg_clause = []
        # if the label is true, only collect clause composed of negative literals
        only_neg = False
        only_neg_clause = []

        if symbol == '(1)':
            return '1', [], to_dnf('1')
        # non-empty symbol
        else:
            clause = symbol.split(' | ')
            # merge
            for c in clause:
                literals_ = c.strip('(').strip(')').split(' & ')

                # negative literals
                neg_literals_ = [l for l in literals_ if '~' in l]
                neg_literals_.sort()
                # -------- Absorption in negative literals --------
                i = 0
                neg_literals = []
                while i < len(neg_literals_):
                    curr = neg_literals_[i].split('_')
                    # make it number
                    curr[0] = curr[0][1:]
                    curr[1] = int(curr[1])
                    curr[2] = int(curr[2])
                    neg_literals.append(curr)
                    # loop over subsequent elements and remove such element that involve robots of same type and same
                    # location but with smaller number
                    j = i + 1
                    while j < len(neg_literals_):
                        subsq = neg_literals_[j].split('_')
                        # if differnent location, no need to check subsequent elements
                        if curr[0] != subsq[0][1:]:
                            break
                        # same # of robots of same type with indicator 0
                        if curr[1] == int(subsq[1]) and curr[2] <= int(subsq[2]):
                            del neg_literals_[j]
                            continue
                        j += 1
                    # next element
                    i += 1

                # -------- Absorption in positive literals ---------
                literals_ = [l for l in literals_ if '~' not in l]
                # return if only have negative literals
                if not literals_ or only_neg:
                    only_neg = True
                    if not literals_ and neg_literals:
                        only_neg_clause.append(neg_literals)
                    continue

                literals_.sort()
                literals_.reverse()
                i = 0
                literals = []
                while i < len(literals_):
                    curr = literals_[i].split('_')
                    # make it number
                    curr[1] = int(curr[1])
                    curr[2] = int(curr[2])
                    curr[3] = int(curr[3])
                    literals.append(curr)
                    # loop over subsequent elements and remove such element that involve robots of same type and same
                    # location but with smaller number
                    j = i + 1
                    while j < len(literals_):
                        subsq = literals_[j].split('_')
                        # if differnent location, no need to check subsequent elements
                        if curr[0] != subsq[0]:
                            break
                        # same # of robots of same type with indicator 0
                        if curr[1] == int(subsq[1]) and curr[2] >= int(subsq[2]) and int(subsq[3]) == 0:
                            del literals_[j]
                            continue
                        j += 1
                    # next element
                    i += 1

                # ---------- mutual exclusion ------------
                # check if the  # robots required >= # robots forbidden
                i = 0
                while i < len(literals):
                    for j in range(len(neg_literals)):
                        # don't aim at the same region
                        if int(literals[i][0][1:]) != int(neg_literals[j][0][1:]):
                            continue
                        if int(literals[i][0][1:]) == int(neg_literals[j][0][1:]) \
                                and literals[i][1] == neg_literals[j][1]:
                            if literals[i][2] >= neg_literals[j][2]:
                                literals = []
                                break
                    i += 1

                # --------- check feasibility -------------
                # literals is empty only because of mutual exclusion, otherwise it has been returned
                # below Absorption in positive literals
                if literals:
                    # --------- Mutual exclusion ----------
                    # if two literals use the same robots, infeasible eg. ['l5', 1, 1, 1] ['l4', 1, 1, 1]
                    if sum([l[3] for l in literals]) != sum(set([l[3] for l in literals])):
                        continue
                    # --------- Violation of team size ---------
                    # check feasibility, whether required robots of same type for one literal exceeds provided robots
                    is_total_number_larger_than_expected = False
                    # required totoal robots exceeds provided robots
                    for type_of_robot in self.type_num.keys():
                        if sum([l[2] for l in literals if l[1] == type_of_robot]) > self.type_num[type_of_robot]:
                            # raise RuntimeError('{0} required totoal robots exceeds provided robots'.format(literals))
                            is_total_number_larger_than_expected = True
                            break
                    if is_total_number_larger_than_expected:
                        continue
                    # requred robots larger than area of region
                    is_total_robots_larger_than_regions = False
                    for region, area in self.regions.items():
                        if sum([l[2] for l in literals if l[0] == region]) > len(area):
                            is_total_robots_larger_than_regions = True
                            break
                    if is_total_robots_larger_than_regions:
                        continue

                if literals:
                    # one-to-one correspondence between the positive and negative literals
                    pos_clause.append(literals)
                    neg_clause.append(neg_literals)  # [] if no negative literals

        # return if the label is true and only include clause of negative literals
        if only_neg:
            return '1', only_neg_clause, to_dnf('1')  # '&'.join(['~' + '_'.join([str(l) for l in lit]) for lit in neg_literals])

        if not pos_clause:
            formula = to_dnf('0')
        else:
            # formula includes positive and negative literals
            # formula = '|'.join(['&'.join(['&'.join(['_'.join([str(l) for l in lit]) for lit in clause]),
            #                               '&'.join(['~' + '_'.join([str(l) for l in lit]) for lit in neg_clause[index]])])
            #                     for index, clause in enumerate(pos_clause)])
            # if formula[-1] == '&':
            #     formula = to_dnf(formula[:-1])
            # formula includes only positive literals
            formula = '|'.join(['&'.join(['_'.join([str(l) for l in lit]) for lit in clause])
                                for index, clause in enumerate(pos_clause)])
            formula = to_dnf(formula)
        return pos_clause, neg_clause, formula

    def get_subgraph(self, init, accept, is_nonempty_self_loop, segment):
        """
        get the subgraph between init and accept
        """
        # (re)-initialize the set of edges that are satisfied by the initial locations
        self.sat_init_edge = []
        subgraph = self.buchi_graph.copy()
        # remove all other initial vertices for the prefix part
        if segment == 'prefix':
            subgraph.remove_nodes_from([node for node in self.buchi_graph.graph['init'] if node != init])
        # remove initial vertices without selfloop, for suffix part
        elif segment == 'suffix':
            subgraph.remove_nodes_from([node for node in self.buchi_graph.graph['init']
                                        if node in subgraph.nodes and not subgraph.nodes[node]['label']
                                        and node != accept])

        remove_edge = []
        # no self-loop, initial node: keep the edge that robot initial locations satisfy
        if not subgraph.nodes[init]['label']:
            for n in subgraph.succ[init]:  # node is not in succ of node
                if not self.ap_sat_label(subgraph.edges[(init, n)]['label'],
                                         subgraph.edges[(init, n)]['neg_label']):
                    remove_edge.append((init, n))
                # save the edge that is satisfied by the initial robot locations
                else:
                    self.sat_init_edge.append((init, n))

        # has self-loop, initial node: do nothing if the initial robot locations satisfy the vertex label
        # else remove any edge that initial robot locations do not satisfy
        # we didn't remove the initial node if the initial robot locations do not satisfy the label, just use
        # self.sat_vertex to record this.
        elif not self.ap_sat_label(subgraph.nodes[init]['label'],
                                   subgraph.nodes[init]['neg_label']):
            for n in subgraph.succ[init]:
                if not self.ap_sat_label(subgraph.edges[(init, n)]['label'],
                                         subgraph.edges[(init, n)]['neg_label']):
                    remove_edge.append((init, n))
                # save the edge that is satisfied by the initial robot locations
                else:
                    self.sat_init_edge.append((init, n))
        # the initial vertex label, not true, is satisfied
        elif subgraph.nodes[init]['label'] != '1':
            self.sat_vertex = True

        if len(list(subgraph.succ[init])) == 0:
            return None, None, None
            # raise RuntimeError('The task is infeasible!')

        subgraph.remove_edges_from(remove_edge)

        # remove all other accepting vertices except it is the exact init, 'accept_init'
        subgraph.remove_nodes_from([node for node in self.buchi_graph.graph['accept']
                                    if node != accept and node != init])

        # node set
        nodes = self.find_all_nodes(subgraph, init, accept)
        subgraph = subgraph.subgraph(nodes).copy()
        subgraph.graph['init'] = init
        subgraph.graph['accept'] = accept

        # remove all outgoing edges of the accepting state from subgraph for the prefix part if head != tail
        if init != accept:
            remove_edge = list(subgraph.edges(accept))
            subgraph.remove_edges_from(remove_edge)

        # unpruned subgraph used to extract the run
        unpruned_subgraph = subgraph.copy()

        # prune the subgraph
        self.prune_subgraph_automaton(subgraph)

        # get all paths in the pruned subgraph
        paths = []
        if init != accept:
            paths = list(nx.all_simple_paths(subgraph, source=init, target=accept))
        else:
            for s in subgraph.succ[init]:
                paths = paths + [[init] + p for p in list(nx.all_simple_paths(subgraph, source=s, target=accept))]

        # if self loop around the accepting state, then create an element with edge label ,
        # solving prefix and suffix together
        if is_nonempty_self_loop:
            subgraph.add_node('artificial', label='1',
                              neg_label=[], formula=to_dnf('1'))
            subgraph.add_edge(accept, 'artificial', label='1',
                              neg_label=[], formula=to_dnf('1'))
            for path in paths:
                path.append('artificial')

        return subgraph, unpruned_subgraph, paths

    def get_element(self, pruned_subgraph):
        """
        map subtasks to integers
        """
        set_sets_edges = []
        # iterate over all edges and partition into set of sets of equivalent edges
        for edge_in_graph in pruned_subgraph.edges():
            is_added = False
            # iterate over each set of equivalent edges
            for set_edges in set_sets_edges:
                is_equivalent = True
                for edge in set_edges:
                    # equivalence: same node label and edge label, not in the same path
                    # when using sympy, == means structurally equivalent,
                    # it is OK here since we sort the literals in the clause
                    if pruned_subgraph.nodes[edge[0]]['formula'] == pruned_subgraph.nodes[edge_in_graph[0]]['formula'] \
                            and pruned_subgraph.edges[edge]['formula'] == pruned_subgraph.edges[edge_in_graph]['formula']\
                            and not (nx.has_path(pruned_subgraph, edge_in_graph[1], edge[0]) or nx.has_path(pruned_subgraph, edge[1], edge_in_graph[0])):
                        continue
                    else:
                        # break if the considered edge edge_in_graph does not belong to this set of equivalent edges
                        is_equivalent = False
                        break
                # if belong to this set, add then mark
                if is_equivalent:
                    set_edges.append(edge_in_graph)
                    is_added = True
                    break
            # if not added, create a new set of equivalent edges
            if not is_added:
                set_sets_edges.append([edge_in_graph])

        # map each set of equivalent edges to one integer/element
        curr_element = 0
        edge2element = dict()
        element2edge = dict()
        for set_edges in set_sets_edges:
            curr_element += 1
            element2edge[curr_element] = set_edges[0]
            for edge in set_edges:
                edge2element[edge] = curr_element

        return edge2element, element2edge

    def element2label2eccl(self, element2edge, pruned_subgraph):
        """
        {(1, 1): {('l2', 1): [(1, 1, 0, 0)]}, (2, 1): {('l4', 1): [(2, 1, 0, 0)]}}
        edge of element 1: {pair of region and type: (1, 1, 0, 0)}
        """
        element_component2label = dict()
        # colloect eccl that correponds to the same label
        for element in element2edge.keys():
            # node label
            self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
            if self_loop_label and self_loop_label != '1':
                element_component2label[(element, 0)] = self.element2label2eccl_helper(element, 0, self_loop_label)

            # edge label
            edge_label = pruned_subgraph.edges[element2edge[element]]['label']
            if edge_label != '1':
                element_component2label[(element, 1)] = self.element2label2eccl_helper(element, 1, edge_label)

        return element_component2label

    def element2label2eccl_helper(self, element, component, label):
        label2eccl = dict()
        for c, clause in enumerate(label):
            for l, literal in enumerate(clause):
                region_type = tuple(literal[0:2])
                if region_type in label2eccl.keys():
                    label2eccl[region_type].append((element, component, c, l))
                else:
                    label2eccl[region_type] = [(element, component, c, l)]
        return label2eccl

    def map_path_to_element_sequence(self, edge2element, paths):
        """

        """
        element_sequences = []  # set of sets of paths sharing the same set of elements
        # put all path that share the same set of elements into one group
        for path in paths:
            element_sequence = []
            # map one path to one seq of integers
            for i in range(len(path)-1):
                element_sequence.append(edge2element[(path[i], path[i+1])])
            is_added = False
            for i in range(len(element_sequences)):
                # if the considered sequence of integers belong to this set of sequences of integers
                if set(element_sequences[i][0]) == set(element_sequence):
                    element_sequences[i].append(element_sequence)
                    is_added = True
                    break
            # create a new set of sequences of integers
            if not is_added:
                element_sequences.append([element_sequence])

        # for each set of sequences of integers, find one poset
        hasse_graphs = {}
        for index, ele_seq in enumerate(element_sequences):
            # all pairs of ordered elements from the sequence of elements
            linear_order = []
            for i in range(len(ele_seq[0])):
                for j in range(i+1, len(ele_seq[0])):
                    linear_order.append((ele_seq[0][i], ele_seq[0][j]))
            # remove contradictive pairs by iterating over the remaining sequences of integers
            for i in range(1, len(ele_seq)):
                for j in range(len(ele_seq[1])-1):
                    if (ele_seq[i][j+1], ele_seq[i][j]) in linear_order:
                        linear_order.remove((ele_seq[i][j+1], ele_seq[i][j]))

            # hasse diagram
            hasse = DiGraph()
            hasse.add_nodes_from(ele_seq[0])
            hasse.add_edges_from(linear_order)
            self.prune_subgraph(hasse)
            try:
                w = max([len(o) for o in nx.antichains(hasse)])
            except nx.exception.NetworkXUnfeasible:
                print(hasse.edges)
            h = nx.dag_longest_path_length(hasse)
            # h = len([e for e in hasse.nodes if pruned_subgraph.nodes[element2edge[e][0]]['label'] != '1'])
            hasse_graphs[index] = [(w, h), {edge for edge in hasse.edges()}, list(hasse.nodes), hasse]

        return sorted(hasse_graphs.values(), key=lambda x: (x[0][0], -x[0][1]), reverse=True)

    def prune_subgraph_automaton(self, subgraph):
        """
        prune the subgraph following ID and ST properties
        """
        # remove the edge following ID and ST properties
        removed_edge = []
        for node in subgraph.nodes():
            for succ in subgraph.succ[node]:
                if subgraph.nodes[node]['formula'] == subgraph.nodes[succ]['formula']:
                    for next_succ in subgraph.succ[succ]:
                        try:
                            if subgraph.edges[(node, next_succ)]['formula'] == \
                                    And(self.buchi_graph.edges[(node, succ)]['formula'],
                                        self.buchi_graph.edges[(succ, next_succ)]['formula']):
                                removed_edge.append((node, next_succ))
                        except KeyError:
                            continue
        subgraph.remove_edges_from(removed_edge)

    def prune_subgraph(self, subgraph):
        """
        remove the edge as long as there exists another path the connects the vertices
        """
        original_edges = list(subgraph.edges)
        for edge in original_edges:
            subgraph.remove_edge(edge[0], edge[1])
            if nx.has_path(subgraph, edge[0], edge[1]):
                continue
            else:
                subgraph.add_edge(edge[0], edge[1])

    def element2label2eccl_suffix(self, element2edge, pruned_subgraph):
        stage_element_component2label = dict()
        # colloect eccl that correponds to the same label
        for element in element2edge.keys():
            # node label
            self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
            if self_loop_label and self_loop_label != '1':
                stage_element_component2label[(1, element, 0)] = self.element2label2eccl_helper_suffix(1, element, 0,
                                                                                                       self_loop_label)
                stage_element_component2label[(2, element, 0)] = self.element2label2eccl_helper_suffix(2, element, 0,
                                                                                                       self_loop_label)

            # edge label
            edge_label = pruned_subgraph.edges[element2edge[element]]['label']
            if edge_label != '1':
                stage_element_component2label[(1, element, 1)] = self.element2label2eccl_helper_suffix(1, element, 1,
                                                                                                       edge_label)
                stage_element_component2label[(2, element, 1)] = self.element2label2eccl_helper_suffix(2, element, 1,
                                                                                                       edge_label)

        return stage_element_component2label

    def element2label2eccl_helper_suffix(self, stage, element, component, label):
        label2eccl = dict()
        for c, clause in enumerate(label):
            for l, literal in enumerate(clause):
                region_type = tuple(literal[0:2])
                if region_type in label2eccl.keys():
                    label2eccl[region_type].append((stage, element, component, c, l))
                else:
                    label2eccl[region_type] = [(stage, element, component, c, l)]
        return label2eccl

    def find_all_nodes(self, subgraph, init, accept):
        """
        find all nodes that is some path that connects head and tail by finding all simple paths
        """
        in_between = {init, accept}
        if init != accept:
            for path in nx.all_simple_paths(subgraph, source=init, target=accept):
                in_between.update(set(path))
        else:
            for suc in subgraph.succ[init]:
                for path in nx.all_simple_paths(subgraph, source=suc, target=accept):
                    in_between.update(set(path))
        return in_between

    def implication_check(self, pruned_subgraph, paths):
        """
        check whether the final locations of the prefix part
        satisfy the edge (prior, accept) and vertex prior
        :return:
        """
        i = 0
        while i < len(paths):
            path_label = pruned_subgraph.edges[tuple(paths[i][-2:])]['label']
            path_neg_label = pruned_subgraph.edges[tuple(paths[i][-2:])]['neg_label']

            prior_label = pruned_subgraph.nodes[paths[i][-2]]['label']
            prior_neg_label = pruned_subgraph.nodes[paths[i][-2]]['neg_label']

            if self.ap_sat_label(path_label, path_neg_label) and \
                    self.ap_sat_label(prior_label, prior_neg_label):
                i += 1
            else:
                del paths[i]





















