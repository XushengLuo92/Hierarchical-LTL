from gurobipy import *
import itertools
import networkx as nx
import math


def element2robot2eccl(poset, element2edge, pruned_subgraph):
    """
    map each indicator to specific literals
    """
    # the robots involved in the component of the element and the corresponding label
    robot2eccl = dict()
    # iterate over all elements and all literals
    for element in poset:
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']
        if edge_label != '1':
            for c, clause in enumerate(edge_label):
                for l, literal in enumerate(clause):
                    robot = literal[-1]
                    if robot == 0:
                        continue
                    # non-zero indicator, keep track
                    if robot in robot2eccl.keys():
                        robot2eccl[robot].append((element, 1, c, l))
                    else:
                        robot2eccl[robot] = [(element, 1, c, l)]
        # same for the vertex label
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        if self_loop_label and self_loop_label != '1':
            for c, clause in enumerate(self_loop_label):
                for l, literal in enumerate(clause):
                    robot = literal[-1]
                    if robot == 0:
                        continue
                    if robot in robot2eccl.keys():
                        robot2eccl[robot].append((element, 0, c, l))
                    else:
                        robot2eccl[robot] = [(element, 0, c, l)]
    return robot2eccl


def incomparable_larger(poset, hasse_diagram, pruned_subgraph, element2edge):
    """
        elements that are larger or imcomparable to a certain element
    """
    incomparable_element = dict()
    larger_element = dict()
    # iterate over the whole poset
    for element in poset:
        incmp = []
        lg = []
        for another_element in poset:
            if another_element != element:
                if nx.has_path(hasse_diagram, source=another_element, target=element):
                    lg.append(another_element)
                elif not nx.has_path(hasse_diagram, source=element, target=another_element):
                    incmp.append(another_element)
        incomparable_element[element] = incmp
        larger_element[element] = lg

    return incomparable_element, larger_element


def update_poset4_suffix(pos, max_pos, element2edge, element_component2label, poset_relation, incomparable_element,
                         larger_element, pruned_subgraph):

    # poset
    pos2 = [element + max_pos for element in pos]

    # element2edge
    element2edge2 = {element + max_pos: edge for element, edge in element2edge.items()}
    element2edge.update(element2edge2)

    # element_component2label
    element_component2label2 = dict()
    for element_component, label_eccl in element_component2label.items():
        label_eccl2 = dict()
        for label, eccls in label_eccl.items():
            eccl2 = [(element_component[0] + max_pos, eccl[1], eccl[2], eccl[3]) for eccl in eccls]
            label_eccl2[label] = eccl2
        element_component2label2[(element_component[0] + max_pos, element_component[1])] = label_eccl2
    element_component2label.update(element_component2label2)

    # poset_relation
    poset_relation2 = set([(pair[0] + max_pos, pair[1] + max_pos) for pair in poset_relation])
    poset_relation.update(poset_relation2)

    # incomparable_element
    incomparable_element2 = {element + max_pos: [e + max_pos for e in in_cmp]
                             for element, in_cmp in incomparable_element.items()}
    incomparable_element.update(incomparable_element2)

    larger_element2 = {element + max_pos: [e + max_pos for e in lg] +
                                          [e for e in pos if pruned_subgraph.edges[element2edge[e]]['label'] != '1']
                       for element, lg in larger_element.items()}
    larger_element.update(larger_element2)
    # update pos
    pos += pos2


