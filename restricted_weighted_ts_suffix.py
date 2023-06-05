import itertools
import networkx as nx


def construct_node_set(poset, element2edge, pruned_subgraph, type_robot_label, minimal_element, last_subtask, loop):
    """
    construct the node set of the routing graph
    """
    node_index = 0
    # nodes for initial location
    init_type_robot_node = dict()
    # nodes for label
    element_component_clause_literal_node = dict()
    # attributes of nodes
    node_location_type_component_element = dict()
    # type_robot (robot type, robot index in the type), location: l2,r1
    for type_robot, location in type_robot_label.items():
        init_type_robot_node[type_robot] = node_index
        # treat as edge component of element -1
        node_location_type_component_element[node_index] = [location, type_robot[0], 1, -1]
        node_index += 1

    # node set for self-loop label
    final_element_type_robot_node = dict()
    for element in poset:
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']
        # node set for edge label,
        if edge_label != '1':
            node_index = construct_node_set_helper(element, 1, edge_label, element_component_clause_literal_node,
                                                   node_location_type_component_element, node_index, minimal_element,
                                                   type_robot_label, final_element_type_robot_node, last_subtask, loop)
        # with self loop
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        if self_loop_label and self_loop_label != '1':
            node_index = construct_node_set_helper(element, 0, self_loop_label, element_component_clause_literal_node,
                                                   node_location_type_component_element, node_index, minimal_element,
                                                   type_robot_label, final_element_type_robot_node, last_subtask)

    return init_type_robot_node, element_component_clause_literal_node, node_location_type_component_element, \
           node_index, final_element_type_robot_node


def construct_node_set_helper(element, component, label, element_component_clause_literal_node,
                              node_location_type_component_element, node_index, minimal_element,
                              type_robot_label, final_element_type_robot_node, last_subtask, loop=False):
    # edge label not equal 1 or element is in the set of minimal elements
    for c in range(len(label)):
        for l in range(len(label[c])):
            end = node_index + label[c][l][2]
            element_component_clause_literal_node[(element, component, c, l)] = list(range(node_index, end))
            node_location_type_component_element.update({node: list(label[c][l][0:2]) + [component, element]
                                                         for node in list(range(node_index, end))})
            node_index = end

    # edge label for minimal elements
    # didn't explicitly create the extra clause for the literal
    # ---------- return to initial locations -----------------
    if loop:
        if component == 1 and element in minimal_element:
            type_robot_node = dict()
            for type_robot, location in type_robot_label.items():
                type_robot_node[type_robot] = node_index
                node_location_type_component_element[node_index] = [location, type_robot[0], 1, element]
                node_index += 1
            final_element_type_robot_node[element] = type_robot_node
    # ---------- return to respective regions -----------------
    else:
        if component == 1 and element in minimal_element:
            type_robot_node = dict()
            for location, type_robots in last_subtask['essential_robot_edge'].items():
                for type_robot in type_robots:
                    type_robot_node[type_robot] = node_index
                    node_location_type_component_element[node_index] = [location, type_robot[0], 1, element]
                    node_index += 1
            final_element_type_robot_node[element] = type_robot_node
    return node_index


def construct_edge_set(poset, element_component_clause_literal_node, element2edge, pruned_subgraph,
                       element_component2label, init_type_robot_node, incomparable_element, strict_larger_element,
                       larger_element, imply,
                       minimal_element, final_element_type_robot_node):
    """
    construct the edge set of the routing graph
    """
    edge_set = []
    for element in poset:
        # prior subtasks: all elements that are incomparable or larger than the current one
        incmp_large_element = incomparable_element[element] + larger_element[element]

        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']

        # edge label
        if edge_label != '1':
            construct_edge_set_for_edge_helper(element, element_component2label,
                                               element_component_clause_literal_node,
                                               init_type_robot_node, incmp_large_element, edge_set)

        if element in minimal_element:
            construct_edge_set_for_edge_minimal_helper(element, element_component2label,
                                                       element_component_clause_literal_node,
                                                       init_type_robot_node, incmp_large_element, edge_set,
                                                       final_element_type_robot_node)
        # vertex label
        if self_loop_label and self_loop_label != '1':
            construct_edge_set_for_node_helper(element, element2edge, element_component2label,
                                               element_component_clause_literal_node,
                                               init_type_robot_node, strict_larger_element, incomparable_element,
                                               edge_set, imply, pruned_subgraph)

    return edge_set


def construct_edge_set_for_edge_helper(element, element_component2label, element_component_clause_literal_node,
                                       init_type_robot_node, incmp_element, edge_set):
    for label, eccls in element_component2label[(element, 1)].items():
        # from initial locations
        from_node = [node for type_robot, node in init_type_robot_node.items() if type_robot[0] == label[1]]
        # collect all nodes that share same region and robot type
        for eccl in eccls:
            to_node = element_component_clause_literal_node[eccl]
            edge_set += list(itertools.product(from_node, to_node))
            # find all qualified nodes that share the same robot type from incomparable or precedent nodes
            for in_ele in incmp_element:
                for comp in [0, 1]:
                    # 0 for vertex label, 1 for edge label
                    try:
                        for in_label, in_eccls in element_component2label[(in_ele, comp)].items():
                            if label[1] == in_label[1]:  # same robot type
                                for in_eccl in in_eccls:
                                    # from_node == # to_node
                                    if len(element_component_clause_literal_node[in_eccl]) == len(to_node):
                                        # if they have the same indicator, then the number of vertices must be the same
                                        from_node = element_component_clause_literal_node[in_eccl][:len(to_node)]
                                        edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]
                                    else:
                                        edge_set += list(
                                            itertools.product(element_component_clause_literal_node[in_eccl],
                                                              to_node))

                    except KeyError:  # label is 1
                        pass

            # additional nodes for edge label, nodes from node label of the current element
            # vertex label
            try:
                for in_label, in_eccls in element_component2label[(element, 0)].items():
                    if label[1] == in_label[1]:  # same robot type
                        for in_eccl in in_eccls:
                            if len(element_component_clause_literal_node[in_eccl]) == len(to_node):
                                from_node = element_component_clause_literal_node[in_eccl][:len(to_node)]
                                edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]
                            else:
                                edge_set += list(itertools.product(element_component_clause_literal_node[in_eccl],
                                                                   to_node))

            except KeyError:
                pass


def construct_edge_set_for_edge_minimal_helper(element, element_component2label,
                                               element_component_clause_literal_node,
                                               init_type_robot_node, incmp_element, edge_set,
                                               final_element_type_robot_node):
    """
    create edge set for the extra literal, no outgoing edges from nodes for the extra literal
    """
    for type_robot, to_node in final_element_type_robot_node[element].items():
        # from initial locations of exact robot of the same type
        from_node = init_type_robot_node[type_robot]
        # the node that type_robot should visit finally
        edge_set.append((from_node, to_node))
        # find all qualified nodes that share the same robot type from incomparable or precedent nodes
        for in_ele in incmp_element:
            for comp in [0, 1]:
                # 0 for vertex label, 1 for edge label
                try:
                    for in_label, in_eccls in element_component2label[(in_ele, comp)].items():
                        if type_robot[0] == in_label[1]:  # same robot type
                            for in_eccl in in_eccls:
                                # randomly set one node
                                edge_set += list(itertools.product(element_component_clause_literal_node[in_eccl],
                                                                   [to_node]))
                except KeyError:  # label is 1
                    pass

        # additional nodes for edge label, nodes from node label of the current element
        # vertex label
        try:
            for in_label, in_eccls in element_component2label[(element, 0)].items():
                if type_robot[0] == in_label[1]:  # same robot type
                    for in_eccl in in_eccls:
                        edge_set += list(itertools.product(element_component_clause_literal_node[in_eccl],
                                                           [to_node]))

        except KeyError:
            pass


def construct_edge_set_for_node_helper(element, element2edge, element_component2label,
                                       element_component_clause_literal_node,
                                       init_type_robot_node, strict_larger_element, incomparable_element,
                                       edge_set, imply, pruned_subgraph):
    """
        edge set for the nodes in NBA
    """
    # can be the first subtask
    if not strict_larger_element[element]:
        for label, eccls in element_component2label[(element, 0)].items():
            # from initial locations
            from_node = [node for type_robot, node in init_type_robot_node.items() if type_robot[0] == label[1]]
            # collect all nodes that share same region and robot type
            for eccl in eccls:
                to_node = element_component_clause_literal_node[eccl]
                edge_set += list(itertools.product(from_node, to_node))

    for another_element in strict_larger_element[element] + incomparable_element[element]:
        edge = element2edge[another_element]
        # have positive literals
        if pruned_subgraph.nodes[edge[1]]['label'] != '1':
            # paired clause with implication
            for pair in imply[edge]:
                # literal in the end vertex
                for index, lit in enumerate(pruned_subgraph.nodes[edge[1]]['label'][pair[1]]):
                    # literal in the edge
                    index_in_edge = pruned_subgraph.edges[edge]['label'][pair[0]].index(lit)
                    from_node = element_component_clause_literal_node[(another_element, 1, pair[0], index_in_edge)]
                    to_node = element_component_clause_literal_node[(element, 0, pair[1], index)]
                    edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]


def construct_graph(num_nodes, node_location_type_component_element, edge_set, p2p):
    """
    build the routing graph from the node and edge set
    """
    ts = nx.DiGraph()
    for node in list(range(num_nodes)):
        ts.add_node(node, location_type_component_element=node_location_type_component_element[node])
    for edge in edge_set:
        ts.add_edge(edge[0], edge[1], weight=p2p[(ts.nodes[edge[0]]['location_type_component_element'][0],
                                                  ts.nodes[edge[1]]['location_type_component_element'][0])])
    return ts
