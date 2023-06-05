import itertools
import networkx as nx


def construct_node_set(poset, element2edge, pruned_subgraph, type_robot_label):
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
    for element in poset:
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']
        # node set for edge label, edge label not equal 1
        if edge_label != '1':
            node_index = construct_node_set_helper(element, 1, edge_label, element_component_clause_literal_node,
                                                   node_location_type_component_element, node_index)
        # with self loop
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        if self_loop_label and self_loop_label != '1':
            node_index = construct_node_set_helper(element, 0, self_loop_label, element_component_clause_literal_node,
                                                   node_location_type_component_element, node_index)

    return init_type_robot_node, element_component_clause_literal_node, node_location_type_component_element, node_index


def construct_node_set_helper(element, component, label, element_component_clause_literal_node,
                              node_location_type_component_element, node_index):
    for c in range(len(label)):
        for l in range(len(label[c])):
            end = node_index + label[c][l][2]
            element_component_clause_literal_node[(element, component, c, l)] = list(range(node_index, end))
            node_location_type_component_element.update({node: label[c][l][0:2] + [component, element]
                                                         for node in list(range(node_index, end))})
            node_index = end
    return node_index


def construct_edge_set(poset, element_component_clause_literal_node, element2edge, pruned_subgraph,
                       element_component2label, init_type_robot_node, incomparable_element, larger_element):
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
            construct_edge_set_helper(element, 1, element_component2label,
                                      element_component_clause_literal_node,
                                      init_type_robot_node, incmp_large_element, edge_set)
        # vertex label
        if self_loop_label and self_loop_label != '1':
            construct_edge_set_helper(element, 0, element_component2label,
                                      element_component_clause_literal_node,
                                      init_type_robot_node, incmp_large_element, edge_set)

    return edge_set


def construct_edge_set_helper(element, component, element_component2label, element_component_clause_literal_node,
                              init_type_robot_node, incmp_element, edge_set):
    for label, eccls in element_component2label[(element, component)].items():
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
                                    # from_node >= # to_node
                                    if len(element_component_clause_literal_node[in_eccl]) >= len(to_node):
                                        # if they have the same indicator, then the number of vertices must be the same
                                        from_node = element_component_clause_literal_node[in_eccl][:len(to_node)]
                                        edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]
                                    else:
                                        edge_set += list(itertools.product(element_component_clause_literal_node[in_eccl],
                                                                           to_node))

                    except KeyError:  # label is 1
                        pass

            # additional nodes for edge label, nodes from node label of the current element
            if component == 1:
                # vertex label
                try:
                    for in_label, in_eccls in element_component2label[(element, 0)].items():
                        if label[1] == in_label[1]:  # same robot type
                            for in_eccl in in_eccls:
                                if len(element_component_clause_literal_node[in_eccl]) >= len(to_node):
                                    from_node = element_component_clause_literal_node[in_eccl][:len(to_node)]
                                    edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]
                                else:
                                    edge_set += list(itertools.product(element_component_clause_literal_node[in_eccl],
                                                                       to_node))

                except KeyError:
                    pass


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
