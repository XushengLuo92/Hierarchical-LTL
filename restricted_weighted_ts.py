import itertools
import networkx as nx
import restricted_milp

def construct_node_set(reduced_task_network, task_hierarchy, type_robot_label):
    """
    construct the node set of the routing graph
    """
    node_index = 0
    # nodes for initial location
    init_type_robot_node = dict()
    # nodes for label
    task_element_component_clause_literal_node = dict()
    # attributes of nodes
    node_location_type_component_task_element = dict()
    # type_robot (robot type, robot index in the type), location: l2,r1
    for type_robot, location in type_robot_label.items():
        init_type_robot_node[type_robot] = node_index
        # treat as edge component of element -1
        node_location_type_component_task_element[node_index] = [location, type_robot[0], 1, -1]
        node_index += 1

    # node set for self-loop label
    for (task, element) in reduced_task_network.nodes():
        pruned_subgraph = task_hierarchy[task].buchi_graph
        element2edge = task_hierarchy[task].element2edge
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']
        # node set for edge label, edge label not equal 1
        if edge_label != '1':
            node_index = construct_node_set_helper(task, element, 1, edge_label, task_element_component_clause_literal_node,
                                                   node_location_type_component_task_element, node_index)
        # with self loop, nodes for self-loop of the first subtask are created no matter whether the initial locations
        # satisfy it or not, which will be dealt with in milp in one-clause constraint
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        if self_loop_label and self_loop_label != '1':
            node_index = construct_node_set_helper(task, element, 0, self_loop_label, task_element_component_clause_literal_node,
                                                   node_location_type_component_task_element, node_index)

    return init_type_robot_node, task_element_component_clause_literal_node, node_location_type_component_task_element, node_index


def construct_node_set_helper(task, element, component, label, task_element_component_clause_literal_node,
                              node_location_type_component_task_element, node_index):
    for c in range(len(label)):
        for l in range(len(label[c])):
            end = node_index + label[c][l][2]
            task_element_component_clause_literal_node[(task, element, component, c, l)] = list(range(node_index, end))
            node_location_type_component_task_element.update({node: list(label[c][l][0:2]) + [component, task, element]
                                                         for node in list(range(node_index, end))})
            node_index = end
    return node_index


def construct_edge_set(task_hierarchy, reduced_task_network, task_element_component_clause_literal_node,
                       task_element_component2label, init_type_robot_node, incomparable_task_element,
                       strict_larger_task_element, larger_task_element):
    """
    construct the edge set of the routing graph
    """
    edge_set = []
    for (task, element) in reduced_task_network.nodes():
        pruned_subgraph = task_hierarchy[task].buchi_graph
        element2edge = task_hierarchy[task].element2edge
        # prior subtasks: all elements that are incomparable or larger than the current one
        incmp_large_task_element = incomparable_task_element[(task, element)] + larger_task_element[(task, element)]

        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']

        # edge label
        if edge_label != '1':
            construct_edge_set_for_edge_helper(task, element, task_element_component2label,
                                               task_element_component_clause_literal_node,
                                               init_type_robot_node, incmp_large_task_element, edge_set)
        # vertex label
        if self_loop_label and self_loop_label != '1':
            construct_edge_set_for_node_helper(task, element, element2edge, task_element_component2label,
                                               task_element_component_clause_literal_node,
                                               init_type_robot_node, strict_larger_task_element, incomparable_task_element,
                                               edge_set, pruned_subgraph)

    return edge_set


def construct_edge_set_for_edge_helper(task, element, task_element_component2label,
                                       task_element_component_clause_literal_node,
                                       init_type_robot_node, incmp_task_element, edge_set):
    for label, teccls in task_element_component2label[(task, element, 1)].items():
        # from initial locations
        from_node = [node for type_robot, node in init_type_robot_node.items() if type_robot[0] == label[1]]
        # collect all nodes that share same region and robot type
        for teccl in teccls:
            to_node = task_element_component_clause_literal_node[teccl]
            edge_set += list(itertools.product(from_node, to_node))
            # find all qualified nodes that share the same robot type from incomparable or precedent nodes
            for in_task_ele in incmp_task_element:
                for comp in [0, 1]:
                    # 0 for vertex label, 1 for edge label
                    try:
                        for in_label, in_teccls in task_element_component2label[in_task_ele +  (comp, )].items():
                            if label[1] == in_label[1]:  # same robot type
                                for in_teccl in in_teccls:
                                    # from_node == # to_node
                                    if len(task_element_component_clause_literal_node[in_teccl]) == len(to_node):
                                        # if they have the same indicator, then the number of vertices must be the same
                                        from_node = task_element_component_clause_literal_node[in_teccl][:len(to_node)]
                                        edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]
                                    else:
                                        edge_set += list(
                                            itertools.product(task_element_component_clause_literal_node[in_teccl],
                                                              to_node))

                    except KeyError:  # label is 1
                        pass

            # additional nodes for edge label, nodes from node label of the current element
            # vertex label
            try:
                for in_label, in_teccls in task_element_component2label[(task, element, 0)].items():
                    if label[1] == in_label[1]:  # same robot type
                        for in_teccl in in_teccls:
                            if len(task_element_component_clause_literal_node[in_teccl]) == len(to_node):
                                from_node = task_element_component_clause_literal_node[in_teccl][:len(to_node)]
                                edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]
                            else:
                                edge_set += list(itertools.product(task_element_component_clause_literal_node[in_teccl],
                                                                   to_node))

            except KeyError:
                pass


def construct_edge_set_for_node_helper(task, element, task_hierarchy, task_element_component2label,
                                       task_element_component_clause_literal_node,
                                       init_type_robot_node, strict_larger_task_element, incomparable_task_element,
                                       edge_set):
    """
        edge set for the nodes in NBA
    """
    # can be the first subtask
    if not strict_larger_task_element[(task, element)]:
        for label, teccls in task_element_component2label[(task, element, 0)].items():
            # from initial locations
            from_node = [node for type_robot, node in init_type_robot_node.items() if type_robot[0] == label[1]]
            # collect all nodes that share same region and robot type
            for teccl in teccls:
                to_node = task_element_component_clause_literal_node[teccl]
                edge_set += list(itertools.product(from_node, to_node))

    for another_task_element in strict_larger_task_element[(task, element)] + incomparable_task_element[(task, element)]:
        element2edge = task_hierarchy[another_task_element[0]]
        pruned_subgraph = task_hierarchy[another_task_element[0]]
        edge = element2edge[another_task_element]
        # have positive literals
        if pruned_subgraph.nodes[edge[1]]['label'] != '1':
            # paired clause with implication
            for pair in pruned_subgraph.imply[edge]:
                # literal in the end vertex
                for index, lit in enumerate(pruned_subgraph.nodes[edge[1]]['label'][pair[1]]):
                    # literal in the edge
                    index_in_edge = pruned_subgraph.edges[edge]['label'][pair[0]].index(lit)
                    from_node = task_element_component_clause_literal_node[(another_task_element, 1, pair[0], index_in_edge)]
                    to_node = task_element_component_clause_literal_node[(task, element, 0, pair[1], index)]
                    edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]


def get_order_info(reduced_task_network, composite_subtasks):
    """
        elements that are larger or imcomparable to a certain element
    """
    pairwise_or_relation_composite_subtasks = set()
    for (_, or_subtasks) in composite_subtasks.items():
        for subtasks in or_subtasks.or_composite_subtasks:
                pairwise_or_relation_composite_subtasks.update({pair for pair in itertools.combinations(subtasks, 2)})
                
    incomparable_task_element = dict()
    larger_task_element = dict()  # prior to
    smaller_task_element = dict()  # after
    strict_larger_task_element = dict()
    # iterate over the whole poset
    for task_element in reduced_task_network.nodes():
        incmp = []
        lg = []
        sm = []
        for another_task_element in reduced_task_network.nodes():
            if another_task_element != task_element:
                if nx.has_path(reduced_task_network, source=another_task_element, target=task_element):
                    lg.append(another_task_element)
                elif nx.has_path(reduced_task_network, source=task_element, target=another_task_element):
                    sm.append(another_task_element)
                elif (task_element[0], another_task_element[0]) not in pairwise_or_relation_composite_subtasks and \
                    (another_task_element[0], task_element[0]) not in pairwise_or_relation_composite_subtasks:
                    incmp.append(another_task_element)
        incomparable_task_element[task_element] = incmp
        larger_task_element[task_element] = lg
        smaller_task_element[task_element] = sm
        strict_larger_task_element[task_element] = [order[0] for order in reduced_task_network.edges() if order[1] == task_element]

    return incomparable_task_element, larger_task_element, smaller_task_element, \
        strict_larger_task_element, pairwise_or_relation_composite_subtasks

def task_element2label2teccl(task_hierarchy, reduced_task_network):
    """
    {(l100, 1, 1): {('l2', 1): [(1, 1, 0, 0)]}, (l200, 2, 1): {('l4', 1): [(2, 1, 0, 0)]}}
    edge of element 1 in task l100: {pair of region and type: corresponding index}
    """

    task_element_component2label2teccl = dict()
    for (task, hierarchy) in task_hierarchy.items():
        element2edge = hierarchy.element2edge
        pruned_subgraph = hierarchy.buchi_graph
        # colloect eccl that correponds to the same label
        for element in element2edge.keys():
            if (task, element) not in reduced_task_network.nodes():
                continue
            # node label
            self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
            if self_loop_label and self_loop_label != '1':
                task_element_component2label2teccl[(task, element, 0)] = task_element2label2teccl_helper(task, element, 0, self_loop_label)

            # edge label
            edge_label = pruned_subgraph.edges[element2edge[element]]['label']
            if edge_label != '1':
                task_element_component2label2teccl[(task, element, 1)] = task_element2label2teccl_helper(task, element, 1, edge_label)
    
    return task_element_component2label2teccl


def task_element2label2teccl_helper(task, element, component, label):
    label2teccl = dict()
    for c, clause in enumerate(label):
        for l, literal in enumerate(clause):
            region_type = tuple(literal[0:2])
            if region_type in label2teccl.keys():
                label2teccl[region_type].append((task, element, component, c, l))
            else:
                label2teccl[region_type] = [(task, element, component, c, l)]
    return label2teccl  

def task_element2robot2eccl(reduced_task_network, task_hierarchy):
    """
    map each indicator to specific literals
    """
    # the robots involved in the component of the element and the corresponding label
    robot2teccl = dict()
    # iterate over all elements and all literals
    for (task, element) in reduced_task_network:
        pruned_subgraph = task_hierarchy[task].buchi_graph
        element2edge = task_hierarchy[task].element2edge
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']
        if edge_label != '1':
            for c, clause in enumerate(edge_label):
                for l, literal in enumerate(clause):
                    robot = literal[-1]
                    if robot == 0:
                        continue
                    # non-zero indicator, keep track
                    if robot in robot2teccl.keys():
                        robot2teccl[robot].append((task, element, 1, c, l))
                    else:
                        robot2teccl[robot] = [(task, element, 1, c, l)]
        # same for the vertex label
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        if self_loop_label and self_loop_label != '1':
            for c, clause in enumerate(self_loop_label):
                for l, literal in enumerate(clause):
                    robot = literal[-1]
                    if robot == 0:
                        continue
                    if robot in robot2teccl.keys():
                        robot2teccl[robot].append((task, element, 0, c, l))
                    else:
                        robot2teccl[robot] = [(task, element, 0, c, l)]
    return robot2teccl

def construct_graph(task_hierarchy, reduced_task_network, composite_subtasks, workspace, show=False):
    """
    build the routing graph from the node and edge set
    """
    incomparable_task_element, larger_task_element, smaller_task_element, strict_larger_task_element, \
        pairwise_or_relation_composite_subtasks = get_order_info(reduced_task_network, composite_subtasks)
    init_type_robot_node, task_element_component_clause_literal_node, node_location_type_component_task_element, \
    num_nodes = construct_node_set(reduced_task_network, task_hierarchy, workspace.type_robot_label)
    task_element_component2label2teccl = task_element2label2teccl(task_hierarchy, reduced_task_network)
    edge_set = construct_edge_set(task_hierarchy, reduced_task_network, task_element_component_clause_literal_node,
                                                        task_element_component2label2teccl,
                                                        init_type_robot_node, incomparable_task_element,
                                                        strict_larger_task_element,
                                                        larger_task_element)
    # print('incomparable_task_element: ', incomparable_task_element)
    # print('larger_task_element: ', larger_task_element)
    # print('smaller_task_element: ', smaller_task_element)
    # print('strict_larger_task_element: ', strict_larger_task_element)
    # print("num_nodes: ", num_nodes)
    # print("task_element_component_clause_literal_node: ", task_element_component_clause_literal_node)
    # print("task_element_component2label2teccl: ", task_element_component2label2teccl)
    # print("edge_set: ", edge_set)
    
    ts = nx.DiGraph(type='routing_graph')
    for node in list(range(num_nodes)):
        ts.add_node(node, location_type_component_task_element=node_location_type_component_task_element[node])
    if reduced_task_network.graph["task"] == "man":
        for edge in edge_set:
            if (ts.nodes[edge[0]]['location_type_component_task_element'][3] == ts.nodes[edge[1]]['location_type_component_task_element'][3]):
                ts.add_edge(edge[0], edge[1], weight=1)
            else:
                ts.add_edge(edge[0], edge[1], weight=10)
    else:
        # navigation
        for edge in edge_set:
            ts.add_edge(edge[0], edge[1], weight=workspace.p2p[(ts.nodes[edge[0]]['location_type_component_task_element'][0],
                                                                ts.nodes[edge[1]]['location_type_component_task_element'][0])])
        


    # TODO further prune the graph to invoke less number of robots
    # reduced_ts = nx.transitive_reduction(ts)
    # reduced_ts.add_nodes_from(ts.nodes(data=True))
    # reduced_ts.add_edges_from((u, v, ts.edges[u, v]) for u, v in reduced_ts.edges())

    return ts, task_element_component_clause_literal_node, init_type_robot_node, \
        strict_larger_task_element, incomparable_task_element, larger_task_element, pairwise_or_relation_composite_subtasks
