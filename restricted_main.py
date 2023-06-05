from task import Task
from restricted_buchi_parse import Buchi
from datetime import datetime
import restricted_poset
# from workspace import Workspace
# from workspace_dars import Workspace
from workspace_case1 import Workspace

import matplotlib.pyplot as plt
import restricted_weighted_ts
import restricted_weighted_ts_suffix
import restricted_milp
import restricted_milp_suf
import pickle
from vis import plot_workspace
import numpy
from post_processing import run
# from MAPP_heuristic import mapp
from restricted_GMAPP import mapp, compute_path_cost, return_to_initial
from vis import vis
import sys
from termcolor import colored, cprint
import networkx as nx
from sympy.logic.boolalg import to_dnf
import numpy as np

print_red_on_cyan = lambda x: cprint(x, 'blue', 'on_red')


def ltl_mrta(formula):
    workspace = Workspace()
    with open('data/workspace', 'wb') as filehandle:
        pickle.dump(workspace, filehandle)

    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # plot_workspace(workspace, ax)
    # plt.show()

    # with open('data/workspace', 'rb') as filehandle:
    #     workspace = pickle.load(filehandle)
    type_robot_location = workspace.type_robot_location.copy()
    # return to initial locations or not
    loop = True
    one_time = False
    draw = True
    show = True
    best_cost = []
    best_path = dict()

    start = datetime.now()
    # --------------- constructing the Buchi automaton ----------------
    task = Task(formula)
    buchi = Buchi(task, workspace)

    buchi.construct_buchi_graph()
    # print('partial time: {0}'.format((datetime.now() - start).total_seconds()))

    init_acpt = buchi.get_init_accept()

    for pair, _ in init_acpt:

        workspace.type_robot_location = type_robot_location.copy()
        workspace.update_after_prefix()
        buchi.atomic_prop = workspace.atomic_prop
        buchi.regions = workspace.regions

        init_state, accept_state = pair[0], pair[1]

        # ======================================= prefix part =================================================#
        #                                                                                                      #
        #                                                                                                      #
        #                                                                                                      #
        # ======================================= prefix part =================================================#

        # ----------------- infer the poset -----------------------
        pruned_subgraph, unpruned_subgraph, paths = buchi.get_subgraph(init_state, accept_state, 'prefix')
        edge2element, element2edge = buchi.get_element(pruned_subgraph)
        if not edge2element:
            continue

        element_component2label = buchi.element2label2eccl(element2edge, pruned_subgraph)

        hasse_graphs = buchi.map_path_to_element_sequence(edge2element, paths)
        # loop over all posets
        for _, poset_relation, pos, hasse_diagram in hasse_graphs:
            if show:
                print_red_on_cyan('================ prefix part ================')

            # ----------------- restore after the suffix if not succeed -----------------
            workspace.type_robot_location = type_robot_location.copy()
            workspace.update_after_prefix()
            buchi.atomic_prop = workspace.atomic_prop
            buchi.regions = workspace.regions

            robot2eccl = restricted_poset.element2robot2eccl(pos, element2edge, pruned_subgraph)

            # print('partial time to poset: {0}'.format((datetime.now() - start).total_seconds()))

            if show:
                for order in poset_relation:
                    print(pruned_subgraph.edges[element2edge[order[0]]]['formula'], ' -> ',
                          pruned_subgraph.edges[element2edge[order[1]]]['formula'])
                print('----------------------------------------------')

            incomparable_element, larger_element, smaller_element, strict_larger_element = \
                restricted_poset.incomparable_larger(pos, poset_relation, hasse_diagram)

            # --------------- construct the routing graph ---------------
            init_type_robot_node, element_component_clause_literal_node, node_location_type_component_element, \
            num_nodes = restricted_weighted_ts.construct_node_set(pos, element2edge, pruned_subgraph,
                                                                  workspace.type_robot_label)

            edge_set = restricted_weighted_ts.construct_edge_set(pos, element_component_clause_literal_node,
                                                                 element2edge, pruned_subgraph,
                                                                 element_component2label,
                                                                 init_type_robot_node, incomparable_element,
                                                                 strict_larger_element,
                                                                 larger_element, buchi.imply)

            ts = restricted_weighted_ts.construct_graph(num_nodes, node_location_type_component_element, edge_set,
                                                        workspace.p2p)

            if show:
                print('partial time before milp: {0}'.format((datetime.now() - start).total_seconds()))

            # --------------------- MILP -------------------------
            maximal_element = [node for node in hasse_diagram.nodes() if hasse_diagram.in_degree(node) == 0]

            robot_waypoint_pre, robot_time_pre, id2robots, robot_label_pre, \
            robot_waypoint_axis, robot_time_axis, time_axis, acpt_run, \
                = restricted_milp.construct_milp_constraint(ts, workspace.type_num, pos,
                                                            pruned_subgraph,
                                                            element2edge,
                                                            element_component_clause_literal_node,
                                                            poset_relation,
                                                            init_type_robot_node,
                                                            strict_larger_element,
                                                            incomparable_element,
                                                            larger_element,
                                                            robot2eccl, init_state,
                                                            buchi, maximal_element, show)
            if not robot_waypoint_pre:
                continue

            for robot, time in list(robot_time_pre.items()):
                #  delete such robots that did not participate (the initial location of robots may just satisfies)
                if time[-1] == 0 and len(time) == 1:
                    del robot_time_pre[robot]
                    del robot_waypoint_pre[robot]

            if show:
                print('----------------------------------------------')
                for type_robot, waypoint in robot_waypoint_pre.items():
                    print(type_robot, " : ", waypoint)
                    print(type_robot, " : ", robot_time_pre[type_robot])
                    print(type_robot, " : ", robot_label_pre[type_robot])
                print('----------------------------------------------')

                print('time axis: ', time_axis)

            for robot, time in list(robot_time_axis.items()):
                #  delete such robots that did not participate (the initial location of robots may just satisfies)
                if not time:
                    del robot_time_axis[robot]
                    del robot_waypoint_axis[robot]

            if show:
                for type_robot, waypoint in robot_waypoint_axis.items():
                    print(type_robot, " : ", waypoint)
                    print(type_robot, " : ", robot_time_axis[type_robot])

                print('----------------------------------------------')

                for stage in acpt_run:
                    print(stage)
                print('----------------------------------------------')

            # --------------------- GMRPP -------------------------
            robot_path_pre = mapp(workspace, buchi, acpt_run, robot_waypoint_axis, robot_time_axis,
                                  'simultaneous', show)

            # vis(workspace, robot_path_pre, {robot: [len(path)] * 2 for robot, path in robot_path_pre.items()},
            #     [])
            # ----------------- check whether the final locations of the prefix part satisfy the accept state ---------
            workspace.type_robot_location = {robot: path[-1] for robot, path in robot_path_pre.items()}
            workspace.update_after_prefix(loop)
            buchi.atomic_prop = workspace.atomic_prop
            buchi.regions = workspace.regions

            last_subtask = acpt_run[-1]
            # add the removed self-loop of initial state
            if buchi.remove_init_attr:
                nx.set_node_attributes(pruned_subgraph, {init_state: buchi.remove_init_attr})
            # check whether final locations satisfy the self-loop of the accept state
            if buchi.ap_sat_label(pruned_subgraph.nodes[accept_state]['label'],
                                  pruned_subgraph.nodes[accept_state]['neg_label']):
                end = datetime.now()
                print('total time for the prefix parts: {0}'.format((end - start).total_seconds()))
                cost = compute_path_cost(robot_path_pre)
                best_cost.append(cost)
                if min(best_cost) >= cost:
                    best_path = robot_path_pre

                print('the total cost of the found path is: ', min(best_cost), best_cost)
                print_red_on_cyan(task.formula)
                print_red_on_cyan([init_state, accept_state, buchi.size,
                                   [pruned_subgraph.number_of_nodes(), pruned_subgraph.number_of_edges()],
                                  'A path is found for the case where the accepting state has a self-loop'])
                if draw:
                    vis(workspace, robot_path_pre, {robot: [len(path)] * 2 for robot, path in robot_path_pre.items()},
                        [])
                if one_time:
                    return
                else:
                    continue

            # ======================================= suffix part =================================================#
            #                                                                                                      #
            #                                                                                                      #
            #                                                                                                      #
            # ======================================= suffix part =================================================#

            # ----------------- infer the poset -----------------------

            pruned_subgraph_suf, unpruned_subgraph_suf, paths_suf = buchi.get_subgraph(accept_state, accept_state,
                                                                                       'suffix', last_subtask)
            # no suffix graph due to that final locations of prefix part do not satisfy the outgoing edges
            # of the accepting vertex
            if not pruned_subgraph_suf:
                continue

            # no paths due to the implication does no t hold
            if not paths_suf:
                continue

            edge2element_suf, element2edge_suf = buchi.get_element(pruned_subgraph_suf)

            element_component2label_suf = buchi.element2label2eccl(element2edge_suf, pruned_subgraph_suf)

            hasse_graphs_suf = buchi.map_path_to_element_sequence(edge2element_suf, paths_suf)

            for _, poset_relation_suf, pos_suf, hasse_diagram_suf in hasse_graphs_suf:
                if show:
                    print_red_on_cyan('================ suffix part ================')

                    for order_suf in poset_relation_suf:
                        print(pruned_subgraph_suf.edges[element2edge_suf[order_suf[0]]]['formula'], ' -> ',
                              pruned_subgraph_suf.edges[element2edge_suf[order_suf[1]]]['formula'])
                    print('----------------------------------------------')

                robot2eccl_suf = restricted_poset.element2robot2eccl(pos_suf, element2edge_suf, pruned_subgraph_suf)

                incomparable_element_suf, larger_element_suf, smaller_element_suf, strict_larger_element_suf = \
                    restricted_poset.incomparable_larger(pos_suf, poset_relation_suf, hasse_diagram_suf)

                # --------------- construct the routing graph ---------------
                minimal_element_suf = [node for node in hasse_diagram_suf.nodes()
                                       if hasse_diagram_suf.out_degree(node) == 0]
                init_type_robot_node_suf, element_component_clause_literal_node_suf, \
                node_location_type_component_element_suf, \
                num_nodes_suf, final_element_type_robot_node \
                    = restricted_weighted_ts_suffix.construct_node_set(pos_suf, element2edge_suf, pruned_subgraph_suf,
                                                                       workspace.type_robot_label,
                                                                       minimal_element_suf, last_subtask, loop)

                edge_set_suf = restricted_weighted_ts_suffix.construct_edge_set(pos_suf,
                                                                                element_component_clause_literal_node_suf,
                                                                                element2edge_suf, pruned_subgraph_suf,
                                                                                element_component2label_suf,
                                                                                init_type_robot_node_suf,
                                                                                incomparable_element_suf,
                                                                                strict_larger_element_suf,
                                                                                larger_element_suf,
                                                                                buchi.imply,
                                                                                minimal_element_suf,
                                                                                final_element_type_robot_node)

                ts_suf = restricted_weighted_ts_suffix.construct_graph(num_nodes_suf,
                                                                       node_location_type_component_element_suf,
                                                                       edge_set_suf,
                                                                       workspace.p2p)

                # --------------------- MILP -------------------------
                maximal_element_suf = [node for node in hasse_diagram_suf.nodes()
                                       if hasse_diagram_suf.in_degree(node) == 0]

                robot_waypoint_suf, robot_time_suf, _, robot_label_suf, robot_waypoint_axis_suf, robot_time_axis_suf, \
                time_axis_suf, acpt_run_suf \
                    = restricted_milp_suf.construct_milp_constraint(ts_suf, workspace.type_num, pos_suf,
                                                                    pruned_subgraph_suf,
                                                                    element2edge_suf,
                                                                    element_component_clause_literal_node_suf,
                                                                    poset_relation_suf, init_type_robot_node_suf,
                                                                    strict_larger_element_suf, incomparable_element_suf,
                                                                    larger_element_suf,
                                                                    robot2eccl_suf, id2robots, accept_state, buchi,
                                                                    minimal_element_suf, final_element_type_robot_node,
                                                                    workspace.type_robot_label,
                                                                    maximal_element_suf, last_subtask, show, loop)
                if not robot_waypoint_suf:
                    continue

                for robot, time in list(robot_time_suf.items()):
                    # delete such robots that did not participate (the initial location of robots may just satisfies)
                    if time[-1] == 0 and len(time) == 1:
                        del robot_time_suf[robot]
                        del robot_waypoint_suf[robot]
                if show:
                    print('----------------------------------------------')
                    for type_robot, waypoint in robot_waypoint_suf.items():
                        print(type_robot, " : ", waypoint)
                        print(type_robot, " : ", robot_time_suf[type_robot])
                        print(type_robot, " : ", robot_label_suf[type_robot])
                    print('----------------------------------------------')

                    print('time axis: ', time_axis_suf)

                for robot, time in list(robot_time_axis_suf.items()):
                    # delete such robots that did not participate (the initial location of robots may just satisfies)
                    if not time:
                        del robot_time_axis_suf[robot]
                        del robot_waypoint_axis_suf[robot]
                if show:
                    for type_robot, waypoint in robot_waypoint_axis_suf.items():
                        print(type_robot, " : ", waypoint)
                        print(type_robot, " : ", robot_time_axis_suf[type_robot])

                    print('----------------------------------------------')

                    for stage in acpt_run_suf:
                        print(stage)
                    print('----------------------------------------------')

                robot_path_suf = mapp(workspace, buchi, acpt_run_suf, robot_waypoint_axis_suf,
                                      robot_time_axis_suf, 'simultaneous', show)

                # return to initial locations
                if not loop:
                    horizon = workspace.longest_time({robot: path[-1] for robot, path in robot_path_suf.items()},
                                                     workspace.type_robot_location)

                    acpt_run_suf = {'subtask': 'return',
                                    'time_element': [horizon, -1],
                                    'essential_robot_edge': {label: [type_robot]
                                                             for type_robot, label in
                                                             workspace.type_robot_label.items()},
                                    'essential_clause_edge': last_subtask['essential_clause_edge'],
                                    'neg_edge': last_subtask['neg_edge'],
                                    'essential_robot_vertex': last_subtask['essential_robot_edge'],
                                    'neg_vertex': last_subtask['neg_edge']
                                    }
                    robot_path_return = return_to_initial(workspace, acpt_run_suf, {robot: path[-1]
                                                                                    for robot, path in
                                                                                    robot_path_suf.items()}
                                                          )
                    for robot, path in robot_path_suf.items():
                        path += robot_path_return[robot][1:]

                end = datetime.now()
                print('total time for the prefix + suffix parts: {0}'.format((end - start).total_seconds()))

                robot_path = {robot: path + robot_path_suf[robot][1:] + robot_path_suf[robot][1:] for
                              robot, path in robot_path_pre.items()}

                cost = compute_path_cost({robot: path + robot_path_suf[robot][1:]
                                          for robot, path in robot_path_pre.items()})
                best_cost.append(cost)
                if min(best_cost) >= cost:
                    best_path = robot_path
                print('the total cost of the found path is: ', min(best_cost), best_cost)
                print_red_on_cyan(task.formula)
                print_red_on_cyan([init_state, accept_state, buchi.size,
                                   ([pruned_subgraph.number_of_nodes(), pruned_subgraph.number_of_edges()],
                                    [pruned_subgraph_suf.number_of_nodes(), pruned_subgraph_suf.number_of_edges()]),
                                   'A path is found for the case where the accepting state does not have a self-loop'])
                if draw:
                    vis(workspace, robot_path, {robot: [len(path)] * 2 for robot, path in robot_path.items()}, [])
                if one_time:
                    return
    return best_cost


if __name__ == '__main__':
    ltl_mrta(None)
