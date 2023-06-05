from task import Task
from z_restricted_buchi_parse import Buchi
from datetime import datetime
import z_restricted_poset
# from workspace import Workspace
from workspace_dars import Workspace
import matplotlib.pyplot as plt
import z_restricted_weighted_ts
import z_restricted_weighted_ts_suffix
import z_restricted_milp
import z_restricted_milp_suf
import pickle
from vis import plot_workspace
import numpy
from post_processing import run
# from MAPP_heuristic import mapp
from z_restricted_GMAPP import mapp, compute_path_cost
from vis import vis
import sys
from termcolor import colored, cprint

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
    start = datetime.now()
    # --------------- constructing the Buchi automaton ----------------
    task = Task(formula)
    print(task.formula)
    buchi = Buchi(task, workspace)

    buchi.construct_buchi_graph()
    # print('partial time: {0}'.format((datetime.now() - start).total_seconds()))

    init_acpt = buchi.get_init_accept()

    for pair, _ in init_acpt:
        init_state, accept_state = pair[0], pair[1]

        # determine whether self-loop exists for the accepting state
        self_loop_label = buchi.buchi_graph.nodes[accept_state]['label']
        is_nonempty_self_loop = self_loop_label == '1' or self_loop_label != []

        print_red_on_cyan('================ prefix part ================')

        # ----------------- infer the poset -----------------------
        pruned_subgraph, unpruned_subgraph, paths = buchi.get_subgraph(init_state, accept_state,
                                                                       is_nonempty_self_loop,
                                                                       'prefix')
        edge2element, element2edge = buchi.get_element(pruned_subgraph)
        if not edge2element:
            continue

        element_component2label = buchi.element2label2eccl(element2edge, pruned_subgraph)

        hasse_graphs = buchi.map_path_to_element_sequence(edge2element, paths)
        # loop over all posets
        for _, poset_relation, pos, hasse_diagram in hasse_graphs:

            # ----------------- restore after the suffix if not succeed -----------------
            workspace.type_robot_location = type_robot_location.copy()
            workspace.update_after_prefix()
            buchi.atomic_prop = workspace.atomic_prop

            robot2eccl = z_restricted_poset.element2robot2eccl(pos, element2edge, pruned_subgraph)

            # print('partial time to poset: {0}'.format((datetime.now() - start).total_seconds()))

            for order in poset_relation:
                print(pruned_subgraph.edges[element2edge[order[0]]]['formula'], ' -> ',
                      pruned_subgraph.edges[element2edge[order[1]]]['formula'])
            print('----------------------------------------------')

            incomparable_element, larger_element = z_restricted_poset.incomparable_larger(pos, hasse_diagram,
                                                                                          pruned_subgraph,
                                                                                          element2edge)

            # --------------- construct the routing graph ---------------
            init_type_robot_node, element_component_clause_literal_node, node_location_type_component_element, num_nodes = \
                z_restricted_weighted_ts.construct_node_set(pos, element2edge, pruned_subgraph,
                                                            workspace.type_robot_label)

            edge_set = z_restricted_weighted_ts.construct_edge_set(pos, element_component_clause_literal_node,
                                                                   element2edge, pruned_subgraph,
                                                                   element_component2label,
                                                                   init_type_robot_node,
                                                                   incomparable_element, larger_element)

            ts = z_restricted_weighted_ts.construct_graph(num_nodes, node_location_type_component_element, edge_set,
                                                          workspace.p2p)

            print('partial time before milp: {0}'.format((datetime.now() - start).total_seconds()))

            # --------------------- MILP -------------------------
            robot_waypoint_pre, robot_time_pre, id2robots, robot_label_pre, \
            robot_waypoint_axis, robot_time_axis, time_axis, acpt_run, \
                = z_restricted_milp.construct_milp_constraint(ts, workspace.type_num, pos,
                                                              pruned_subgraph,
                                                              element2edge,
                                                              element_component_clause_literal_node,
                                                              poset_relation,
                                                              init_type_robot_node,
                                                              incomparable_element,
                                                              larger_element,
                                                              robot2eccl, init_state,
                                                              buchi,
                                                              is_nonempty_self_loop)
            if not robot_waypoint_pre:
                continue

            end = datetime.now()

            for robot, time in list(robot_time_pre.items()):
                #  delete such robots that did not participate (the initial location of robots may just satisfies)
                if time[-1] == 0 and len(time) == 1:
                    del robot_time_pre[robot]
                    del robot_waypoint_pre[robot]

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

            for type_robot, waypoint in robot_waypoint_axis.items():
                print(type_robot, " : ", waypoint)
                print(type_robot, " : ", robot_time_axis[type_robot])

            print('----------------------------------------------')

            for stage in acpt_run:
                print(stage)
            print('----------------------------------------------')

            robot_path_pre = mapp(workspace, buchi, acpt_run, robot_waypoint_axis, robot_time_axis,
                                  is_nonempty_self_loop, 'simultaneous')

            # vis(workspace, robot_path_pre, {robot: [len(path)] * 2 for robot, path in robot_path_pre.items()},
            #     [])
            if is_nonempty_self_loop:
                print_red_on_cyan(task.formula)
                print_red_on_cyan('A path is found for the case where the accepting state has a self-loop')
                vis(workspace, robot_path_pre, {robot: [len(path)] * 2 for robot, path in robot_path_pre.items()},
                    [])

                return

            # ======================================= suffix part ==================================================
            print_red_on_cyan('================ suffix part ================')
            if not is_nonempty_self_loop:
                # ----------------- update after the prefix -----------------
                workspace.type_robot_location = {robot: path[-1] for robot, path in robot_path_pre.items()}
                workspace.update_after_prefix()
                buchi.atomic_prop = workspace.atomic_prop

                # ----------------- infer the poset -----------------------

                pruned_subgraph, unpruned_subgraph, paths = buchi.get_subgraph(accept_state, accept_state, False,
                                                                               'suffix')
                # no suffix graph due to that final locations of prefix part do not satisfy the outgoing edges
                # of the accepting vertex
                if not pruned_subgraph:
                    continue

                buchi.implication_check(pruned_subgraph, paths)
                # no paths due to the implication does not hold
                if not paths:
                    continue

                edge2element, element2edge = buchi.get_element(pruned_subgraph)

                element_component2label = buchi.element2label2eccl(element2edge, pruned_subgraph)

                hasse_graphs = buchi.map_path_to_element_sequence(edge2element, paths)

                for _, poset_relation, pos, hasse_diagram in hasse_graphs:

                    for order in poset_relation:
                        print(pruned_subgraph.edges[element2edge[order[0]]]['formula'], ' -> ',
                              pruned_subgraph.edges[element2edge[order[1]]]['formula'])
                    print('----------------------------------------------')

                    robot2eccl = z_restricted_poset.element2robot2eccl(pos, element2edge, pruned_subgraph)

                    incomparable_element, larger_element = z_restricted_poset.incomparable_larger(pos, hasse_diagram,
                                                                                                  pruned_subgraph,
                                                                                                  element2edge)
                    # --------------- construct the routing graph ---------------
                    minimal_element = [node for node in hasse_diagram.nodes() if hasse_diagram.out_degree(node) == 0]
                    init_type_robot_node, element_component_clause_literal_node, node_location_type_component_element, \
                    num_nodes, final_element_type_robot_node \
                        = z_restricted_weighted_ts_suffix.construct_node_set(pos,
                                                                             element2edge,
                                                                             pruned_subgraph,
                                                                             workspace.type_robot_label,
                                                                             minimal_element)

                    edge_set = z_restricted_weighted_ts_suffix.construct_edge_set(pos,
                                                                                  element_component_clause_literal_node,
                                                                                  element2edge, pruned_subgraph,
                                                                                  element_component2label,
                                                                                  init_type_robot_node,
                                                                                  incomparable_element,
                                                                                  larger_element, minimal_element,
                                                                                  final_element_type_robot_node)

                    ts = z_restricted_weighted_ts_suffix.construct_graph(num_nodes,
                                                                         node_location_type_component_element, edge_set,
                                                                         workspace.p2p)

                    # --------------------- MILP -------------------------
                    robot_waypoint_suf, robot_time_suf, _, robot_label_suf, robot_waypoint_axis, robot_time_axis, \
                    time_axis, acpt_run \
                        = z_restricted_milp_suf.construct_milp_constraint(ts,
                                                                          workspace.type_num,
                                                                          pos,
                                                                          pruned_subgraph,
                                                                          element2edge,
                                                                          element_component_clause_literal_node,
                                                                          poset_relation,
                                                                          init_type_robot_node,
                                                                          incomparable_element,
                                                                          larger_element,
                                                                          robot2eccl,
                                                                          id2robots,
                                                                          accept_state,
                                                                          buchi,
                                                                          is_nonempty_self_loop,
                                                                          minimal_element,
                                                                          final_element_type_robot_node,
                                                                          workspace.type_robot_label)
                    if not robot_waypoint_suf:
                        continue

                    for robot, time in list(robot_time_suf.items()):
                        # delete such robots that did not participate (the initial location of robots may just satisfies)
                        if time[-1] == 0 and len(time) == 1:
                            del robot_time_suf[robot]
                            del robot_waypoint_suf[robot]

                    print('----------------------------------------------')
                    for type_robot, waypoint in robot_waypoint_suf.items():
                        print(type_robot, " : ", waypoint)
                        print(type_robot, " : ", robot_time_suf[type_robot])
                        print(type_robot, " : ", robot_label_suf[type_robot])
                    print('----------------------------------------------')

                    print('time axis: ', time_axis)

                    for robot, time in list(robot_time_axis.items()):
                        # delete such robots that did not participate (the initial location of robots may just satisfies)
                        if not time:
                            del robot_time_axis[robot]
                            del robot_waypoint_axis[robot]

                    for type_robot, waypoint in robot_waypoint_axis.items():
                        print(type_robot, " : ", waypoint)
                        print(type_robot, " : ", robot_time_axis[type_robot])

                    print('----------------------------------------------')

                    for stage in acpt_run:
                        print(stage)
                    print('----------------------------------------------')

                    robot_path_suf = mapp(workspace, buchi, acpt_run, robot_waypoint_axis, robot_time_axis,
                                          is_nonempty_self_loop,
                                          'simultaneous')

                    end = datetime.now()
                    print('total time for the prefix + suffix parts: {0}'.format((end - start).total_seconds()))

                    robot_path = {robot: path + robot_path_suf[robot][1:] + robot_path_suf[robot][1:] for
                                  robot, path in robot_path_pre.items()}

                    cost = compute_path_cost({robot: path + robot_path_suf[robot][1:]
                                              for robot, path in robot_path_pre.items()})
                    print('the total cost of the found path is: ', cost)
                    print_red_on_cyan(task.formula)
                    print_red_on_cyan(
                        'A path is found for the case where the accepting state does not have a self-loop')
                    vis(workspace, robot_path, {robot: [len(path)] * 2 for robot, path in robot_path.items()}, [])

                    return


if __name__ == '__main__':
    ltl_mrta(None)
