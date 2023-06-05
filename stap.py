from task import Task
from buchi_parse_aggressive import Buchi
from datetime import datetime
import poset
# from workspace import Workspace
from workspace_dars import Workspace
import matplotlib.pyplot as plt
import weighted_ts
import milp
import milp_suf
import pickle
from vis import plot_workspace
import numpy
from post_processing import run
# from MAPP_heuristic import mapp
from GMAPP import mapp
from vis import vis


factor = 1

workspace = Workspace()
with open('data/workspace', 'wb') as filehandle:
    pickle.dump(workspace, filehandle)

# fig = plt.figure()
# ax = fig.add_subplot(111)
# plot_workspace(workspace, ax)
# plt.show()

# with open('data/workspace', 'rb') as filehandle:
#     workspace = pickle.load(filehandle)

start = datetime.now()
# --------------- constructing the Buchi automaton ----------------
task = Task()

buchi = Buchi(task, workspace)

buchi.construct_buchi_graph()
# print('partial time: {0}'.format((datetime.now() - start).total_seconds()))

init_state, accept_state, is_nonempty_self_loop, next_vertex = buchi.get_init_accept()

# ----------------- infer the poset -----------------------
pruned_subgraph, unpruned_subgraph, paths = buchi.get_subgraph(init_state, accept_state, is_nonempty_self_loop,
                                                               next_vertex)

# print('partial time to pruned_graph: {0}'.format((datetime.now() - start).total_seconds()))

edge2element, element2edge = buchi.get_element(pruned_subgraph)
if not edge2element:
    raise RuntimeError('The task is infeasible!')

element_component2label = buchi.element2label2eccl(element2edge, pruned_subgraph)

poset_relation, pos, hasse_diagram = buchi.map_path_to_element_sequence(edge2element, element2edge, pruned_subgraph, paths)

robot2eccl = poset.element2robot2eccl(pos, element2edge, pruned_subgraph)

# print('partial time to poset: {0}'.format((datetime.now() - start).total_seconds()))

for order in poset_relation:
    print(pruned_subgraph.edges[element2edge[order[0]]]['formula'], ' -> ',
          pruned_subgraph.edges[element2edge[order[1]]]['formula'])
print('----------------------------------------------')

incomparable_element, larger_element = poset.incomparable_larger(pos, hasse_diagram, pruned_subgraph,
                                                                 element2edge)

# --------------- construct the routing graph ---------------
init_type_robot_node, element_component_clause_literal_node, node_location_type_component_element, num_nodes = \
    weighted_ts.construct_node_set(pos, element2edge, pruned_subgraph, workspace.type_robot_label)

edge_set = weighted_ts.construct_edge_set(pos, element_component_clause_literal_node,
                                          element2edge, pruned_subgraph, element_component2label, init_type_robot_node,
                                          incomparable_element, larger_element)

ts = weighted_ts.construct_graph(num_nodes, node_location_type_component_element, edge_set, workspace.p2p, factor)


print('partial time before milp: {0}'.format((datetime.now() - start).total_seconds()))

# --------------------- MILP -------------------------
robot_waypoint_pre, robot_time_pre, id2robots, robot_label_pre, robot_waypoint_axis, robot_time_axis, \
time_axis, acpt_run, exe_robot_next_vertex = milp.construct_milp_constraint(ts, workspace.type_num, pos, pruned_subgraph, element2edge,
                                                     element_component_clause_literal_node, poset_relation,
                                                     init_type_robot_node, incomparable_element, larger_element,
                                                     robot2eccl, init_state, buchi, is_nonempty_self_loop)

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

robot_path_pre = mapp(workspace, buchi, acpt_run, robot_waypoint_axis, robot_time_axis, is_nonempty_self_loop,
                      'simultaneous')

print('total time for the prefix part:', (datetime.now()-start).total_seconds())

if is_nonempty_self_loop:
    vis(workspace, robot_path_pre, {robot: [len(path)] * 2 for robot, path in robot_path_pre.items()}, [])

# =========================================================================================================
print('=================================================================================================')
if not is_nonempty_self_loop:
    # ----------------- update after the prefix -----------------
    workspace.type_robot_location = {robot: path[-1] for robot, path in robot_path_pre.items()}
    workspace.update_after_prefix()
    buchi.atomic_prop = workspace.atomic_prop

    # ----------------- infer the poset -----------------------

    pruned_subgraph, unpruned_subgraph, paths = buchi.get_subgraph(next_vertex, accept_state, False, next_vertex)

    edge2element, element2edge = buchi.get_element(pruned_subgraph)

    element_component2label = buchi.element2label2eccl(element2edge, pruned_subgraph)

    poset_relation, pos, hasse_diagram = buchi.map_path_to_element_sequence(edge2element, element2edge, pruned_subgraph, paths)

    robot2eccl = poset.element2robot2eccl(pos, element2edge, pruned_subgraph)

    incomparable_element, larger_element = poset.incomparable_larger(pos, hasse_diagram, pruned_subgraph,
                                                                     element2edge)

    # --------------- construct the routing graph ---------------

    init_type_robot_node, element_component_clause_literal_node, node_location_type_component_element, num_nodes = \
        weighted_ts.construct_node_set(pos, element2edge, pruned_subgraph, workspace.type_robot_label)

    edge_set = weighted_ts.construct_edge_set(pos, element_component_clause_literal_node,
                                              element2edge, pruned_subgraph, element_component2label,
                                              init_type_robot_node, incomparable_element, larger_element)

    ts = weighted_ts.construct_graph(num_nodes, node_location_type_component_element, edge_set,
                                     workspace.p2p)

    # --------------------- MILP -------------------------
    init_state = next_vertex
    robot_waypoint_suf, robot_time_suf, id2robots, robot_label_suf, robot_waypoint_axis, robot_time_axis, \
    time_axis, acpt_run = milp_suf.construct_milp_constraint(ts, workspace.type_num, pos, pruned_subgraph,
                                                             element2edge, element_component_clause_literal_node,
                                                             poset_relation, init_type_robot_node, incomparable_element,
                                                             larger_element, robot2eccl, id2robots, init_state, buchi,
                                                             is_nonempty_self_loop)

    for robot, time in list(robot_time_suf.items()):
        #  delete such robots that did not participate (the initial location of robots may just satisfies)
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

    robot_path_suf = mapp(workspace, buchi, acpt_run, robot_waypoint_axis, robot_time_axis, is_nonempty_self_loop,
                          'simultaneous')

    end = datetime.now()
    print('total time for the prefix + suffix parts: {0}'.format((end - start).total_seconds()))

    robot_path = {robot: path + robot_path_suf[robot][1:] for robot, path in robot_path_pre.items()}
    vis(workspace, robot_path, {robot: [len(path)] * 2 for robot, path in robot_path.items()}, [])
