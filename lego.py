from workspace_case1 import Workspace
import itertools
import networkx as nx
import pickle
from task import Task
from restricted_buchi_parse import Buchi
from datetime import datetime
import restricted_poset
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
from vis import vis
import sys
from termcolor import colored, cprint
from sympy.logic.boolalg import to_dnf
import numpy as np
from collections import namedtuple

print_red_on_cyan = lambda x: cprint(x, 'blue', 'on_red')

PrimitiveSubtask = namedtuple('PrimitiveSubtask', ['element_in_poset'])
CompositeSubtask = namedtuple('CompositeSubtask', ['subtask2element'])
Hierarchy = namedtuple('Hierarchy', ['level', 'formula', 'buchi_graph', 'hass_graphs', 'element2edge'])
PrimitiveSubtaskId = namedtuple('PrimitiveSubtaskId', ['parent', 'element'])

def get_ordered_subtasks(task):
    workspace = Workspace()
    with open('data/workspace', 'wb') as filehandle:
        pickle.dump(workspace, filehandle)
    type_robot_location = workspace.type_robot_location.copy()
    buchi = Buchi(task, workspace)
    buchi.construct_buchi_graph()
    init_acpt = buchi.get_init_accept()
    for pair, _ in init_acpt:
        workspace.type_robot_location = type_robot_location.copy()
        workspace.update_after_prefix()
        buchi.atomic_prop = workspace.atomic_prop
        buchi.regions = workspace.regions

        init_state, accept_state = pair[0], pair[1]
        # ----------------- infer the poset -----------------------
        pruned_subgraph, unpruned_subgraph, paths = buchi.get_subgraph(init_state, accept_state, 'prefix')
        edge2element, element2edge = buchi.get_element(pruned_subgraph)
        if not edge2element:
            continue
        element_component2label = buchi.element2label2eccl(element2edge, pruned_subgraph)
        hasse_graphs = buchi.map_path_to_element_sequence(edge2element, paths)
        return pruned_subgraph, hasse_graphs, element2edge


def print_partial_order(pruned_subgraph, hasse_graphs, element2edge):
    for (w, h), poset_relation, _, _ in hasse_graphs:
        # print("--------- w {0}, h {1} ---------".format(w, h))
        for order in poset_relation:
            print("pairwise order: ", pruned_subgraph.edges[element2edge[order[0]]]['formula'], ' -> ',
                    pruned_subgraph.edges[element2edge[order[1]]]['formula'], "           subtasks: ", element2edge[order[0]],
                    element2edge[order[1]])
        break

def get_primitive_and_composite_subtask(pruned_subgraph, hasse_graphs, element2edge):
    primitive_elements = set()
    composite_elements = set()
    composite_subtask_element_dict = dict()
    for element in element2edge:
        task_label = pruned_subgraph.edges[element2edge[element]]['label']
        if is_primitive(task_label):
            primitive_elements.add(element)
        elif task_label != "1":
            composite_elements.add(element)
            for clause in task_label:
                subtask  = clause[0][0]
                if subtask not in composite_subtask_element_dict.keys():
                    composite_subtask_element_dict[subtask] = [element]
                else:
                    composite_subtask_element_dict[subtask].append(element)

    return primitive_elements, composite_elements, composite_subtask_element_dict

def is_primitive(label):
    # task is primitive if exist literal with "location" smaller than 100
    # TODO what if a disjunctive of composite subtask | primitive subtask
    if label != '1':
        for clause in label:
            assert(len(clause) == 1)
            subtask = clause[0][0]
            is_primitive = int(subtask[1:]) < 100
            if is_primitive:
                return True
    return False

def print_primitive_subtasks(pruned_subgraph, element2edge, primitive_subtasks):
    for primitive_subtask in primitive_subtasks:
        edge = element2edge[primitive_subtask]
        print("primitive element {2}, subtask: {0}, label: {1}".format(edge, pruned_subgraph.edges[edge]["label"], primitive_subtask))

def print_composite_subtasks(pruned_subgraph, element2edge, composite_subtasks_dict):
    for (subtask, elements) in composite_subtasks_dict.items():
        for element in elements:
            print("composite subtask {0}, element {1}, label {2}".format(subtask, element, pruned_subgraph.edges[element2edge[element]]["label"]))

def get_task_specification():
    hierarchy = []
    level_one = dict()
    level_one["l0"] = "<> (l100_1_1_0 && <> l1_1_1_0)"
    hierarchy.append(level_one)

    level_two = dict()
    level_two["l100"] = "<> (l2_1_1_0 && <> l4_1_1_0)"
    # level_two["l200"] = "<> (l3_1_1_0 && <> (l5_1_1_0 || l1_1_1_0)) && <> l6_1_1_0 && <> !l6_1_1_0 U l3_1_1_0"
    hierarchy.append(level_two)
    return hierarchy

def build_buchi_graph_and_poset(task_specification):
    primitive_subtasks = dict()
    composite_subtasks = dict()
    task_hierarchy = dict()
    for index, level in enumerate(task_specification):
        for (subtask, formula) in level.items():
            task = Task()
            task.update_task(formula)
            pruned_subgraph, hasse_graphs, element2edge = get_ordered_subtasks(task)
            task_hierarchy[subtask] = Hierarchy(level=index+1, formula=formula, buchi_graph=pruned_subgraph, hass_graphs=hasse_graphs, element2edge=element2edge)
            primitive_elements, composite_element, composite_subtask_element_dict = get_primitive_and_composite_subtask(pruned_subgraph, hasse_graphs, element2edge)
            primitive_subtasks[subtask] = PrimitiveSubtask(element_in_poset=primitive_elements)
            composite_subtasks[subtask] = CompositeSubtask(subtask2element=composite_subtask_element_dict)
    return task_hierarchy, primitive_subtasks, composite_subtasks

def print_subtask_info(task_hierarchy, primitive_subtasks, composite_subtasks):
    for (task, hierarchy) in task_hierarchy.items():
        print_red_on_cyan(">>>>>> level {0} task {1}, formula {2}".format(hierarchy.level, task, hierarchy.formula))
        print_partial_order(hierarchy.buchi_graph, hierarchy.hass_graphs, hierarchy.element2edge)
        print_primitive_subtasks(hierarchy.buchi_graph, hierarchy.element2edge, primitive_subtasks[task].element_in_poset)
        print_composite_subtasks(hierarchy.buchi_graph, hierarchy.element2edge, composite_subtasks[task].subtask2element)

def get_task_seq(target_task, task_hierarchy, composite_subtasks):
    task_seq = [target_task]
    level = task_hierarchy[target_task].level
    while level > 1:
        for (task, hierarchy) in task_hierarchy.items():
            if hierarchy.level == level - 1 and target_task in composite_subtasks.keys():
                task_seq.append(task)
                level = level - 1
                break
    return task_seq
        
     
def produce_global_poset(task_hierarchy, composite_subtasks, primitive_subtasks):
    # get partial order within single composite task
    primitive_subtasks_partial_order = produce_global_poset_within_composite_subtask(task_hierarchy, primitive_subtasks)
    # get partial order between composite tasks
    primitive_subtasks_partial_order.extend(produce_global_poset_from_composite_subtask_pair(task_hierarchy, composite_subtasks, primitive_subtasks))
    # # get partial order from composite subtask with primitive subtask in the first level
    # primitive_subtasks_partial_order.extend(produce_global_poset_from_composite_and_primitive_subtasks(task_hierarchy, composite_subtasks, primitive_subtasks))
    primitive_subtasks_with_identifier = []
    for task in task_hierarchy.keys():
        for primitive_subtask in primitive_subtasks[task].element_in_poset:
            primitive_subtasks_with_identifier.append(PrimitiveSubtaskId(parent=task, element=primitive_subtask))
    return primitive_subtasks_with_identifier, primitive_subtasks_partial_order

def produce_global_poset_within_composite_subtask(task_hierarchy, primitive_subtasks):
    primitive_subtasks_partial_order = []
    for (task, hierarchy) in task_hierarchy.items():
        buchi_graph = hierarchy.buchi_graph
        element2edge = hierarchy.element2edge
        hass_graph = hierarchy.hass_graphs[0]
        poset_relation = hass_graph[1]
        primitive_elements = primitive_subtasks[task].element_in_poset
        checked_primitive_pairs = []
        for ele_a in primitive_elements:
            for ele_b in primitive_elements:
                if ele_a == ele_b or ((ele_a, ele_b) in checked_primitive_pairs or (ele_b, ele_a) in checked_primitive_pairs):
                    continue
                checked_primitive_pairs.append((ele_a, ele_b))
                if (ele_a, ele_b) in poset_relation:
                    primitive_subtasks_partial_order.append((PrimitiveSubtaskId(parent=task, element=ele_a), PrimitiveSubtaskId(parent=task, element=ele_b)))
                    print("within composite subtask {0} element: {1} -> {2} label: {3} -> {4}".\
                        format(task, ele_a, ele_b, buchi_graph.edges[element2edge[ele_a]]["label"],\
                            buchi_graph.edges[element2edge[ele_b]]["label"]))
                elif (ele_a, ele_b) in poset_relation:
                    primitive_subtasks_partial_order.append((PrimitiveSubtaskId(parent=task, element=ele_b), PrimitiveSubtaskId(parent=task, element=ele_a)))
                    print("within composite subtask {0} element: {1} -> {2} label: {3} -> {4}".\
                        format(task, ele_b, ele_a, buchi_graph.edges[element2edge[ele_b]]["label"],\
                            buchi_graph.edges[element2edge[ele_a]]["label"]))
                else:
                    print("within composite subtask {0} element: {1} || {2} label: {3} || {4}".\
                        format(task, ele_a, ele_b, buchi_graph.edges[element2edge[ele_a]]["label"],\
                            buchi_graph.edges[element2edge[ele_b]]["label"]))
    return primitive_subtasks_partial_order
                
    
def produce_global_poset_from_composite_subtask_pair(task_hierarchy, composite_subtasks, primitive_subtasks):
    # get the smallest common ancestor of any two composite subtasks
    Parents = namedtuple('Parents', ['parent_front', 'parent_back', 'common_parent'])
    composite_subtask_pair_with_parent = dict()
    for task_a in task_hierarchy.keys():
        task_a_seq = get_task_seq(task_a, task_hierarchy, composite_subtasks)
        for task_b in task_hierarchy.keys():
            if task_a == task_b or ((task_a, task_b) in composite_subtask_pair_with_parent.keys() or 
                                     (task_b, task_a) in composite_subtask_pair_with_parent.keys()):
                continue
            task_b_seq = get_task_seq(task_b, task_hierarchy, composite_subtasks)
            parent = find_shared_element(task_a_seq, task_b_seq)
            assert(parent != None)
            task_a_parent_index = max(1, task_a_seq.index(parent))
            task_b_parent_index = max(1, task_b_seq.index(parent))
            # if parent == task_a or parent == task_b:
            #     continue
            composite_subtask_pair_with_parent[(task_a, task_b)] = Parents(parent_front=task_a_seq[task_a_parent_index-1],
                                                                           parent_back=task_b_seq[task_b_parent_index-1],
                                                                           common_parent=parent)
    for (subtask_pair, parents) in composite_subtask_pair_with_parent.items():
        print("subtask {0} {1}, parent {2} {3}, common {4}".format(subtask_pair[0], subtask_pair[1], \
            parents.parent_front, parents.parent_back, parents.common_parent))
    # get partial order based on composite tasks
    primitive_subtasks_partial_order = []
    for (subtask_pair, parents) in composite_subtask_pair_with_parent.items():
        hass_graph = task_hierarchy[parents.common_parent].hass_graphs[0]
        poset_relation = hass_graph[1]
        buchi_graph = task_hierarchy[parents.common_parent].buchi_graph
        element2edge =task_hierarchy[parents.common_parent].element2edge
        one_is_parent_of_the_other = subtask_pair[0] == parents.common_parent or subtask_pair[1] == parents.common_parent
        # if ont task is not the parent of the other task, then loop over all combinations of primitive subtasks
        if not one_is_parent_of_the_other:
            composite_subtask_element_dict = composite_subtasks[parents.common_parent]
            parent_element_from_pair0 = composite_subtask_element_dict.subtask2element[parents.parent_front][0]
            parent_element_from_pair1 = composite_subtask_element_dict.subtask2element[parents.parent_back][0]
            # all child tasks of a task inherit the parital order w.r.t. all child tasks of a nother task
            if (parent_element_from_pair0, parent_element_from_pair1) in poset_relation:
                primitive_subtasks_partial_order.extend([(PrimitiveSubtaskId(parent=subtask_pair[0], element=pre), PrimitiveSubtaskId(parent=subtask_pair[1], element=suc)) \
                for pre in primitive_subtasks[subtask_pair[0]].element_in_poset for suc in primitive_subtasks[subtask_pair[1]].element_in_poset])
                print("[composite subtask {0} -> {1},  parent {2} {3} common {4}] parent element: {5} -> {6} parent label: {7} -> {8}".\
                    format(subtask_pair[0], subtask_pair[1], parents.parent_front, parents.parent_back, parents.common_parent, \
                        parent_element_from_pair0, parent_element_from_pair1, buchi_graph.edges[element2edge[parent_element_from_pair0]]["label"],\
                        buchi_graph.edges[element2edge[parent_element_from_pair1]]["label"]))
            elif (parent_element_from_pair1, parent_element_from_pair0) in poset_relation:
                primitive_subtasks_partial_order.extend([(PrimitiveSubtaskId(parent=subtask_pair[1], element=pre), PrimitiveSubtaskId(parent=subtask_pair[0], element=suc)) \
                for pre in primitive_subtasks[subtask_pair[1]].element_in_poset for suc in primitive_subtasks[subtask_pair[0]].element_in_poset])
                print("[composite subtask {0} -> {1}, parent {2} {3} common {4}] parent element: {5} -> {6} parent label: {7} -> {8}".\
                    format(subtask_pair[1], subtask_pair[0], parents.parent_back, parents.parent_front, parents.common_parent, \
                        parent_element_from_pair1, parent_element_from_pair0, buchi_graph.edges[element2edge[parent_element_from_pair1]]["label"],\
                        buchi_graph.edges[element2edge[parent_element_from_pair0]]["label"]))
            else:
                print("[composite subtask {0} || {1}, parent {2} {3} common {4}] parent element: {5} || {6} parent label: {7} || {8}".\
                    format(subtask_pair[0], subtask_pair[1], parents.parent_front, parents.parent_back, parents.common_parent,\
                        parent_element_from_pair0, parent_element_from_pair1, buchi_graph.edges[element2edge[parent_element_from_pair0]]["label"],\
                        buchi_graph.edges[element2edge[parent_element_from_pair1]]["label"]))
        # if one task is the parent of the other task, then got the primitive subtasks of parent and composite subtask of child
        else:
            child = subtask_pair[0]
            parent_of_child = parents.parent_front
            if parents.common_parent == subtask_pair[0]:
                child = subtask_pair[1]
                parent_of_child = parents.parent_back
            for primitive_element in primitive_subtasks[parents.common_parent].element_in_poset:
                composite_element = composite_subtasks[parents.common_parent].subtask2element[child][0]
                # all child tasks of a task inherit the parital order w.r.t. all child tasks of a nother task
                if (primitive_element, composite_element) in poset_relation:
                    primitive_subtasks_partial_order.extend([(PrimitiveSubtaskId(parent=parents.common_parent, element=primitive_element), PrimitiveSubtaskId(parent=child, element=suc)) \
                    for suc in primitive_subtasks[child].element_in_poset])
                    print("[comp subtask {0}, parent {1} common {2}] parent element: {3} -> {4} parent label: {5} -> {6}".\
                        format(child, parent_of_child, parents.common_parent, primitive_element, composite_element, \
                            buchi_graph.edges[element2edge[primitive_element]]["label"],\
                            buchi_graph.edges[element2edge[composite_element]]["label"]))
                elif (composite_element, primitive_element) in poset_relation:
                    primitive_subtasks_partial_order.extend([(PrimitiveSubtaskId(parent=child, element=pre), PrimitiveSubtaskId(parent=parents.common_parent, element=primitive_element)) \
                    for pre in primitive_subtasks[child].element_in_poset])
                    print("[comp subtask {0}, parent {1} common {2}] parent element: {3} -> {4} parent label: {5} -> {6}".\
                        format(child, parent_of_child, parents.common_parent, composite_element, primitive_element, \
                            buchi_graph.edges[element2edge[composite_element]]["label"],\
                            buchi_graph.edges[element2edge[primitive_element]]["label"]))
                else:
                    print("[comp subtask {0}, parent {1} common {2}] parent element: {3} || {4} parent label: {5} || {6}".\
                        format(child, parent_of_child, parents.common_parent, primitive_element, composite_element, \
                            buchi_graph.edges[element2edge[primitive_element]]["label"],\
                            buchi_graph.edges[element2edge[composite_element]]["label"]))
    return primitive_subtasks_partial_order

def produce_global_poset_from_composite_and_primitive_subtasks(task_hierarchy, composite_subtasks, primitive_subtasks):
    # get the partial order betwen two primitive subtasks in the first level
    primitive_subtasks_partial_order = []
    first_level_task = "l0"
    first_level_hierarchy = task_hierarchy[first_level_task]
    hass_graph = first_level_hierarchy.hass_graphs[0]
    poset_relation = hass_graph[1]
    element2edge = first_level_hierarchy.element2edge
    buchi_graph = first_level_hierarchy.buchi_graph
    checked_primitive_tasks = []
    for primitive_task_a in primitive_subtasks[first_level_task].element_in_poset:
        for primitive_task_b in primitive_subtasks[first_level_task].element_in_poset:
            if primitive_task_a == primitive_task_b or ((primitive_task_a, primitive_task_b) in checked_primitive_tasks or \
                (primitive_task_b, primitive_task_a) in checked_primitive_tasks):
                continue
            checked_primitive_tasks.append((primitive_task_a, primitive_task_b))
            # all child tasks of a task inherit the parital order w.r.t. all child tasks of a nother task
            if (primitive_task_a, primitive_task_b) in poset_relation:
                primitive_subtasks_partial_order.append((PrimitiveSubtaskId(parent=first_level_task, element=primitive_task_a), PrimitiveSubtaskId(parent=first_level_task, element=primitive_task_b)))
                print("[primitive: parent {0}] element: {1} -> {2} label: {3} -> {4}".\
                    format(first_level_task, primitive_task_a, primitive_task_b, buchi_graph.edges[element2edge[primitive_task_a]]["label"],\
                        buchi_graph.edges[element2edge[primitive_task_b]]["label"]))
            elif (primitive_task_b, primitive_task_a) in poset_relation:
                primitive_subtasks_partial_order.append((PrimitiveSubtaskId(parent=first_level_task, element=primitive_task_b), PrimitiveSubtaskId(parent=first_level_task, element=primitive_task_a)))
                print("[primitive: parent {0}] element: {1} -> {2} label: {3} -> {4}".\
                    format(first_level_task, primitive_task_b, primitive_task_a, buchi_graph.edges[element2edge[primitive_task_b]]["label"],\
                        buchi_graph.edges[element2edge[primitive_task_a]]["label"]))
            else:
                print("[primitive: parent {0}] element: {1} || {2} label: {3} -> {4}".\
                    format(first_level_task, primitive_task_a, primitive_task_b, buchi_graph.edges[element2edge[primitive_task_a]]["label"],\
                        buchi_graph.edges[element2edge[primitive_task_b]]["label"]))
    # get partial order based on composite tasks
    for primitive_element in primitive_subtasks[first_level_task].element_in_poset:
        for composite_task in task_hierarchy.keys():
            if composite_task == first_level_task:
                continue
            composite_task_seq = get_task_seq(composite_task, task_hierarchy, composite_subtasks)
            assert(len(composite_task_seq) > 1)
            parent = composite_task_seq[-2]
            composite_element = composite_subtasks[first_level_task].subtask2element[parent][0]
            # all child tasks of a task inherit the parital order w.r.t. all child tasks of a nother task
            if (primitive_element, composite_element) in poset_relation:
                primitive_subtasks_partial_order.extend([(PrimitiveSubtaskId(parent=first_level_task, element=primitive_element), PrimitiveSubtaskId(parent=composite_task, element=suc)) \
                 for suc in primitive_subtasks[composite_task].element_in_poset])
                print("[pre_comp subtask {0}, {1}, parent {2}] element: {3} -> {4} label: {5} -> {6}".\
                    format(primitive_element, composite_task, first_level_task, primitive_element, composite_element, buchi_graph.edges[element2edge[primitive_element]]["label"],\
                        buchi_graph.edges[element2edge[composite_element]]["label"]))
            elif (composite_element, primitive_element) in poset_relation:
                primitive_subtasks_partial_order.extend([(PrimitiveSubtaskId(parent=composite_task, element=pre), PrimitiveSubtaskId(parent=first_level_task, element=primitive_element)) \
                 for pre in primitive_subtasks[composite_task].element_in_poset])
                print("[comp_pre subtask {0}, {1}, parent {2}] element: {3} -> {4} label: {5} -> {6}".\
                    format(composite_task, primitive_element, first_level_task, composite_element, primitive_element, buchi_graph.edges[element2edge[composite_element]]["label"],\
                        buchi_graph.edges[element2edge[primitive_element]]["label"]))
            else:
                print("[pre_comp subtask {0}, {1}, parent {2}] element: {3} || {4} label: {5} -> {6}".\
                    format(primitive_element, composite_task, first_level_task, primitive_element, composite_element, buchi_graph.edges[element2edge[primitive_element]]["label"],\
                        buchi_graph.edges[element2edge[composite_element]]["label"]))
    
    return primitive_subtasks_partial_order

    # primitive_subtasks_partial_order = []
    # for (subtask_pair, parent) in composite_subtask_pair_with_parent.items():
    #     composite_subtask_element_dict = composite_subtasks[parent]
    #     element_from_pair0 = composite_subtask_element_dict.subtask2element[subtask_pair[0]][0]
    #     element_from_pair1 = composite_subtask_element_dict.subtask2element[subtask_pair[1]][0]
    #     hass_graph = task_hierarchy[parent].hass_graphs[0]
    #     poset_relation = hass_graph[1]
    #     buchi_graph = task_hierarchy[parent].buchi_graph
    #     element2edge =task_hierarchy[parent].element2edge
    #     # all child tasks of a task inherit the parital order w.r.t. all child tasks of a nother task
    #     if (element_from_pair0, element_from_pair1) in poset_relation:
    #         primitive_subtasks_partial_order.extend([(PrimitiveSubtaskId(parent=subtask_pair[0], element=pre), PrimitiveSubtaskId(parent=subtask_pair[1], element=suc)) \
    #         for pre in primitive_subtasks[subtask_pair[0]].element_in_poset for suc in primitive_subtasks[subtask_pair[1]].element_in_poset])
    #         print("[subtask {0}, {1}, parent {2}] element: {3} -> {4} label: {5} -> {6}".\
    #             format(subtask_pair[0], subtask_pair[1], parent, element_from_pair0, element_from_pair1, buchi_graph.edges[element2edge[element_from_pair0]]["label"],\
    #                 buchi_graph.edges[element2edge[element_from_pair1]]["label"]))
    #     elif (element_from_pair1, element_from_pair0) in poset_relation:
    #         primitive_subtasks_partial_order.extend([(PrimitiveSubtaskId(parent=subtask_pair[1], element=pre), PrimitiveSubtaskId(parent=subtask_pair[0], element=suc)) \
    #         for pre in primitive_subtasks[subtask_pair[1]].element_in_poset for suc in primitive_subtasks[subtask_pair[0]].element_in_poset])
    #         print("[{0}, {1}, parent {2}] element: {3} -> {4} label: {5} -> {6}".\
    #             format(subtask_pair[1], subtask_pair[0], parent, element_from_pair1, element_from_pair0, buchi_graph.edges[element2edge[element_from_pair1]]["label"],\
    #                 buchi_graph.edges[element2edge[element_from_pair0]]["label"]))
    #     else:
    #         print("[{0}, {1}, parent {2}] element: {3} || {4} label: {5} || {6}".\
    #             format(subtask_pair[0], subtask_pair[1], parent, element_from_pair0, element_from_pair1, buchi_graph.edges[element2edge[element_from_pair0]]["label"],\
    #                 buchi_graph.edges[element2edge[element_from_pair1]]["label"]))
    # return primitive_subtasks_partial_order
    
def print_primitive_subtasks_with_identifer(primitive_subtasks_with_identifer, task_hierarchy):
     # print all primitive subtasks with identifier
    print("************* all primitive subtasks **************")
    for ele in primitive_subtasks_with_identifer:
        element2edge = task_hierarchy[ele.parent].element2edge
        buchi_graph = task_hierarchy[ele.parent].buchi_graph
        print("parent {0}, element {1}, edge {2}, label {3}".format(ele.parent, ele.element, element2edge[ele.element], 
                                                                    buchi_graph.edges[element2edge[ele.element]]['label']))
            
def find_shared_element(vector1, vector2):
    for element in vector1:
        if element in vector2:
            return element
    return None  # No shared element found

def main():
    task_specification = get_task_specification()
    task_hierarchy, primitive_subtasks, composite_subtasks = build_buchi_graph_and_poset(task_specification)
    print_subtask_info(task_hierarchy, primitive_subtasks, composite_subtasks)
    primitive_subtasks_with_identifier, primitive_subtasks_partial_order = produce_global_poset(task_hierarchy, composite_subtasks, primitive_subtasks)
    print_primitive_subtasks_with_identifer(primitive_subtasks_with_identifier, task_hierarchy)
    print("************* partial order between primitive subtasks **************")
    for partial_order in primitive_subtasks_partial_order:
        pre = partial_order[0]
        suc = partial_order[1]
        pre_element2edge = task_hierarchy[pre.parent].element2edge
        pre_buchi_graph = task_hierarchy[pre.parent].buchi_graph
        suc_element2edge = task_hierarchy[suc.parent].element2edge
        suc_buchi_graph = task_hierarchy[suc.parent].buchi_graph
        print("parent {0}, element {1}, edge {2}, label {3} -> parent {4}, element {5}, edge {6}, label {7}".\
            format(pre.parent, pre.element, pre_element2edge[pre.element], pre_buchi_graph.edges[pre_element2edge[pre.element]]['label'],
                   suc.parent, suc.element, suc_element2edge[suc.element], suc_buchi_graph.edges[suc_element2edge[suc.element]]['label']))
        

if __builtins__:
    main()

# ----------- log --------------


# for seqsim in ['simultaneous', 'sequential']:
#     best_cost = []
#     best_path = dict()

#     for pair, _ in init_acpt:
#         workspace.type_robot_location = type_robot_location.copy()
#         workspace.update_after_prefix()
#         buchi.atomic_prop = workspace.atomic_prop
#         buchi.regions = workspace.regions

#         init_state, accept_state = pair[0], pair[1]

#         # ======================================= prefix part =================================================#
#         #                                                                                                      #
#         #                                                                                                      #
#         #                                                                                                      #
#         # ======================================= prefix part =================================================#

#         # ----------------- infer the poset -----------------------
#         pruned_subgraph, unpruned_subgraph, paths = buchi.get_subgraph(init_state, accept_state, 'prefix')
#         edge2element, element2edge = buchi.get_element(pruned_subgraph)
#         if not edge2element:
#             continue

#         element_component2label = buchi.element2label2eccl(element2edge, pruned_subgraph)

#         hasse_graphs = buchi.map_path_to_element_sequence(edge2element, paths)
#         # loop over all posets
#         for _, poset_relation, pos, hasse_diagram in hasse_graphs:
#             if show:
#                 print_red_on_cyan('================ prefix part ================')

#             # ----------------- restore after the suffix if not succeed -----------------
#             workspace.type_robot_location = type_robot_location.copy()
#             workspace.update_after_prefix()
#             buchi.atomic_prop = workspace.atomic_prop
#             buchi.regions = workspace.regions

#             robot2eccl = restricted_poset.element2robot2eccl(pos, element2edge, pruned_subgraph)

#             # print('partial time to poset: {0}'.format((datetime.now() - start).total_seconds()))

#             if show:
#                 for order in poset_relation:
#                     print("pairwise order: ", pruned_subgraph.edges[element2edge[order[0]]]['formula'], ' -> ',
#                           pruned_subgraph.edges[element2edge[order[1]]]['formula'])
#                 print('----------------------------------------------')

#             incomparable_element, larger_element, smaller_element, strict_larger_element = \
#                 restricted_poset.incomparable_larger(pos, poset_relation, hasse_diagram)

#             # --------------- construct the routing graph ---------------
#             init_type_robot_node, element_component_clause_literal_node, node_location_type_component_element, \
#             num_nodes = restricted_weighted_ts.construct_node_set(pos, element2edge, pruned_subgraph,
#                                                                   workspace.type_robot_label)

#             edge_set = restricted_weighted_ts.construct_edge_set(pos, element_component_clause_literal_node,
#                                                                  element2edge, pruned_subgraph,
#                                                                  element_component2label,
#                                                                  init_type_robot_node, incomparable_element,
#                                                                  strict_larger_element,
#                                                                  larger_element, buchi.imply)

#             ts = restricted_weighted_ts.construct_graph(num_nodes, node_location_type_component_element, edge_set,
#                                                         workspace.p2p)

#             if show:
#                 print('partial time before milp: {0}'.format((datetime.now() - start).total_seconds()))

#             # --------------------- MILP -------------------------
#             maximal_element = [node for node in hasse_diagram.nodes() if hasse_diagram.in_degree(node) == 0]

#             robot_waypoint_pre, robot_time_pre, id2robots, robot_label_pre, \
#             robot_waypoint_axis, robot_time_axis, time_axis, acpt_run, \
#                 = restricted_milp.construct_milp_constraint(ts, workspace.type_num, pos,
#                                                             pruned_subgraph,
#                                                             element2edge,
#                                                             element_component_clause_literal_node,
#                                                             poset_relation,
#                                                             init_type_robot_node,
#                                                             strict_larger_element,
#                                                             incomparable_element,
#                                                             larger_element,
#                                                             robot2eccl, init_state,
#                                                             buchi, maximal_element, show)
#             if not robot_waypoint_pre:
#                 continue

#             for robot, time in list(robot_time_pre.items()):
#                 #  delete such robots that did not participate (the initial location of robots may just satisfies)
#                 if time[-1] == 0 and len(time) == 1:
#                     del robot_time_pre[robot]
#                     del robot_waypoint_pre[robot]

#             if show:
#                 print('----------------------------------------------')
#                 for type_robot, waypoint in robot_waypoint_pre.items():
#                     print("waypoint (type, robot): ", type_robot, " : ", waypoint)
#                     print("time (type, robot): ", type_robot, " : ", robot_time_pre[type_robot])
#                     print("component (type, robot): ", type_robot, " : ", robot_label_pre[type_robot])
#                 print('----------------------------------------------')

#                 print('time axis: ', time_axis)

#             for robot, time in list(robot_time_axis.items()):
#                 #  delete such robots that did not participate (the initial location of robots may just satisfies)
#                 if not time:
#                     del robot_time_axis[robot]
#                     del robot_waypoint_axis[robot]

#             if show:
#                 for type_robot, waypoint in robot_waypoint_axis.items():
#                     print("waypoint (type, robot): ", type_robot, " : ", waypoint)
#                     print("time axis (type, robot): ", type_robot, " : ", robot_time_axis[type_robot])

#                 print('----------------------------------------------')

#                 for stage in acpt_run:
#                     print(stage)
#                 print('----------------------------------------------')
            
#             if one_time:
#                 exit()
