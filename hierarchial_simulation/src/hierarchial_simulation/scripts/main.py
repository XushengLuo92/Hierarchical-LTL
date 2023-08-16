from hierarchical_LTL import hierarchical_ltl_planner

import argparse
import rospy
def create_parser():
    """ create parser

    Returns:
        _type_: _description_
    """
    parser = argparse.ArgumentParser(description='FM')
    parser.add_argument('--task', default="man", type=str)
    parser.add_argument('--case', default=5, type=int)
    parser.add_argument('--vis', action='store_true', help='Enable visualization')
    parser.add_argument('--dot', action='store_true', help='Enable dot graph')

    return parser

def task_execution(time_task_element_type_robot_axis, reduced_task_network, type_robots):
    finished_tasks = []
    current_exec_tasks = []
    current_exec_robots = []
    candidate_tasks = set()
    
    task_element2type_robot = {time_task_element_type_robot[1] : time_task_element_type_robot[2] \
        for time_task_element_type_robot in time_task_element_type_robot_axis if len(time_task_element_type_robot) == 3}
    task_element2time =  {time_task_element_type_robot[1] : time_task_element_type_robot[0] \
        for time_task_element_type_robot in time_task_element_type_robot_axis if len(time_task_element_type_robot) == 3}
    print("task_element2type_robot: ", task_element2type_robot)
    print("task_element2time: ", task_element2time)
    executing_task_network = reduced_task_network.copy()
    
    invalid_str = "-1"
    finished_task_str = invalid_str
    while executing_task_network.nodes():
        # candidate tasks: 1. no parent in updated task network 2. no parent in executing tasks
        candidate_tasks = set(node for node in executing_task_network.nodes() \
            if executing_task_network.in_degree(node) == 0)
        removed_candidate_tasks = set(task for task in candidate_tasks for exec_task in current_exec_tasks \
            if (exec_task, task) in executing_task_network.edges())
        removed_candidate_tasks.update(removed_candidate_tasks)
        candidate_tasks.difference_update(removed_candidate_tasks)
        # deal with subtasks that are ignored due to or relation
        remove_task_element_from_network = set(task for task in candidate_tasks if task not in task_element2type_robot.keys())
        candidate_tasks.difference_update(remove_task_element_from_network)
        executing_task_network.remove_nodes_from(remove_task_element_from_network)
        
        for type_robot in type_robots:
            if type_robot in current_exec_robots:
                continue
            candidate_tasks_for_robot = [task_element for task_element in candidate_tasks \
                if task_element2type_robot[task_element] == type_robot]
            if not candidate_tasks_for_robot:
                continue
            # if there are several candiate tasks for a robot, sort according to task plan
            ordered_candidata_tasks = [task_element2time[task_element] for task_element in candidate_tasks_for_robot]
            task_index = ordered_candidata_tasks.index(min(ordered_candidata_tasks))

            current_exec_robots.append(type_robot)
            current_exec_tasks.append(candidate_tasks_for_robot[task_index])
            
            # generate message 
            is_human = type_robot == (1, 1)
            task_id = reduced_task_network.nodes[current_exec_tasks[-1]]['label'][0][0][0]
            print("is human {0}, task id {1}".format(is_human, task_id))
        # send message
        for idx, robot in enumerate(current_exec_robots):
            topic = 'fanuc_' + str(robot[0]) + '_' + str(robot[1])
            task = reduced_task_network.nodes[current_exec_tasks[idx]]['label'][0][0][0]
            
        # remove current subtasks
        executing_task_network.remove_nodes_from(current_exec_tasks)
        print("current_exec_robots: ", current_exec_robots)
        for task in current_exec_tasks:
            print("current_exec_tasks: ", task, reduced_task_network.nodes[task]['label'], 
                  reduced_task_network.nodes[task]['label'][0][0][0])
        
        # if finished_task_str == invalid_str:
        #     finished_task_str = input("Finished task: ")
        #     finished_task_split = finished_task_str.split()     
        #     finished_task_str = invalid_str
        #     # update when some tasks are finished
        #     finished_task = (finished_task_split[0], int(finished_task_split[1]))
        #     finished_tasks.append(finished_task)
        #     print("finished_task: ", finished_task)
        #     current_exec_tasks.remove(finished_task)
        #     current_exec_robots.remove(task_element2type_robot[finished_task])
            
        # (1, 1) human (1, 0) robot
        while finished_task_str == invalid_str:
            finished_task_str = input("Finished task: ")
            if finished_task_str != '0' and finished_task_str != '1':
                finished_task_str = invalid_str
        is_human = int(finished_task_str) == 1
        finished_task_str = invalid_str
        if is_human:
            task_idx = current_exec_robots.index((1,1))
        else:
            task_idx = current_exec_robots.index((1,0))
        # robot_id = get_from_topic()
        print("finished_task: ", current_exec_tasks[task_idx])
        current_exec_tasks.pop(task_idx)
        current_exec_robots.pop(task_idx)

if __builtins__:
    parser = create_parser()
    args = parser.parse_known_args()[0]
    
    if args.task == "man":
        time_task_element_type_robot_axis, reduced_task_network, type_robots = hierarchical_ltl_planner(args=args)
        task_execution(time_task_element_type_robot_axis, reduced_task_network, type_robots)
    else:
        hierarchical_ltl_planner(args=args)
#  python main.py --task man --case 5