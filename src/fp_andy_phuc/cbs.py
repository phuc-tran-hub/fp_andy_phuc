#!/usr/bin/env python

# Name: Phuc Tran and Andy Kotz
# Task: Machine Learning Implementation of Conflict Based Search

import rospy 

import networkx as nx
import math
import random
import copy
import matplotlib.pyplot as plt 

# GOAL: Create a connected graph of robots, goals, and possible location nodes
def include_robot(robot_locations, goal_locations):
    """Include the robots' poses in the new tree."""
    # Initialize a location graph
    location_graph = nx.Graph()
    locations = []

    i = 0
    # For each of the robot, create a new node
    while i < len(robot_locations):
        robot_node_i = "/tb3_" + str(i)
        x = robot_locations[i][0]
        y = robot_locations[i][1]
        locations.append((x, y))
        location_graph.add_node(robot_node_i, location=[x, y])
        i += 1

    i = 0
    # For each of the goal, create a new node
    while i < len(goal_locations):
        goal_node_i = "/goal_" + str(i)
        x = goal_locations[i][0]
        y = goal_locations[i][1]
        locations.append((x, y))
        location_graph.add_node(goal_node_i, location=[x, y])
        i += 1

    return location_graph

def construct_graph(spanning_tree, num_node, threshold, grid_x, grid_y):
    """Create a spanning tree that covers all nodes."""
    # Default node name
    node_str = "node"
    locations = []
    # initialize the nodes in the graph
    i = 0
    while i < num_node:

        x = random.randint(0, grid_x)
        y = random.randint(0, grid_y)
        # let's make sure that the location is not repeated and it's not touching one of the robots' initial pose
        if (x, y) not in locations and [x, y, 0] not in robot_locations:
            locations.append((x,y))
            new_node = node_str + str(i)
            spanning_tree.add_node(new_node, location=[x, y])
            i += 1

    # Iterate through all the possible combinations of the nodes
    for node_i in location_graph.nodes:
        for node_j in location_graph.nodes:
            if node_i != node_j:

                # don't link the robots together (avoid collisions)
                if node_i[:4] == "/tb3" and node_j[:4] == "/tb3":
                    continue
                else:
                    # compute the distance between the two nodes
                    positions_i = spanning_tree.nodes[node_i]["location"]
                    positions_j = spanning_tree.nodes[node_j]["location"]
                    distance = math.sqrt((positions_i[0] - positions_j[0]) ** 2 + (positions_i[1] - positions_j[1]) ** 2)

                    # if nodes are close enough to make the references
                    if distance < threshold:
                        spanning_tree.add_edge(node_i, node_j, weight=distance)

    # Recursively call until the graph becomes connected
    if not nx.is_connected(spanning_tree):
        return construct_graph(location_graph, num_node, threshold, grid_x, grid_y)
    return spanning_tree


def allocate():
    """Allocate the robots' names to the goals."""
    global goal_names, location_graph, task_path_dict, task_allocation_dict

    task_allocation_dict = {}
    task_path_dict = {}

    # Greedy allocation: choose the most efficient robot in sequential order of the nodes
    if goal_names:
        # Create the tasks to allocate and the robot to allocate
        tasks_to_allocate = goal_names[:]
        robot_to_allocate = robot_names[:]
        while tasks_to_allocate:

            # Let the initialize min cost be infinities, keep track of
            # the minimum cost of the task, and the most efficient to doing that task
            new_task = tasks_to_allocate.pop()
            min_cost = float('inf')
            min_path = []
            most_eff_robot = ""
            for robot in robot_to_allocate:

                # add the constraints to the graph
                location_graph_with_constraints = copy.deepcopy(location_graph)
                for constraint in constraint_list:
                    if constraint[0] == robot and location_graph_with_constraints.has_node(constraint[0]):
                        location_graph_with_constraints.remove_node(constraint[1])

                # with the length of the cost
                length, path = nx.single_source_dijkstra(location_graph_with_constraints, robot, new_task)
                if length < min_cost:
                    min_cost = length
                    min_path = path
                    most_eff_robot = robot

            # assign the pairing and remove the robot
            task_allocation_dict[most_eff_robot] = new_task
            task_path_dict[most_eff_robot] = min_path
            robot_to_allocate.remove(most_eff_robot)


def calculate_timestamp():
    """Calculate when the robot will reach each node."""
    global task_path_dict, location_graph, time_stamp_dict

    time_stamp_dict = {}

    for robot in task_path_dict:

        print(robot)
        current_z = 0
        # find the time to travel the linear distance
        path_list = task_path_dict[robot]
        print(path_list)
        total_time = 0

        i = 0
        while i < len(path_list) - 1:
            # calculating the linear time
            lin_distance = location_graph.get_edge_data(path_list[i], path_list[i+1])["weight"]
            linear_time = lin_distance / robot_linear_vel

            # calculating the angular_time
            positions_i = location_graph.nodes[path_list[i]]["location"]
            positions_j = location_graph.nodes[path_list[i+1]]["location"]

            # theoretically, find the angular distance between the
            # two positions, subtract it from the current z orientation
            ang_distance = math.atan2(positions_j[1] - positions_i[1], positions_j[0] - positions_i[0])
            ang_distance = ang_distance - current_z
            current_z = ang_distance

            # the angular distance might be negative
            angular_time = abs(ang_distance / robot_angular_vel)

            # calculate set the time stamp the robot will be at each node
            total_time = total_time + linear_time + angular_time
            if robot not in time_stamp_dict:
                new_dict = {path_list[i+1]: total_time}
                time_stamp_dict[robot] = new_dict
            else:
                retrieve_dict = time_stamp_dict[robot]
                retrieve_dict[path_list[i+1]] = total_time
            i += 1
        print("----")


def add_constraint(constraint, shorter_robot):
    """Add a new constraint to the robot"""
    constraint_list.append([shorter_robot, constraint])


def find_node_conflict(robot1, robot2):
    """Find the number of  conflict between two robots as well as storing the constraint"""

    path1 = task_path_dict[robot1]
    path2 = task_path_dict[robot2]

    if len(path1) < len(path2):
        shorter_path = path1
        longer_path = path2
    else:
        shorter_path = path2
        longer_path = path1

    robot1_timestamp_dict = time_stamp_dict[robot1]
    robot2_timestamp_dict = time_stamp_dict[robot2]
    for node in shorter_path[1:]:
        # if the nodes both exist in the path, check if they're at the same time
        if node in longer_path:
            if abs(robot1_timestamp_dict[node] - robot2_timestamp_dict[node]) < 10:
                # find what robot is finishing earlier
                finish_time1 = max(time_stamp_dict[robot1].values())
                finish_time2 = max(time_stamp_dict[robot2].values())
                shorter_robot = robot1 if finish_time1 < finish_time2 else robot2

                # remove the first conflict and add constraint to the
                # the robot who's finishing earlier than the other robot
                path1 = task_path_dict[robot1]
                path2 = task_path_dict[robot2]
                conflict_nodes = list(set(path1) & set(path2))
                add_constraint(conflict_nodes.pop(0), shorter_robot)

                # 1 shows that there is at least one conflict, let's resolve it before anything else
                return 1

    # We went through the shorter path without finding any conflict
    return 0


def resolve_conflict():
    """Resolve conflict between random pairings of robots."""
    hashset = set(robot_names)
    constraint_number = 0

    while hashset:
        # Perform random pairing
        robot1 = hashset.pop()
        robot2 = hashset.pop()
        if find_node_conflict(robot1, robot2) > 0:
            constraint_number += 1

    print(constraint_number)
    return constraint_number


def perform_cbs():
    """A blackbox to perform conflict-based search"""
    rospy.loginfo("hello")
    spanning_tree = nx.Graph()

    robot_linear_vel = 0.2  # m/s
    robot_angular_vel = 0.2 # rad/s

    # ros parameters
    constraint_list = []
    robot_names = rospy.get_param("robot_names")
    goal_names = rospy.get_param("goal_names")
    robot_locations = rospy.get_param("robot_locations")
    goal_locations = rospy.get_param("goal_locations")

    # global parameters (instance variables)
    task_allocation_dict = {}
    task_path_dict = {}
    time_stamp_dict = {}

    # Let's create the base global location graph
    location_graph = include_robot(robot_locations, goal_locations)

    # location_graph = construct_graph(spanning_tree, location_graph, 15, 2, 10, 10)
    nx.draw(location_graph, with_labels=True)
    plt.savefig("/root/catkin_ws/src/location_graph.png")

    # # Allocate the robots to each goals and calculate the time stamp
    # allocate()
    # calculate_timestamp()
    # print(time_stamp_dict)

    # # If a conflict occurs
    # while resolve_conflict() > 0:

    #     print(time_stamp_dict)
    #     # Allocate but there will be conflicts next time, as well as the timestamp, so we can resolve future conflicts
    #     allocate()
    #     calculate_timestamp()

