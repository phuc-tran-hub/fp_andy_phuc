#!/usr/bin/env python

# Name: Phuc Tran and Andy Kotz
# Task: Machine Learning Implementation of Conflict Based Search

from operator import index
import rospy 
import numpy as np
import networkx as nx
import math
import json
import random

import matplotlib.pyplot as plt 

# GOAL: Create a connected graph of robots, goals, and possible location nodes
def include_robot(location_graph, robot_locations, goal_locations):
    """Include the robots' poses in the new tree."""
    # Initialize a location graph
    i = 0
    # For each of the robot, replace the node's label with the same position to the robot's name 
    while i < len(robot_locations):
        robot_node_i = "robot_" + str(i)
        x = robot_locations[i][0]
        y = robot_locations[i][1]
        pos = np.array([x, y])

        selected_data = dict( (n,d['pos']) for n,d in location_graph.nodes().items() if np.array_equal(d['pos'], pos) )
        keys_that_match = list(selected_data.keys())
        index_to_replace = keys_that_match[0]
        mapping = {index_to_replace: robot_node_i}
        location_graph = nx.relabel_nodes(location_graph, mapping)

        i += 1

    i = 0
    # For each of the goal, replace the node's label with the same position to the robot's name 
    while i < len(goal_locations):
        goal_node_i = "goal_" + str(i)
        x = goal_locations[i][0]
        y = goal_locations[i][1]
        pos = np.array([x, y])

        selected_data = dict( (n,d['pos']) for n,d in location_graph.nodes().items() if np.array_equal(d['pos'], pos))
        keys_that_match = list(selected_data.keys())
        index_to_replace = keys_that_match[0]
        mapping = {index_to_replace: goal_node_i}
        location_graph = nx.relabel_nodes(location_graph, mapping)
        i += 1
    
    return location_graph

def allocate(goal_names, robot_names, location_graph, constraint_list):
    """Allocate the robots' names to the goals."""
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
                location_graph_with_constraints = location_graph.copy()
                for constraint in constraint_list:
                    if constraint[0] == robot and location_graph_with_constraints.has_node(constraint[1]):
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
    
    rospy.loginfo(task_path_dict)
    return task_allocation_dict, task_path_dict

def reallocation(constraint_list, robot_names, location_graph, task_allocation_dict, task_path_dict):
    if constraint_list: 
        for robot in robot_names:
                location_graph_with_constraints = location_graph.copy()

                for constraint in constraint_list:
                    if constraint[0] == robot and location_graph_with_constraints.has_node(constraint[1]):
                        location_graph_with_constraints.remove_node(constraint[1])
                
                length, path = nx.single_source_dijkstra(location_graph_with_constraints, robot, task_allocation_dict[robot])
                task_path_dict[robot] = path
    
    return task_path_dict

def calculate_timestamp(task_path_dict, location_graph, robot_linear_vel, robot_angular_vel):
    """Calculate when the robot will reach each node."""

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
            # retrieving the two positions
            positions_i = location_graph.nodes[path_list[i]]["pos"]
            positions_j = location_graph.nodes[path_list[i+1]]["pos"]

            # calculating the linear time
            lin_distance = math.sqrt((positions_i[0] - positions_j[0]) ** 2 + (positions_i[1] - positions_j[1]) ** 2)
            linear_time = lin_distance / robot_linear_vel

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

    return time_stamp_dict


def add_constraint(constraint, shorter_robot, constraint_list):
    """Add a new constraint to the robot"""
    constraint_list.append([shorter_robot, constraint])
    return constraint_list

def find_node_conflict(robot1, robot2, task_path_dict, time_stamp_dict):
    """Find the number of  conflict between two robots as well as storing the constraint"""
    conflict_threshold = rospy.get_param("conflict_threshold")
    constraint_list = []
    
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
        # if the nodes both exist in the path, check if they're at the same time range
        if node in longer_path:
            if abs(robot1_timestamp_dict[node] - robot2_timestamp_dict[node]) < conflict_threshold:
                # find what robot is finishing earlier
                finish_time1 = max(time_stamp_dict[robot1].values())
                finish_time2 = max(time_stamp_dict[robot2].values())
                shorter_robot = robot1 if finish_time1 < finish_time2 else robot2

                # remove the first conflict and add constraint to the
                # the robot who's finishing earlier than the other robot
                path1 = task_path_dict[robot1]
                path2 = task_path_dict[robot2]
                conflict_nodes = list(set(path1) & set(path2))

                # Make sure you're not removing the goal node from the robot's path
                shorter_robot = robot1 if conflict_nodes[0] == path2[-1] else robot2
                constraint_list = add_constraint(conflict_nodes.pop(0), shorter_robot, constraint_list)

                # return the constraint list after adding onto it
                return constraint_list

    # We went through the shorter path without finding any conflict
    return constraint_list


def resolve_conflict(robot_names, task_path_dict, time_stamp_dict):
    """Resolve conflict between random pairings of robots."""
    hashset = set(robot_names)
    constraint_number = 0
    current_layer_constraints = [] 

    while hashset:
        # Perform random pairing
        robot1 = hashset.pop()
        robot2 = hashset.pop()
        constraint_list = find_node_conflict(robot1, robot2, task_path_dict, time_stamp_dict)
        current_layer_constraints += constraint_list
        if constraint_list:
            constraint_number += 1

    return current_layer_constraints, constraint_number


def convert_path_to_locations(location_graph, task_path_dict):
    """Converts each path to location coordinates."""

    # Iterate through each robot, find the node path lists and convert them to coordinates
    for robot in task_path_dict:
        robot_node_list = task_path_dict[robot]
        robot_location_list = []
        for node in robot_node_list:
            node_location = location_graph.nodes[node]["pos"]
            robot_location_list.append(node_location.tolist())
        
        # Reupdate the path dictionary
        task_path_dict[robot] = robot_location_list
    return task_path_dict

def perform_cbs(location_graph):
    """A blackbox to perform conflict-based search"""
    robot_linear_vel = rospy.get_param("robot_linear_vel") # m/s
    robot_angular_vel = rospy.get_param("robot_angular_vel") # rad/s

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

    # Include the robots in the graph
    location_graph = include_robot(location_graph, robot_locations, goal_locations)

    nx.draw(location_graph, with_labels=True)
    plt.savefig("location_graph.png")

    # Allocate the robots to each goals and calculate the time stamp of when each robot will reach the node
    # For the first allocate, there are no constraints
    task_allocation_dict, task_path_dict =  allocate(goal_names, robot_names, location_graph, [])
    time_stamp_dict = calculate_timestamp(task_path_dict, location_graph, robot_linear_vel, robot_angular_vel)

    # holds all the constraints
    large_constraint_list = [] 

    # If a conflict occurs, add that constraint to the constraint list, allocate again, and find the timestamp dictionary
    while True:
        constraint_number = 0
        
        # For the current layer of tree hiearchy, find the constraint list, the number of constraints
        constraint_list, constraint_number = resolve_conflict(robot_names, task_path_dict, time_stamp_dict)
        large_constraint_list += constraint_list

        rospy.loginfo(large_constraint_list)
        rospy.loginfo(constraint_number)
        # Let's reallocate to find the new paths
        if constraint_number > 0:            
            task_path_dict = reallocation(large_constraint_list, robot_names, location_graph, task_allocation_dict, task_path_dict)
            time_stamp_dict = calculate_timestamp(task_path_dict, location_graph, robot_linear_vel, robot_angular_vel)
        # Eventually, the constraint number will become zero with the given path and timestamps
        else:
            task_path_dict = convert_path_to_locations(location_graph, task_path_dict)
            break

    rospy.loginfo("finished")
    return task_allocation_dict, task_path_dict