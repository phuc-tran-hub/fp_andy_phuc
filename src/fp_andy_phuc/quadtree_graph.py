#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.

# Import of python modules.
import networkx as nx
from PIL import Image
import numpy as np


def image_to_graph(image_name="/root/catkin_ws/src/fp_andy_phuc/nodes/map.png", space_size=64, quadtree_depth=5):

    map = np.array(Image.open(image_name, mode="r"))
    map = np.fliplr(np.transpose(map[:, :, 0]/255))

    area = {"left": 0, "right": space_size,
            "bottom": 0, "top": space_size}

    graphs = []
    for i in range(quadtree_depth):
        graph, squares = generate_graph(area, map, quadtree_depth)
        graphs.append(graph)

    for graph in graphs:
        squares_string = ""
        edges_string = ""
        sep = 0.05
        for j in list(graph.nodes(data=True)):
            i = j[1]["square"]
            squares_string += "\n"+str(i["left"]+sep) + "<x<"+str(i["right"]-sep) + \
                "\left\{"+str(i["bottom"]+sep)+"<y<" + \
                str(i["top"]-sep)+"\\right\}"
        for i in list(graph.edges(data=True)):
            sq1 = squares[i[0]]
            sq2 = squares[i[1]]
            center1 = np.array([float(sq1["left"]+sq1["right"]),
                                float(sq1["bottom"]+sq1["top"])])/2
            center2 = np.array([float(sq2["left"]+sq2["right"]),
                                float(sq2["bottom"]+sq2["top"])])/2
            edges_string += "\n"+"\left("+str(center1[0])+"+\left("+str(center2[0])+"-"+str(center1[0])+"\\right)t," + \
                str(center1[1])+"+\left("+str(center2[1]) + \
                "-"+str(center1[1])+"\\right)t\\right)"

    return graph, squares_string, edges_string


def generate_graph(area, map, iterations):
    """Generate a graph from a map."""

    # Generate a graph
    graph = nx.Graph()

    # Generate a list of free squares
    free_squares = subdivide(area, iterations, map, map)

    # Add nodes to the graph
    for i in range(len(free_squares)):
        sq = free_squares[i]
        center = np.array([float(sq["left"]+sq["right"]),
                           float(sq["bottom"]+sq["top"])])/2
        graph.add_node(
            i, square=free_squares[i], pos=center)

    # Add edges to the graph
    for i in range(len(free_squares)):
        for j in range(i+1, len(free_squares)):
            if is_adjacent(free_squares[i], free_squares[j]):
                graph.add_edge(i, j)

    return graph, free_squares


def is_adjacent(square1, square2):
    if square1["left"] == square2["right"] or square1["right"] == square2["left"]:
        if square1["top"] > square2["bottom"] and square1["bottom"] < square2["top"]:
            return True
    elif square1["top"] == square2["bottom"] or square1["bottom"] == square2["top"]:
        if square1["right"] > square2["left"] and square1["left"] < square2["right"]:
            return True


def subdivide(square, iterations, map, sub_map):

    # If the square does not have any obstacles, return it
    if np.all(sub_map) and iterations == 0:
        return [square]
    elif iterations > 0:
        squares = []
        for i in range(2):
            for j in range(2):
                left = square["left"] + i * \
                    (square["right"] - square["left"]) / 2
                right = square["left"] + \
                    (i + 1) * (square["right"] - square["left"]) / 2
                bottom = square["bottom"] + j * \
                    (square["top"] - square["bottom"]) / 2
                top = square["bottom"] + \
                    (j + 1) * (square["top"] - square["bottom"]) / 2

                sub_map = map[int(left):int(right), int(bottom):int(top)]

                new_squares = subdivide(
                    {"left": left, "right": right, "bottom": bottom, "top": top}, iterations - 1, map, sub_map)
                squares.extend(new_squares)
        return squares
    else:
        return []


if __name__ == "__main__":
    """Run the main function."""
    print(image_to_graph())
