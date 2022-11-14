#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.

# Import of python modules.
import networkx as nx
from PIL import Image
import numpy as np
import rospy


def image_to_graph(image_name="/root/catkin_ws/src/fp_andy_phuc/nodes/map.png", space_size=64, quadtree_depth=6):

    map = np.array(Image.open(image_name, mode="r"))
    map = np.fliplr(np.transpose(map[:, :, 0]/255))

    area = {"left": 0, "right": space_size,
            "bottom": 0, "top": space_size, "parent": None, "depth": quadtree_depth+1, "index": 0}

    index = [-1]
    graph, nodes = generate_graph(area, map, quadtree_depth, index)

    squares_string = ""
    edges_string = ""
    sep = 0.05
    for j in list(graph.nodes(data=True)):
        i = j[1]["square"]
        squares_string += "\n"+str(i["left"]+sep) + "<x<"+str(i["right"]-sep) + \
            "\left\{"+str(i["bottom"]+sep)+"<y<" + \
            str(i["top"]-sep)+"\\right\}"
    for i in list(graph.edges(data=True)):
        sq1 = nodes[i[0]]
        sq2 = nodes[i[1]]
        center1 = np.array([float(sq1["left"]+sq1["right"]),
                            float(sq1["bottom"]+sq1["top"])])/2
        center2 = np.array([float(sq2["left"]+sq2["right"]),
                            float(sq2["bottom"]+sq2["top"])])/2
        edges_string += "\n"+"\left("+str(center1[0])+"+\left("+str(center2[0])+"-"+str(center1[0])+"\\right)t," + \
            str(center1[1])+"+\left("+str(center2[1]) + \
            "-"+str(center1[1])+"\\right)t\\right)"

    return graph, squares_string, edges_string


def generate_graph(area, map, iterations, index):
    """Generate a graph from a map."""

    # Generate a graph
    graphs = []
    for i in range(iterations+1):
        graphs.append(nx.Graph())

    # Generate a list of free squares
    free_squares, top_square = subdivide(area, iterations, map, map, index)
    free_squares = sorted(free_squares, key=lambda d: d["index"])
    nodes = {}

    for i in free_squares:
        nodes[i["index"]] = i

    # Add nodes to the graph
    for sq in free_squares:
        center = np.array([float(sq["left"]+sq["right"]),
                           float(sq["bottom"]+sq["top"])])/2
        graphs[sq["depth"]].add_node(
            sq["index"], square=sq, pos=center)

    # Add edges to the graph
    lowest_squares = [i for i in free_squares if i["depth"] == 0]
    for i in range(len(lowest_squares)):
        for j in range(i+1, len(lowest_squares)):
            if is_adjacent(lowest_squares[i], lowest_squares[j]):
                square1 = lowest_squares[i]
                square2 = lowest_squares[j]
                while square1 is not None and square2 is not None and square1["index"] != square2["index"] and not graphs[square1["depth"]].has_edge(square1["index"], square2["index"]):
                    graphs[square1["depth"]].add_edge(
                        square1["index"], square2["index"])
                    if square1["parent"] is not None and square2["parent"] is not None:
                        square1 = nodes[square1["parent"]]
                        square2 = nodes[square2["parent"]]
                    else:
                        square1 = None
                        square2 = None

    return graphs[0], nodes


def is_adjacent(square1, square2):
    if square1["depth"] != square2["depth"]:
        return False
    if square1["left"] == square2["right"] or square1["right"] == square2["left"]:
        if square1["top"] > square2["bottom"] and square1["bottom"] < square2["top"]:
            return True
    elif square1["top"] == square2["bottom"] or square1["bottom"] == square2["top"]:
        if square1["right"] > square2["left"] and square1["left"] < square2["right"]:
            return True


def subdivide(square, iterations, map, sub_map, index):

    index[0] += 1
    this_index = index[0]

    # If the square does not have any obstacles, return it
    if iterations == 0 and np.all(sub_map):
        this_square = square
        this_square.update(
            {"children": [], "index": this_index, "depth": iterations})
        return [this_square], this_square
    elif iterations > 0 and np.any(sub_map):
        all_squares = []
        children = []
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

                all_new_squares, new_square = subdivide(
                    {"left": left, "right": right, "bottom": bottom, "top": top, "parent": this_index}, iterations - 1, map, sub_map, index)
                if new_square:
                    all_squares.extend(all_new_squares)
                    children.append(new_square["index"])
        if len(children) > 0:
            this_square = square
            this_square.update(
                {"children": children, "index": this_index, "depth": iterations})
            all_squares.append(this_square)
            return all_squares, this_square
        else:
            return None, None
    else:
        return None, None


if __name__ == "__main__":
    """Run the main function."""
    print(image_to_graph())
