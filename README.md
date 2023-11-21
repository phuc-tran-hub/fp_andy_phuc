# Conflict-Based Search and Environment Quadtree for Multi-Robot Deliverers

## Demo 
[Conflict Based Search](https://drive.google.com/file/d/1oJL_hS6Rw3mR_GHLfQ_WqM_AYvCPYSzY/view?resourcekey)

## The Problem 
Our target problem was the increasing use of delivery robots in your average neighborhoods. As multi-robot systems become increasingly popular, cloud traffic will become increasingly difficult. We want to make sure that each robot has an efficient algorithm that would allow them to avoid possible collisions. 

## Approach
Andy's task was to create a graph using a quad-tree subdivision algorithm that could easily be queried for all robots. My task was to utilize the quad-tree graph to perform a conflict-based search for our multi-robot system. We would attempt to solve the conflict at the highest level possible, moving to the next level if necessary.

## Implementation 
### Conflict-Based Search
In our early implementation, we found the path with the lowest cost. If there is a conflict with another robot, we remove that node from the robot's path and continue to find another path. NOTE: this meant that the graph had to be dense enough to stay connected. For future implementations, I would have liked to make a more efficient system to provide a time interval where that node isn't available. I would also have liked to implement a continuous path and estimated time prediction publisher.

### Quadtree Graph 
We decided on a list of graphs for each level of the quadtree. Each node contains a reference to its parents and its four children. In the future, we need to complete integration with conflict-based search for a hierarchal approach to pathfinding. 
