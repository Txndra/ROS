#!/usr/bin/env python3

import rospy
import heapq
import math
from algorithms.neighbors import find_neighbors


def heuristic(n1, n2, width):
    """ Euclidean distance between two flat indices """
    x1, y1 = n1 % width, n1 // width
    x2, y2 = n2 % width, n2 // width
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)



def a_star(start, goal, width, height, costmap, resolution, origin, grid_visualisation):
    """
    Enhanced A* using:
      - weighted heuristic (epsilon)
      - tie-breaking on equal f-costs
    """

    epsilon = 1.5 

    # --------------------------------------------------------
    # Basic validation
    # --------------------------------------------------------
    if start < 0 or start >= len(costmap):
        rospy.logerr("Start node out of bounds.")
        return [], 0

    if goal < 0 or goal >= len(costmap):
        rospy.logerr("Goal node out of bounds.")
        return [], 0

    open_set = []                   # priority queue
    closed_set = set()              # visited nodes
    g_cost = {start: 0.0}           # cost so far
    parent = {start: None}          # reconstruct path

    nodes_expanded = 0

    # --------------------------------------------------------
    # Push start node into the queue (with weighted heuristic)
    # --------------------------------------------------------
    start_h = heuristic(start, goal, width)
    start_f = epsilon * start_h
    heapq.heappush(open_set, (start_f, start_h, start))

    max_iterations = 100000
    iteration = 0

    while open_set and iteration < max_iterations:
        iteration += 1
        f_cost, h_cost, current_node = heapq.heappop(open_set)
        
        if current_node in closed_set:
            continue

        closed_set.add(current_node)
        nodes_expanded += 1

        try:
            grid_visualisation.set_color(current_node, "pale yellow")
        except Exception:
            pass

        # ---------------------------
        # GOAL REACHED
        # ---------------------------
        if current_node == goal:
            path = []
            node = current_node
            while node is not None:
                path.append(node)
                node = parent[node]
            path.reverse()

            for p in path:
                try:
                    grid_visualisation.set_color(p, "green")
                except Exception:
                    pass

            return path, nodes_expanded

        # ---------------------------
        # EXPAND NEIGHBOURS
        # ---------------------------
        current_g = g_cost[current_node]
        neighbours = find_neighbors(current_node, width, height, costmap, orthogonal_step_cost=1.0)

        for next_node, move_cost in neighbours:

            if next_node in closed_set:
                continue

            new_g = current_g + move_cost

            if next_node not in g_cost or new_g < g_cost[next_node]:
                g_cost[next_node] = new_g
                parent[next_node] = current_node

                h = heuristic(next_node, goal, width)
                f = new_g + epsilon * h    # weighted A*

                # ---------------------------------------
                # TIE-BREAKING:
                # lower h_cost should win if f is equal
                # stored inside heap entry for comparison
                # ---------------------------------------
                heapq.heappush(open_set, (f, h, next_node))

                try:
                    grid_visualisation.set_color(next_node, "orange")
                except Exception:
                    pass

    rospy.logwarn("Enhanced A* failed to find a path after %d expansions.", nodes_expanded)
    return [], nodes_expanded
