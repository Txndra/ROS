#!/usr/bin/env python3
"""
a_star_algo.py
Standard A* algorithm using Euclidean distance heuristic.
Used as the baseline/benchmark algorithm for performance comparison.
"""

import heapq
import math
from geometry_msgs.msg import PoseStamped
from algorithms import neighbors


def calculate_heuristic(index, goal_index, width):
    """Euclidean distance heuristic between two grid indices."""
    curr_x = index % width
    curr_y = index // width
    goal_x = goal_index % width
    goal_y = goal_index // width
    return math.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)


def reconstruct_path(parents, curr_idx, resolution, origin, width):
    """Traces back from goal to start and converts grid indices to metric poses."""
    path = []
    while curr_idx is not None:
        ix = curr_idx % width
        iy = curr_idx // width
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x = (ix * resolution) + origin[0]
        p.pose.position.y = (iy * resolution) + origin[1]
        p.pose.orientation.w = 1.0
        path.append(p)
        curr_idx = parents[curr_idx]
    return path[::-1]


def a_star(start_idx, goal_idx, width, height, costmap, resolution, origin, grid_visualisation):
    """
    Standard A* algorithm with Euclidean distance heuristic.
    Returns (path, total_nodes_checked) where path is a list of PoseStamped.
    """

    # Priority queue: (f_score, index)
    pq = [(0, start_idx)]
    g_costs = {start_idx: 0}
    parents = {start_idx: None}
    nodes_checked = 0

    while pq:
        f, curr = heapq.heappop(pq)
        nodes_checked += 1

        # Visualise traversed node in pale yellow
        grid_visualisation.set_color(curr, "pale yellow")

        if curr == goal_idx:
            path = reconstruct_path(parents, curr, resolution, origin, width)
            return path, nodes_checked

        for nb_info in neighbors.find_neighbors(curr, width, height, costmap, 1.0):
            nb_idx = nb_info[0]
            nb_step_cost = nb_info[1]

            # Visualise neighbour node in orange
            grid_visualisation.set_color(nb_idx, "orange")

            new_g = g_costs[curr] + nb_step_cost

            if nb_idx not in g_costs or new_g < g_costs[nb_idx]:
                g_costs[nb_idx] = new_g
                h = calculate_heuristic(nb_idx, goal_idx, width)
                heapq.heappush(pq, (new_g + h, nb_idx))
                parents[nb_idx] = curr

    # No path found
    return [], nodes_checked
