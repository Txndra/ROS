#!/usr/bin/env python3

import rospy
import heapq
import math
from algorithms.neighbors import find_neighbors


# -------------------------------------------------------------
# Assumed Turtlebot3 velocities
# These are used to estimate time cost for turning and moving
# -------------------------------------------------------------
MAX_LINEAR_VEL = 0.22   # m/s
MAX_ANGULAR_VEL = 2.84  # rad/s


# -------------------------------------------------------------
# Heuristic function
# Uses Euclidean distance between two nodes in the grid
# -------------------------------------------------------------
def heuristic(n1, n2, width):
    x1, y1 = n1 % width, n1 // width
    x2, y2 = n2 % width, n2 // width
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


# -------------------------------------------------------------
# Turning penalty
# Calculates how sharp the turn is between 3 nodes
# prev_node -> current_node -> next_node
# The sharper the angle, the larger the penalty
# -------------------------------------------------------------
def turning_penalty(prev_node, current_node, next_node, width):

    x1, y1 = prev_node % width, prev_node // width
    x2, y2 = current_node % width, current_node // width
    x3, y3 = next_node % width, next_node // width

    # Direction vectors
    v1 = (x2 - x1, y2 - y1)
    v2 = (x3 - x2, y3 - y2)

    # Dot product
    dot = v1[0]*v2[0] + v1[1]*v2[1]

    # Vector magnitudes
    mag1 = math.sqrt(v1[0]**2 + v1[1]**2)
    mag2 = math.sqrt(v2[0]**2 + v2[1]**2)

    if mag1 == 0 or mag2 == 0:
        return 0

    # Compute angle between vectors
    cos_theta = dot / (mag1 * mag2)
    cos_theta = max(-1, min(1, cos_theta))  # clamp
    angle = math.acos(cos_theta)

    return angle


# -------------------------------------------------------------
# Travel time penalty
# Approximates how long the robot would take to move
# and rotate between nodes
# -------------------------------------------------------------
def travel_time_penalty(distance, angle):

    time_translation = distance / MAX_LINEAR_VEL
    time_rotation = abs(angle) / MAX_ANGULAR_VEL

    return time_translation + time_rotation


# -------------------------------------------------------------
# Enhanced A* with:
# - weighted heuristic
# - turning penalty
# - travel time cost
# - path smoothing
# -------------------------------------------------------------
def a_star_smoothed(start, goal, width, height, costmap, resolution, origin, grid_visualisation):

    epsilon = 1.5   # weight for heuristic

    # Basic bounds checking
    if start < 0 or start >= len(costmap):
        rospy.logerr("Start node out of bounds")
        return [], 0

    if goal < 0 or goal >= len(costmap):
        rospy.logerr("Goal node out of bounds")
        return [], 0

    open_set = []
    closed_set = set()

    g_cost = {start: 0}
    parent = {start: None}

    nodes_expanded = 0

    # Push start node
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

        # Visualise node expansion
        try:
            grid_visualisation.set_color(current_node, "pale yellow")
        except Exception:
            pass

        # -------------------------------------------------
        # Goal reached
        # -------------------------------------------------
        if current_node == goal:

            path = []
            node = current_node

            while node is not None:
                path.append(node)
                node = parent[node]

            path.reverse()

            # Colour final path
            for p in path:
                try:
                    grid_visualisation.set_color(p, "green")
                except Exception:
                    pass

            # Smooth the path
            smoothed_path = smooth_path(path, costmap, width, height)

            return smoothed_path, nodes_expanded

        # -------------------------------------------------
        # Expand neighbours
        # -------------------------------------------------
        current_g = g_cost[current_node]

        neighbours = find_neighbors(
            current_node,
            width,
            height,
            costmap,
            orthogonal_step_cost=1.0
        )

        for next_node, move_cost in neighbours:

            if next_node in closed_set:
                continue

            # Calculates turning cost
            if parent[current_node] is not None:
                angle = turning_penalty(parent[current_node], current_node, next_node, width)
            else:
                angle = 0

            # Compute travel-time penalty for distance and angle
            distance = heuristic(current_node, next_node, width)

            time_pen = travel_time_penalty(distance, angle)

            # Total cost includes movement, turning, and travel-time penalties
            new_g = current_g + move_cost + angle + time_pen

            if next_node not in g_cost or new_g < g_cost[next_node]:

                g_cost[next_node] = new_g
                parent[next_node] = current_node

                h = heuristic(next_node, goal, width)
                f = new_g + epsilon * h

                heapq.heappush(open_set, (f, h, next_node))

                try:
                    grid_visualisation.set_color(next_node, "orange")
                except Exception:
                    pass

    rospy.logwarn("Enhanced A* failed to find path after %d expansions", nodes_expanded)

    return [], nodes_expanded


# -------------------------------------------------------------
# Path smoothing using line-of-sight
# Removes unnecessary zig-zag nodes
# -------------------------------------------------------------
def smooth_path(path, costmap, width, height):

    if len(path) <= 2:
        return path

    smoothed = [path[0]]

    i = 0

    while i < len(path) - 1:

        j = len(path) - 1

        # Try connecting current node directly to a far node
        while j > i + 1:

            if is_line_of_sight(path[i], path[j], costmap, width, height):
                break

            j -= 1

        smoothed.append(path[j])
        i = j

    return smoothed


# -------------------------------------------------------------
# Line-of-sight check using Bresenham's algorithm
# Ensures no obstacles exist between two nodes
# -------------------------------------------------------------
def is_line_of_sight(start, end, costmap, width, height):

    x0, y0 = start % width, start // width
    x1, y1 = end % width, end // width

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1

    err = dx - dy

    while True:

        index = y0 * width + x0

        # Check bounds
        if index < 0 or index >= len(costmap):
            return False

        # Check obstacle (100+ is lethal in ROS costmaps)
        if costmap[index] >= 100:
            return False

        if x0 == x1 and y0 == y1:
            break

        e2 = 2 * err

        if e2 > -dy:
            err -= dy
            x0 += sx

        if e2 < dx:
            err += dx
            y0 += sy

    return True