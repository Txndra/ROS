#!/usr/bin/env python3
import rospy
import heapq
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist, PoseStamped
from gridviz import GridViz
import neighbor # Uses your neighbor.py helper

# --- TASK 2.1 HELPER FUNCTIONS ---

def calculate_heuristic(index, goal_index, width):
    """Calculates Euclidean distance between two grid indices."""
    curr_x, curr_y = index % width, index // width
    goal_x, goal_y = goal_index % width, goal_index // width
    return ((goal_x - curr_x)**2 + (goal_y - curr_y)**2)**0.5

def reconstruct_path(parents, curr_idx, resolution, origin, width):
    """Traces back from goal to start and converts to Meters."""
    path = []
    while curr_idx is not None:
        ix, iy = curr_idx % width, curr_idx // width
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x = (ix * resolution) + origin[0]
        p.pose.position.y = (iy * resolution) + origin[1]
        p.pose.orientation.w = 1.0 
        path.append(p)
        curr_idx = parents[curr_idx]
    return path[::-1]

def a_star_smoothed(start, goal, width, height, costmap, resolution, origin, grid_visualisation):
    """The Optimized A* implementation with Turning Penalty."""
    # Convert Start/Goal meters to Grid Indices
    s_x = int((start.pose.position.x - origin[0]) / resolution)
    s_y = int((start.pose.position.y - origin[1]) / resolution)
    start_idx = (s_y * width) + s_x

    g_x = int((goal.pose.position.x - origin[0]) / resolution)
    g_y = int((goal.pose.position.y - origin[1]) / resolution)
    goal_idx = (g_y * width) + g_x

    # Priority Queue: (f_score, current_index, last_direction)
    pq = [(0, start_idx, (0,0))]
    g_costs = {start_idx: 0}
    parents = {start_idx: None}

    while pq:
        f, curr, last_dir = heapq.heappop(pq)

        # (i) Visualise node being traversed
        grid_visualisation.set_color(curr, "pale yellow")

        if curr == goal_idx:
            return reconstruct_path(parents, curr, resolution, origin, width)

        # (ii) Expand neighbors using neighbor.py
        for nb_idx, dx, dy in neighbor.get_neighbors(curr, width, height, costmap):
            grid_visualisation.set_color(nb_idx, "orange")

            # Calculate cost + turning penalty (smoothing)
            move_cost = 1.414 if (dx != 0 and dy != 0) else 1.0
            penalty = 0
            if last_dir != (0,0) and (dx, dy) != last_dir:
                penalty = 15.0 # The "Smoothing" factor

            new_g = g_costs[curr] + move_cost + penalty

            if nb_idx not in g_costs or new_g < g_costs[nb_idx]:
                g_costs[nb_idx] = new_g
                h = calculate_heuristic(nb_idx, goal_idx, width)
                heapq.heappush(pq, (new_g + h, nb_idx, (dx, dy)))
                parents[nb_idx] = curr
    return []

# --- ROS SERVICE CALLBACK ---

def make_plan(req):
    # Map metadata from the request
    costmap, width, height = req.costmap_ros, req.width, req.height
    start, goal = req.start, req.goal
    resolution, origin = 0.05, [-10.0, -10.0, 0.0]

    # Initialize visualization
    grid_viz = GridViz(costmap, resolution, origin, start, goal, width)

    # Run the Task 2.1 algorithm
    path = a_star_smoothed(start, goal, width, height, costmap, resolution, origin, grid_viz)

    if path:
        rospy.loginfo("Task 2.1: Optimized path found!")
    
    resp = PathPlanningPluginResponse()
    resp.plan = path
    return resp

if __name__ == '__main__':
    rospy.init_node('path_planning_server_task2')
    rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
    rospy.spin()