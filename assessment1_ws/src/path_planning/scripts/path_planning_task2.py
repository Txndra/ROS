#!/usr/bin/env python3
import rospy
import heapq
import math
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist, PoseStamped
from gridviz import GridViz
from algorithms import neighbors

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

def a_star_smoothed(start_idx, goal_idx, width, height, costmap, resolution, origin, grid_visualisation):
    """The Optimised A* implementation with Turning Penalty."""

    # Priority Queue: (f_score, current_index, last_direction)
    pq = [(0, start_idx, (0,0))]
    g_costs = {start_idx: 0}
    parents = {start_idx: None}

    while pq:
        f, curr, last_dir = heapq.heappop(pq)
        grid_visualisation.set_color(curr, "pale yellow")

        if curr == goal_idx:
            return reconstruct_path(parents, curr, resolution, origin, width)

        for nb_info in neighbors.find_neighbors(curr, width, height, costmap, 1.0):
            nb_idx = nb_info[0]
            nb_step_cost = nb_info[1]
            
            # Calculate dx, dy for the vector
            curr_x, curr_y = curr % width, curr // width
            nb_x, nb_y = nb_idx % width, nb_idx // width
            dx, dy = nb_x - curr_x, nb_y - curr_y

            grid_visualisation.set_color(nb_idx, "orange")

            # --- NEW ANGLE LOGIC FOR OBJECTIVE 1 ---
            angle_penalty = 0
            if last_dir != (0,0):
                angle_old = math.atan2(last_dir[1], last_dir[0])
                angle_new = math.atan2(dy, dx)
                # Calculate smallest difference in radians
                diff = angle_new - angle_old
                theta = abs(math.atan2(math.sin(diff), math.cos(diff)))
                
                # Proportional penalty (Objective 1.1)
                angle_penalty = theta * 10.0 
                
                # Backtracking check (Objective 1.2)
                if last_dir == (-dx, -dy):
                    angle_penalty += 50.0

            # Combined cost
            new_g = g_costs[curr] + nb_step_cost + angle_penalty

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
    
    # These are already integers (indices), so we don't need .pose.position
    start_idx = req.start
    goal_idx = req.goal
    
    resolution = 0.05
    origin = [-3.898960, -3.985016, 0.0]

    # 2. Now pass them to GridViz
    # (Using the indices directly as provided by the request)
    grid_viz = GridViz(costmap, resolution, origin, start_idx, goal_idx, width)
    
    # 3. Call the algorithm
    # Pass the integers start_idx and goal_idx
    path = a_star_smoothed(start_idx, goal_idx, width, height, costmap, resolution, origin, grid_viz)   
    
    if path:
        rospy.loginfo("Task 2.1: Optimized path found!")
    
    resp = PathPlanningPluginResponse()
    resp.plan = path
    return resp

if __name__ == '__main__':
    rospy.init_node('path_planning_server_task2')
    rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
    rospy.spin()
