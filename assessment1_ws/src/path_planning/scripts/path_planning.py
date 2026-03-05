#!/usr/bin/env python3

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from gridviz import GridViz
from a_star_algo import a_star

class AStarPathServer:
    def __init__(self):
        """Initializes the service node and communication handles."""
        rospy.init_node('path_planning_server', log_level=rospy.INFO, anonymous=False)
        rospy.on_shutdown(self.handle_shutdown)
      
        self.res = 0.05
        self.map_origin = [-3.898960, -3.985016, 0.000000]
        
        # Communication
        self.velocity_signal = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.service_provider = rospy.Service(
            "/move_base/SrvClientPlugin/make_plan",
            PathPlanningPlugin,
            self.process_path_request
        )
        
        rospy.loginfo("NAV SERVER: A* System Online.")

    def process_path_request(self, req):
        """Processes the incoming planning request and returns the A* solution."""
        
        costmap_data = req.costmap_ros
        cols = req.width
        rows = req.height
        start_node = req.start
        goal_node = req.goal

        visualizer = GridViz(costmap_data, self.res, self.map_origin, start_node, goal_node, cols)

        timer_start = rospy.Time.now()

        # Execute core algorithm
        final_route, total_nodes_checked = a_star(
            start_node, goal_node, cols, rows, costmap_data,
            self.res, self.map_origin, visualizer
        )

        elapsed_time = (rospy.Time.now() - timer_start).to_sec()
        response = PathPlanningPluginResponse()

        if not final_route:
            rospy.logerr(">>> SEARCH FAILURE: No valid path identified.")
            path_len = 0
            expanded = total_nodes_checked
        else:
            path_len = len(final_route)
            expanded = total_nodes_checked - path_len
            rospy.loginfo(">>> SEARCH SUCCESS: Plan generated.")
            response.plan = final_route
            
        self.log_performance(path_len, elapsed_time, expanded)

        return response

    def log_performance(self, length, time, expanded):
        """Redesigned performance output for assessment logging."""
        header = " [ A* ALGORITHM DATA ] "
        rospy.loginfo(header.center(40, "="))
        rospy.loginfo(f"| Outcome:      {'SUCCESS' if length > 0 else 'FAILED'}")
        rospy.loginfo(f"| Path Cost:    {length} units")
        rospy.loginfo(f"| Compute Time: {time:.4f} s")
        rospy.loginfo(f"| Search Scope: {expanded} nodes expanded")
        rospy.loginfo("=" * 40)

    def handle_shutdown(self):
        """Ensures the robot stops all movement upon node termination."""
        rospy.loginfo("Terminating Server...")
        self.velocity_signal.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        server = AStarPathServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
