#!/usr/bin/env python3

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from gridviz import GridViz

def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a msg of type PathPlanningPluginResponse
  ''' 
  costmap = req.costmap_ros
  width = req.width
  height = req.height
  start = req.start
  goal = req.goal
  
  # side of each grid map square in meters
  resolution = 0.05
  
  # origin of grid map - FIXED from your YAML
  origin = [-10.0, -10.0, 0.0] 

  # Initialize visualization
  grid_visualisation = GridViz(costmap, resolution, origin, start, goal, width)

  # time statistics
  start_time = rospy.Time.now()

  # -------------------------------------------------------------------------
  # Placeholder for A* (Task 2.1)
  # For now, we define an empty path so the service doesn't crash.
  path = [] 
  
  # Once you write your a_star function, you will call it like this:
  # path = a_star(start, goal, width, height, costmap, resolution, origin, grid_visualisation)
  # -------------------------------------------------------------------------

  if not path:
    rospy.logwarn("No path returned by the path algorithm")
    path = []
  else:
    rospy.loginfo('Path sent to navigation stack')

  resp = PathPlanningPluginResponse()
  resp.plan = path
  return resp
