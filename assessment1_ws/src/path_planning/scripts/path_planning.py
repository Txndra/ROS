#!/usr/bin/env python3

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist, PoseStamped
from gridviz import GridViz
from a_star_algo import a_star
from path_smoothing import a_star_smoothed

def get_index(x, y, origin, resolution, width):
    """Convert (x, y) coordinates to grid index"""
    gx = int((x - origin[0]) / resolution)
    gy = int((y - origin[1]) / resolution)
    return (gy * width) + gx


def index_to_pose(index, width, resolution, origin):
    """Convert grid index to PoseStamped"""
    col = index % width
    row = index // width
    x = origin[0] + col * resolution
    y = origin[1] + row * resolution

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0
    pose.pose.orientation.w = 1.0
    return pose


def make_plan(req):
    costmap = req.costmap_ros
    width = req.width
    height = req.height
    resolution = 0.05
    origin = [-3.898960, -3.985016, 0.0]

    # Hardcoded waypoints
    waypoints_m = [
        [-3, -2.5],
        [-1.86, -2.5],
        [0.115, 0.161],
        [0.725, -0.472],
        [1.02, -0.888]
    ]

    # Convert waypoints to grid indices
    waypoint_indices = [get_index(x, y, origin, resolution, width) for x, y in waypoints_m]
    full_sequence = [req.start] + waypoint_indices

    global_full_path = []

    rospy.loginfo("==== Planning Full Path ====")

    for i in range(len(full_sequence) - 1):
        seg_start = full_sequence[i]
        seg_goal = full_sequence[i + 1]

        rospy.loginfo(f"Planning segment {i+1}: start index {seg_start}, goal index {seg_goal}")

        # Initialize visualizer
        grid_visualisation = GridViz(costmap, resolution, origin, seg_start, seg_goal, width)

        # Compute A* path
        compute_start = rospy.Time.now()
        path_segment, nodes_expanded = a_star_smoothed(
            seg_start, seg_goal, width, height, costmap,
            resolution, origin, grid_visualisation
        )
        compute_end = rospy.Time.now()
        computation_time = (compute_end - compute_start).to_sec()

        if not path_segment:
            rospy.logwarn(f"Failed to plan segment {i+1}")
            continue

        path_length = len(path_segment)

        rospy.loginfo(f"===== SEGMENT {i+1} METRICS =====")
        rospy.loginfo(f"Nodes Expanded: {nodes_expanded}")
        rospy.loginfo(f"Path Length: {path_length}")
        rospy.loginfo(f"Computation Time: {computation_time:.4f} seconds")

        # Merge segment into global path
        if not global_full_path:
            global_full_path.extend(path_segment)
        else:
            global_full_path.extend(path_segment[1:])

    rospy.loginfo("Full path planning complete. Sending path to move_base.")

    # Publish path to move_base
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.sleep(0.5)

    for node in global_full_path:
        pose = index_to_pose(node, width, resolution, origin)
        goal_pub.publish(pose)
        rospy.sleep(0.05)  # adjust speed of path following

    # Return the path indices
    resp = PathPlanningPluginResponse()
    resp.plan = global_full_path
    return resp


def clean_shutdown():
    cmd_vel.publish(Twist())
    rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('path_planning_server', log_level=rospy.INFO, anonymous=False)

    make_plan_service = rospy.Service(
        "/move_base/SrvClientPlugin/make_plan",
        PathPlanningPlugin,
        make_plan
    )

    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Path Planning Server Ready")
    rospy.spin()
