#!/usr/bin/env python

# Path planning node for ground targets
import path_planning.PathPlanner as pp
import rospy
import copy
from targets_path_planning.msg import Point3d, AllPaths, AllRobotsPos, Path, RobotPos
from path_planning.Point import Point
from path_planning.Heightmap import Heightmap
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const

def callback(msg_data):
    path_publisher = rospy.Publisher('path_data', AllPaths, queue_size=10)
    paths_msg = AllPaths()
    paths_msg.path_list = []
    hm = Heightmap()
    hmap, height, width = hm.prepare_heightmap()
    map_handler = pp.PathPlanner(hmap, height, width)
    map_handler.gridmap_preparing()
    for msg in msg_data.pos_list:
        robot_name = msg.robot_name
        #robot_num = robot_name[len(robot_name) - 1]
        robot_pos = convert_to_point(msg.pos)
        start_id, goal_id = map_handler.get_start_and_goal_id(robot_pos)
        print('Path planning for ' + robot_name + ' has begun!')
        path, path_ids, path_cost = map_handler.find_path(start_id, goal_id)
        smoothed_path = map_handler.curve_smoothing(path_ids)
        if smoothed_path:
            msg = prepare_path_msg(smoothed_path, robot_name)
            paths_msg.path_list.append(msg)
    path_publisher.publish(paths_msg)

def convert_to_point(msg):
    x = msg.x
    y = msg.y
    z = msg.z
    point = Point(x, y, z)
    return point

def prepare_path_msg(path, robot_name):
    msg = Path()
    msg.robot_name = robot_name
    msg.path = []
    for state in path:
        point = Point3d()
        point.x = state.x
        point.y = state.y
        point.z = state.z
        msg.path.append(point)
    return msg

rospy.init_node('path_planner')
robot_pos_sub = rospy.Subscriber('robots_pos_data', AllRobotsPos, callback)
rospy.spin()
