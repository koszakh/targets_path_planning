#!/usr/bin/env python

# Target navigation node

import rospy
import rvo2
from targets_path_planning.msg import AllPaths, AllRobotsPos, RobotPos, Vector2d
from path_planning.Point import Point
import gazebo_communicator.GazeboConstants as const
import gazebo_communicator.GazeboCommunicator as gc
from time import sleep
import copy

def convert_to_path(msg):
    path = []
    for state in msg:
        x = state.x
        y = state.y
        z = state.z
        p = Point(x, y, z)
        path.append(p)
    return path

def deleting_intermediate_points(path, cut_step):
    path_copy = copy.copy(path)
    for i in range(len(path) - 1):
        if (i % cut_step > 0):
            if path[i] in path_copy:
                path_copy.remove(path[i])
    return path_copy

def paths_callback(msg_data):
    print('Paths are received')
    paths_list = msg_data.path_list
    robots = []
    for path_msg in paths_list:
        final_path = convert_to_path(path_msg.path)	
        name = path_msg.robot_name
        robot = gc.Robot(path_msg.robot_name)
        robot.waypoints_publisher(final_path)
        path = deleting_intermediate_points(final_path, 10)
        gc.visualise_path(path, const.PATH_COLORS[len(robots)], name + '_v_')
        robots.append(robot)	
    sleep(1)
    print('Robot movement has begun!')
    for robot in robots:
        robot.start()

rospy.init_node('robot_navigator')
paths_sub = rospy.Subscriber('all_paths_data', AllPaths, paths_callback)
rospy.spin()
