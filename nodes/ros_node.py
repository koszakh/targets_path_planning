#!/usr/bin/env python

import rospy
from targets_path_planning.msg import Point3d, AllPaths, Path
from path_planning.Point import Point, Vector2d
from geometry_msgs.msg import Pose, Twist
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
import path_planning.PathPlanner as pp
import path_planning.Constants as const
from path_planning.ORCA import ORCAsolver
from path_planning.Heightmap import Heightmap
import copy
from math import sqrt, asin, fabs, pi
from time import sleep
from time import time
import random

     
def convert_to_path(msg):
    path = []
    for point_msg in msg.path:
        p = convert_to_point(point_msg)
        path.append(p)
    return path

def convert_to_point(msg):
    x = msg.x
    y = msg.y
    z = msg.z
    point = Point(x, y, z)
    return point

def make_pose_msg(state, orient):
    msg = Pose()
    msg.position.x = state.x
    msg.position.y = state.y
    msg.position.z = state.z
    if orient:
        msg.orientation.x = orient[0]
        msg.orientation.y = orient[1]
        msg.orientation.z = orient[2]
    return msg        

rospy.init_node('ros_node')
sleep(5)
robots = []
flag = False
if flag:
	hm = Heightmap()
	hmap, height, width, x_step, y_step, grid_step = hm.prepare_heightmap()
	map_handler = pp.PathPlanner(hmap, height, width, grid_step)
	map_handler.cell_maker()
	cells = map_handler.cells
	map_handler.gridmap_preparing()
	orca = ORCAsolver(hmap, cells, x_step, y_step)
	smoothed_paths = {}
	for name in gc_const.ROBOT_NAMES:
	    robot = gc.Robot(name)
	    robots.append(robot)
	    robot_pos = robot.get_robot_position()
	    start_id, goal_id = map_handler.get_start_and_goal_id(robot_pos)
	    path, path_ids, path_cost = map_handler.find_path(start_id, goal_id)
	    if path:
		smoothed_path = map_handler.curve_smoothing(path_ids)
	    if smoothed_path:
		orca.add_agent(name, smoothed_path)
		smoothed_paths[name] = smoothed_path
	    print('Current agents count: ' + str(orca.sim.getNumAgents()))
        if orca.sim.getNumAgents() < len(gc_const.ROBOT_NAMES):
            print('Path planning is impossible!')
        else:
            orca.run_orca()
	del orca
	orca = ORCAsolver(hmap, cells, x_step, y_step)
        if len(smoothed_paths) < len(gc_const.ROBOT_NAMES):
            print('Path planning is impossible!')
        else:
  	    for name in gc_const.ROBOT_NAMES:
	        orca.add_agent(name, smoothed_paths[name])
	        print('Current agents count: ' + str(orca.sim.getNumAgents()))
	    orca.run_orca_2d()
else:
        flag1 = False
        robot = gc.Robot('jeep1')
        if flag1:
            hm = Heightmap()
	    hmap, height, width, x_step, y_step, grid_step = hm.prepare_heightmap()
	    map_handler = pp.PathPlanner(hmap, height, width, grid_step)
	    map_handler.gridmap_preparing()
            robot_pos = robot.get_robot_position()
	    start_id, goal_id = map_handler.get_start_and_goal_id(robot_pos)
	    path, path_ids, path_cost = map_handler.find_path(start_id, goal_id)
	    if path:
	        smoothed_path = map_handler.curve_smoothing(path_ids)
                f = open('/root/catkin_ws/src/targets_path_planning/jeep_path.txt', 'w+')
                f.write('path = [')
                f.close()
                for state in smoothed_path:
                    f = open('/root/catkin_ws/src/targets_path_planning/jeep_path.txt', 'a+')
                    f.write('Point(' + str(state.x) + ', ' + str(state.y) + ', ' + str(state.z) + '), ')
                    f.close()
                f = open('/root/catkin_ws/src/targets_path_planning/jeep_path.txt', 'a+')
                f.write(']')
                f.close()
	    if smoothed_path:
                gc.visualise_path(smoothed_path, gc_const.RED_VERTICE_PATH, 'v')
                robot.set_final_path(smoothed_path)
                robot.start()
        else:
            path = [Point(-8.5, -3.5, 3.21568627451), Point(-8.5, -4.0, 3.13725490196), Point(-8.5, -4.5, 3.13725490196), Point(-8.5, -5.0, 3.21568627451), Point(-8.5, -5.5, 3.37254901961), Point(-8.40404401145, -5.90404401145, 3.37254901961), Point(-8.35, -6.0, 3.37254901961), Point(-8.16666666667, -6.33333333333, 3.42483660131), Point(-8.125, -6.5, 3.45098039216), Point(-8.1, -6.6, 3.46666666667), Point(-8.0, -7.0, 3.52941176471)]
            gc.visualise_path(path, gc_const.RED_VERTICE_PATH, 'v')
            robot.set_final_path(path)
            robot.start()
        #robot1 = gc.Robot('p3at1')
	#robot2 = gc.Robot('p3at2')
	#robot3 = gc.Robot('p3at3')
        #robot1.set_final_path()
        #robot2.set_final_path()
        #robot3.set_final_path()
        #robot1.start()
        #robot2.start()
        #robot3.start()

print('Finish!')
rospy.spin()

