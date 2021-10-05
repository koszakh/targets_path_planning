#!/usr/bin/env python

# Target navigation node

import rospy
import rvo2
from targets_path_planning.msg import AllPaths, Vector2d
from path_planning.Point import Point
import gazebo_communicator.BatteryTracker as bt
import gazebo_communicator.GazeboConstants as const
import gazebo_communicator.GazeboCommunicator as gc
from gazebo_communicator.Robot import Robot
from time import sleep
import copy

def convert_to_paths(msg):

	paths = []
	
	for path_msg in msg.paths:
	
		path = convert_to_path(path_msg)
		paths.append(path)
		
	return paths

def convert_to_path(msg):

	path = []

	for state in msg:

		x = state.x
		y = state.y
		z = state.z
		p = Point(x, y, z)
		path.append(p)

	return path


def paths_callback(msg_data):
	
	workers = {}
	chargers = {}
	paths_list = msg_data.all_paths

	for paths_list in paths_list:
	
		name = paths_list.robot_name
		robot = Robot(name, "worker", b_trackers)
		w_points = convert_to_path(workpoints[name])
		path = convert_to_paths(paths[name])
		robot.workpoints_publisher(w_points)
		robot.waypoints_publisher(path)
		workers[name] = robot	
	
	cont_flag = False
	
	while not cont_flag:
	
		cont_flag = True
	
		for key in workers.keys():
	
			if not workers[key].paths:
			
				cont_flag = False

	print('Robot movement has begun!')
	for key in workers.keys():
	
		workers[key].start()

rospy.init_node('robot_navigator')
paths_sub = rospy.Subscriber('all_paths_data', AllPaths, paths_callback)
rospy.spin()
