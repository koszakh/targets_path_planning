#!/usr/bin/env python

# Paths getter node

import rospy
import rvo2
from targets_path_planning.msg import AllPaths, Poses, NamesList
from path_planning.Point import Point, Vector2d
import gazebo_communicator.BatteryTracker as bt
import gazebo_communicator.GazeboConstants as const
import gazebo_communicator.GazeboCommunicator as gc
from gazebo_communicator.Robot import Robot
from time import sleep
import copy

def convert_to_paths(msg):

	paths = []
	
	for path_msg in msg:
	
		path = convert_to_path(path_msg.path)
		paths.append(path)
		
	return paths

def convert_to_path(msg):

	path = []

	for state in msg:

		p = convert_to_point(state)
		path.append(p)

	return path
	
def convert_to_point(msg):

	x = msg.x
	y = msg.y
	z = msg.z
	p = Point(x, y, z)
	return p
	
def convert_to_vect(msg):

	x = msg.x
	y = msg.y
	vect = Vector2d(x, y)
	return vect


def paths_callback(msg_data):
	
	print('PATHS RECEIVED!')
	workers = {}
	chargers = {}
	paths_list = msg_data.all_paths
	names = [path_list.robot_name for path_list in paths_list]
	print(names)
	b_trackers = bt.get_battery_trackers(names)

	for path_list in paths_list:

		name = path_list.robot_name
		robot = Robot(name, "worker", b_trackers)
		w_points = convert_to_path(path_list.workpoints.path)
		paths = convert_to_paths(path_list.paths)
		robot.workpoints_publisher(w_points)
		robot.waypoints_publisher(paths)
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

	flag = False

	for charger in chargers:

		if charger.point_achiveability(ch_point):

			flag = True

def c_names_callback(msg_data):

	print('CHARGERS NAMES RECEIVED: ' + str(msg_data.names))

rospy.init_node('paths_getter')
paths_sub = rospy.Subscriber('worker_paths', AllPaths, paths_callback)
c_names_sub = rospy.Subscriber('chargers_names', NamesList, c_names_callback)
rospy.spin()
