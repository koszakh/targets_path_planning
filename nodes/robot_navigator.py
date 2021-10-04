#!/usr/bin/env python

# Target navigation node

import rospy
import rvo2
from targets_path_planning.msg import Paths, Vector2d
from path_planning.Point import Point
import gazebo_communicator.BatteryTracker as bt
import gazebo_communicator.GazeboConstants as const
import gazebo_communicator.GazeboCommunicator as gc
from gazebo_communicator.Robot import Robot
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

def paths_callback(msg_data):
	
	print('Paths received.')
	paths_list = msg_data.paths
	robots = []
	names = [path.robot_name for path in paths_list]
	trackers = bt.get_battery_trackers(names)

	for path_msg in paths_list:
	
		final_path = convert_to_path(path_msg.path)	
		name = path_msg.robot_name
		robot = Robot(path_msg.robot_name, "worker", trackers)
		robot.waypoints_publisher([final_path])
		robots.append(robot)	
	
	cont_flag = False
	
	while not cont_flag:
	
		cont_flag = True
	
		for robot in robots:
	
			if not robot.path:
			
				cont_flag = False

	print('Robot movement has begun!')
	for robot in robots:
	
		robot.start()

rospy.init_node('robot_navigator')
paths_sub = rospy.Subscriber('all_paths_data', Paths, paths_callback)
rospy.spin()
