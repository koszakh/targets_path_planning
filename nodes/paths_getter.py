#!/usr/bin/env python

# Paths getter node

import rospy
import rvo2
from targets_path_planning.msg import AllPaths, Poses
from path_planning.Point import Point, Vector2d
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

		p = convert_to_point(state)
		path.append(p)

	return path
	
def convert_to_point(msg):

	x = msg.x
	y = msg.y
	z = msg.z
	p = Point(x, y, z)
	return p
	
def convert_to_point(msg):

	x = msg.x
	y = msg.y
	z = msg.z
	vect = Vector2d(x, y)
	return vect


def paths_callback(msg_data):
	
	print('PATHS RECEIVED!')

rospy.init_node('paths_getter')
paths_sub = rospy.Subscriber('worker_paths', AllPaths, paths_callback)
rospy.spin()
