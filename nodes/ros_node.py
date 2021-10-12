#!/usr/bin/env python

import rospy
from targets_path_planning.msg import Path
from path_planning.Point import Point, Vector2d
from geometry_msgs.msg import Pose, Twist
import gazebo_communicator.GazeboCommunicator as gc
from gazebo_communicator.Robot import Robot
import gazebo_communicator.BatteryTracker as bt
import gazebo_communicator.GazeboConstants as gc_const
import path_planning.PathPlanner as pp
import path_planning.Constants as const
from path_planning.MovementManager import MovementManager
from path_planning.Heightmap import Heightmap
import copy
from math import sqrt, asin, fabs, pi
from time import sleep
from time import time
import random
from scipy.spatial.transform import Rotation

def get_min_dist(p, points):

	min_dist = float('inf')
	
	for n_p in points:
	
		dist = p.get_distance_to(n_p)
		
		if dist < min_dist:
		
			min_dist = dist
			
	return min_dist

def delete_intermediate_points(path, cut_step):

	path_copy = copy.copy(path)

	for i in range(len(path) - 1):

		if (i % cut_step > 0):

			if path[i] in path_copy:

				path_copy.remove(path[i])

	return path_copy
	 
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
sleep(1)
flag = True

name1 = 'sim_p3at1'
name2 = 'sim_p3at2'
names = [name1, name2]
p1 = Point(5, 0, 0)
p2 = Point(0, 0, 0)
gc.spawn_target(name1, p1, (0, 0, 0, 0))
gc.spawn_target(name2, p2, (0, 0, 0, 0))


g1 = Point(-5, 0, 0)
g2 = Point(0, 0, 0)
paths = {}
workpoints = {}
w_ch_points = {}
paths[name1] = [[p1, g1]]
paths[name2] = [[p2, g2]]
workpoints[name1] = [g1]
workpoints[name2] = [g2]
w_ch_points[name1] = [g1]
w_ch_points[name2] = [g2]

mm = MovementManager(None, names, [])
mm.prepare_robots(paths, workpoints, w_ch_points, {}, {}, {})
mm.start()
print('Finish!')
rospy.spin()

