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
flag = False
if flag:
	
	p1 = Point(5, 0, 0)
	ch_p1 = Point(0, 0, 0)
	g1 = Point(-5, 0, 0)
	path1 = [p1, g1]
	path1_1 = [g1, p1]
	
	p2 = Point(5, 5, 0)
	pre_ch_p = Point(3, 2, 0)
	pre_ch_p2 = (pre_ch_p, ch_p1, name1)
	path2_ch = [p2, pre_ch_p]
	path2_b = [ch_p1, p2]
	
	vect1 = p1.get_angle_between_points(g1)
	rot1 = Rotation.from_euler('xyz', [0, 0, vect1], degrees=True)
	quat1 = rot1.as_quat()

	vect2 = p2.get_angle_between_points(pre_ch_p)
	rot2 = Rotation.from_euler('xyz', [0, 0, vect2], degrees=True)
	quat2 = rot2.as_quat()

	gc.spawn_target(name1, p1, quat1)
	gc.spawn_target(name2, p2, quat2)
	
	w_paths = {}
	w_points = {}
	ch_points = {}
	w_paths[name1] = [path1, path1_1]
	w_points[name1] = [g1]
	ch_points[name1] = [ch_p1]
	
	pre_ch_points = {}
	to_ch_paths = {}
	to_base_paths = {}
	pre_ch_points[name2] = [pre_ch_p2]
	to_ch_paths[name2] = [path2_ch]
	to_base_paths[name2] = [path2_b]
	
	mm = MovementManager(None, [name1], [name2])
	mm.prepare_robots(w_paths, w_points, ch_points, pre_ch_points, to_ch_paths, to_base_paths)
	mm.start()
	
else:


	name = 'sim_p3at1'
	name1 = 'sim_p3at2'
	d_path = "/home/admin/catkin_ws/src/targets_path_planning/urdf/pioneer3at_cam.urdf"
	d_path1 = "/home/admin/catkin_ws/src/targets_path_planning/urdf/pioneer3at_aruco.urdf"
	p = Point(0, 0, 0)
	p1 = Point(1, 0, 0)	
	gc.spawn_urdf_model(name, d_path, p, (0, 0, 0, 0))
	gc.spawn_urdf_model(name1, d_path1, p1, (0, 0, 0, 0))
	
	robot1 = Robot(name)
	robot2 = Robot(name1)
	robot1.movement(0.2, 0)
	sleep(2)


print('Finish!')
rospy.spin()

