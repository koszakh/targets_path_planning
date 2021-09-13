#!/usr/bin/env python

import rospy
from targets_path_planning.msg import AllPaths, Path
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

if flag:

	p1 = Point(-4, 0, 0)
	g1 = Point(4, 0, 0)
	g1_1 = p1.get_point_at_distance_and_angle(g1, p1.get_distance_to(g1) / 2)
	name1 = 'sim_p3at1'
	path1 = [p1, g1_1, g1]
	vect1 = p1.get_angle_between_points(g1)
	rot1 = pp.Rotation.from_euler('xyz', [0, 0, vect1], degrees=True)
	quat1 = rot1.as_quat()
	gc.spawn_target(name1, p1, quat1)
	robot1 = gc.Robot(name1)
	
	p2 = Point(0, -4, 0)
	g2 = Point(0, 4, 0)
	g2_1 = p2.get_point_at_distance_and_angle(g2, p2.get_distance_to(g2) / 2)
	name2 = 'sim_p3at2'
	path2 = [p2, g2_1, g2]
	vect2 = p2.get_angle_between_points(g2)
	rot2 = pp.Rotation.from_euler('xyz', [0, 0, vect2], degrees=True)
	quat2 = rot2.as_quat()
	gc.spawn_target(name2, p2, quat2)
	robot2 = gc.Robot(name2)
	
	orca = ORCAsolver(None, None, None, None, None, None)
	orca.add_agent(name1, path1)
	orca.add_agent(name2, path2)
	
	paths = orca.run_orca()
	
	robot1.waypoints_publisher(paths[robot1.name])
	robot2.waypoints_publisher(paths[robot2.name])
	
	robots = [robot1, robot2]
	cont_flag = False
	
	while not cont_flag:

		cont_flag = True
	
		for robot in robots:
		
			if not robot.path:
			
				cont_flag = False
				
	for robot in robots:
	
		robot.start()
	
	
else:

	offset = 5.5

	c1 = -12.5
	c2 = c1 + offset * 2
	c3 = c2 + offset * 2
	
	min_b = c1 - offset
	max_b = c1 + offset
	
	g_min_b = c2 - offset
	g_max_b = c2 + offset
	
	g1_min_b = c3 - offset
	g1_max_b = c3 + offset
	
	s1 = []
	s2 = []
	s3 = []

	px = random.uniform(min_b, max_b)
	py = random.uniform(min_b, max_b)
	gx = random.uniform(g_min_b, g_max_b)
	gy = random.uniform(g_min_b, g_max_b)
	g_1x = random.uniform(g1_min_b, g1_max_b)
	g_1y = random.uniform(g1_min_b, g1_max_b)
	p = Point(px, py, 0.2)
	s1.append(p)
	g = Point(gx, gy, 0)
	s2.append(g)
	g_1 = Point(g_1x, g_1y, 0)
	s3.append(g_1)
	#g_1 = p.get_point_at_distance_and_angle(g, p.get_distance_to(g) / 2)
	g_1.set_z(0)
	gc.spawn_sdf_model(g, gc_const.BLUE_VERTICE_PATH, 'g')
	gc.spawn_sdf_model(g_1, gc_const.BLUE_VERTICE_PATH, 'g_1')
	name = 'sim_p3at1'
	vect = p.get_angle_between_points(g)
	rot = pp.Rotation.from_euler('xyz', [0, 0, vect], degrees=True)
	quat = rot.as_quat()
	gc.spawn_target(name, p, quat)
	
	p1x = random.uniform(min_b, max_b)
	p1y = random.uniform(min_b, max_b)
	p1 = Point(p1x, p1y, 0.2)
	
	while get_min_dist(p1, s1) < const.UB_NEIGHBOR_DIST:
	
		p1x = random.uniform(min_b, max_b)
		p1y = random.uniform(min_b, max_b)
		p1 = Point(p1x, p1y, 0.2)
		
	s1.append(p1)
	
	g1x = random.uniform(g_min_b, g_max_b)
	g1y = random.uniform(g_min_b, g_max_b)
	g1 = Point(g1x, g1y, 0)
	
	while get_min_dist(g1, s2) < const.UB_NEIGHBOR_DIST:
	
		g1x = random.uniform(g_min_b, g_max_b)
		g1y = random.uniform(g_min_b, g_max_b)
		g1 = Point(g1x, g1y, 0)
	
	s2.append(g1)
		
	g1_1x = random.uniform(g1_min_b, g1_max_b)
	g1_1y = random.uniform(g1_min_b, g1_max_b)
	g1_1 = Point(g1_1x, g1_1y, 0)
	
	while get_min_dist(g1_1, s3) < const.UB_NEIGHBOR_DIST:
	
		g1_1x = random.uniform(g1_min_b, g1_max_b)
		g1_1y = random.uniform(g1_min_b, g1_max_b)
		g1_1 = Point(g1_1x, g1_1y, 0)
	
	s3.append(g1_1)
	
	#g1_1 = p1.get_point_at_distance_and_angle(g1, p1.get_distance_to(g1) / 2)
	g1_1.set_z(0)
	vect1 = p1.get_angle_between_points(g1)
	gc.spawn_sdf_model(g1, gc_const.GREEN_VERTICE_PATH, 'g1')
	gc.spawn_sdf_model(g1_1, gc_const.GREEN_VERTICE_PATH, 'g1_1')
	name1 = 'sim_p3at2'
	rot1 = pp.Rotation.from_euler('xyz', [0, 0, vect1], degrees=True)
	quat1 = rot1.as_quat()
	gc.spawn_target(name1, p1, quat1)
	
	p2x = random.uniform(min_b, max_b)
	p2y = random.uniform(min_b, max_b)
	p2 = Point(p2x, p2y, 0.2)
	
	while get_min_dist(p2, s1) < const.UB_NEIGHBOR_DIST:
	
		p2x = random.uniform(min_b, max_b)
		p2y = random.uniform(min_b, max_b)
		p2 = Point(p2x, p2y, 0.2)
	
	g2x = random.uniform(g_min_b, g_max_b)
	g2y = random.uniform(g_min_b, g_max_b)
	g2 = Point(g2x, g2y, 0)
	
	while get_min_dist(g2, s2) < const.UB_NEIGHBOR_DIST:
	
		g2x = random.uniform(g_min_b, g_max_b)
		g2y = random.uniform(g_min_b, g_max_b)
		g2 = Point(g2x, g2y, 0)
	
	g2_1x = random.uniform(g1_min_b, g1_max_b)
	g2_1y = random.uniform(g1_min_b, g1_max_b)
	g2_1 = Point(g2_1x, g2_1y, 0)
	
	while get_min_dist(g2_1, s3) < const.UB_NEIGHBOR_DIST:
	
		g2_1x = random.uniform(g1_min_b, g1_max_b)
		g2_1y = random.uniform(g1_min_b, g1_max_b)
		g2_1 = Point(g2_1x, g2_1y, 0)
		
	#g2_1 = p2.get_point_at_distance_and_angle(g2, p2.get_distance_to(g2) / 2)
	g2_1.set_z(0)
	gc.spawn_sdf_model(g2, gc_const.RED_VERTICE_PATH, 'g2')
	gc.spawn_sdf_model(g2_1, gc_const.RED_VERTICE_PATH, 'g2_1')
	name2 = 'sim_p3at3'
	vect2 = p2.get_angle_between_points(g2)
	rot2 = pp.Rotation.from_euler('xyz', [0, 0, vect2], degrees=True)
	quat2 = rot2.as_quat()
	gc.spawn_target(name2, p2, quat2)
	robot = gc.Robot(name)
	robot1 = gc.Robot(name1)
	robot2 = gc.Robot(name2)
	orca = ORCAsolver(None, None, None, None, None, None)
	orca.add_agent(robot.name, [robot.get_robot_position(), g, g_1])
	orca.add_agent(robot1.name, [robot1.get_robot_position(),g1, g1_1])
	orca.add_agent(robot2.name, [robot2.get_robot_position(), g2, g2_1])
	paths = orca.run_orca()
	robot.waypoints_publisher(paths[robot.name])
	robot1.waypoints_publisher(paths[robot1.name])
	robot2.waypoints_publisher(paths[robot2.name])
	robots = [robot, robot1, robot2]
	cont_flag = False
	
	while not cont_flag:

		cont_flag = True
	
		for robot in robots:
		
			if not robot.path:
			
				cont_flag = False
				
	for robot in robots:
	
		robot.start()
	
	#colors = copy.copy(gc_const.PATH_COLORS)

	#for key in paths.keys():

		#path = paths[key]
		#path = delete_intermediate_points(path, 12)
		#color = random.choice(colors)
		#colors.remove(color)
		#gc.visualise_path(path, color, str(key) + '_')
	
print('Finish!')
rospy.spin()

