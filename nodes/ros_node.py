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
flag = False

if flag:

	p1 = Point(0, 0, 0)
	p2 = Point(4, 1, 0)
	gc.visualise_path([p1, p2], gc_const.GREEN_VERTICE_PATH, 'v')
	name = 'sim_p3at1'
	angle = p1.get_dir_vector_between_points(p2).vector_to_angle()
	rot = Rotation.from_euler('xyz', [0, 0, angle], degrees=True)
	quat = rot.as_quat()
	gc.spawn_target(name, p1, quat)
	robot = gc.Robot(name)
	
else:

	p = Point(30, 0, 0.2)
	g = Point(-30, 0, 0)
	g_1 = p.get_point_in_direction(p.get_dir_vector_between_points(g), 6)
	g_1.set_z(0)
	#gc.spawn_sdf_model(g, gc_const.BLUE_VERTICE_PATH, 'g')
	#gc.spawn_sdf_model(g_1, gc_const.BLUE_VERTICE_PATH, 'g_1')
	name = 'p3at1'
	vect = p.get_angle_between_points(g)
	rot = pp.Rotation.from_euler('xyz', [0, 0, vect], degrees=True)
	quat = rot.as_quat()
	#gc.spawn_target(name, p, quat)
	p1 = Point(-5, -1.7, 0.2)
	g1 = Point(1, 1.7, 0)
	g1_1 = p1.get_point_at_distance_and_angle(g1, p1.get_distance_to(g1) / 2)
	g1_1.set_z(0)
	vect1 = p1.get_angle_between_points(g1)
	gc.spawn_sdf_model(g1, gc_const.GREEN_VERTICE_PATH, 'g1')
	gc.spawn_sdf_model(g1_1, gc_const.GREEN_VERTICE_PATH, 'g1_1')
	name1 = 'p3at2'
	rot1 = pp.Rotation.from_euler('xyz', [0, 0, vect1], degrees=True)
	quat1 = rot1.as_quat()
	gc.spawn_target(name1, p1, quat1)
	p2 = Point(-4.5, 1.7, 0)
	g2 = Point(1, -1.7, 0)
	g2_1 = p2.get_point_at_distance_and_angle(g2, p2.get_distance_to(g2) / 2)
	g2_1.set_z(0)
	gc.spawn_sdf_model(g2, gc_const.RED_VERTICE_PATH, 'g2')
	gc.spawn_sdf_model(g2_1, gc_const.RED_VERTICE_PATH, 'g2_1')
	name2 = 'p3at3'
	vect2 = p2.get_angle_between_points(g2)
	rot2 = pp.Rotation.from_euler('xyz', [0, 0, vect2], degrees=True)
	quat2 = rot2.as_quat()
	gc.spawn_target(name2, p2, quat2)
	robot = gc.Robot(name)
	robot1 = gc.Robot(name1)
	robot2 = gc.Robot(name2)
	orca = ORCAsolver(None, None, None, None, None, None)
	#orca.add_agent(robot.name, [robot.get_robot_position(), g])
	orca.add_agent(robot1.name, [robot1.get_robot_position(),g1_1, g1])
	orca.add_agent(robot2.name, [robot2.get_robot_position(), g2_1, g2])
	paths = orca.run_orca()
	#robot.path = paths[robot.name]
	robot1.path = paths[robot1.name]
	robot2.path = paths[robot2.name]

	for key in paths.keys():

		path = paths[key]
		path = delete_intermediate_points(path, 6)
		
		gc.visualise_path(path, random.choice(gc_const.PATH_COLORS), str(key) + '_')
	
	robot.start()
	robot1.start()
	robot2.start()
	
print('Finish!')
rospy.spin()

