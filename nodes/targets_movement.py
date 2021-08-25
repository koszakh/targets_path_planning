#!/usr/bin/env python

# Path planning node for ground targets
import path_planning.PathPlanner as pp
import rospy
import copy
import numpy
from targets_path_planning.msg import AllPaths, Path
from geometry_msgs.msg import Point
from path_planning.Point import Point as PointGeom, Vector2d
from path_planning.Heightmap import Heightmap
import path_planning.Constants as const
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
from path_planning.ORCA import ORCAsolver
import random
import pathlib
import io

def group_movement():

	robot_names = ['sim_p3at' + str(i) for i in range(1, const.ROBOTS_COUNT + 1)]
	init_dir = gc_const.PATHS_DIR
	start_index = 10
	
	for path_dir in gc_const.LOCAL_PATH_DIRS:
	
		current_dir = init_dir + path_dir
		currentDirectory = pathlib.Path(path)
		dir_items = [currentFile.stem for currentFile in currentDirectory.iterdir()]
		
		for i in range(const.ROBOTS_COUNT / len(gc_const.LOCAL_PATH_DIRS)):
		
			name = random.choice(robot_names)
			robot_names.remove(name)
			rnd_path = random.choice(dir_items)
			dir_items.remove(rnd_path)
			file_path = current_dir + rnd_path + '.txt'
			path = txt_path_parser(file_path)
			start_p = path[0]
			next_p = path[1]
			angle = start_p.get_dir_vector_between_points(next_p).vector_to_angle()
			gc.spawn_target(name, start_p, (0, 0, angle, 0))
			#print(path)
				
		

def prepare_paths_msg(names, paths):

	msg = AllPaths()
	msg.path_list = []
	for name in names:
	
		path = paths[name]
		path_msg = prepare_path_msg(name, path)
		msg.path_list.append(path_msg)
	
	return msg

def prepare_path_msg(name, path):

	msg = Path()
	msg.path = []
	msg.robot_name = name
	
	for state in path:
	
		point = Point()
		point.x = state.x
		point.y = state.y
		point.z = state.z
		msg.path.append(point)
	
	return msg

def convert_to_vector(msg_data):

	x = msg_data.x
	y = msg_data.y
	point = Vector2d(x, y)
	return point

def convert_to_point(msg_data):

	x = msg_data.x
	y = msg_data.y
	z = msg_data.z
	point = PointGeom(x, y, z)
	return point
	
def txt_path_parser(file_path):

	path = []
	
	with io.open(file_path, encoding='utf-8') as file:
			
		for line in file:
		
			point = line[line.find('(') + 1:line.rfind(')')]
			coords = point.split(', ')
			x = float(coords[0])
			y = float(coords[1])
			z = float(coords[2])
			p = PointGeom(x, y, z)
			path.append(p)
			
	return path

rospy.init_node('path_planner')
file_path = '/root/catkin_ws/src/targets_path_planning/test_path.txt'

path = txt_path_parser(file_path)
gc.spawn_target('sim_p3at1', path[0], (0, 0, 0, 0))
gc.visualise_path(path, gc_const.GREEN_VERTICE_PATH, 'v')
rospy.spin()
