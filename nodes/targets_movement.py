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
from scipy.spatial.transform import Rotation
from time import sleep

def group_movement(paths, targets_count):

	robots = []
	target_names = ['sim_p3at' + str(i) for i in range(1, targets_count + 1)]
	
	if len(target_names) < len(paths):
	
		for name in target_names:

			path = random.choice(paths)
			paths.remove(path)
			short_path = delete_intermediate_points(path, 30)
			gc.visualise_path(short_path, random.choice(gc_const.PATH_COLORS), 'v_')
			robot = spawn_target(name, path)
			robots.append(robot)
			
		cont_flag = False
		
		while not cont_flag:
	
			cont_flag = True
		
			for robot in robots:
		
				if not robot.path:
				
					cont_flag = False
			
		for robot in robots:

			robot.start()
	else:
	
		print('The number of targets is more than the number of paths!')

def get_paths_from_dir(folder_path):

	current_dir = folder_path
	currentDirectory = pathlib.Path(current_dir)
	dir_items = [currentFile.stem for currentFile in currentDirectory.iterdir()]
	paths = []
	
	for item in dir_items:

		dir_items.remove(item)
		file_path = current_dir + item + ".txt"
		path = txt_path_parser(file_path)
		
		if len(path) > 0:

			paths.append(path)
		
	return paths
			
def spawn_target(name, path):

	start_p = path[0]
	next_p = path[1]
	angle = start_p.get_dir_vector_between_points(next_p).vector_to_angle()
	rot = Rotation.from_euler('xyz', [0, 0, angle], degrees=True)
	quat = rot.as_quat()
	gc.spawn_target(name, start_p, quat)
	robot = gc.Robot(name)
	robot.waypoints_publisher(path)
	return robot
		

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

	#print('File path: ' + str(file_path))
	#print('Type: ' + str(type(file_path)))
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

def delete_intermediate_points(path, cut_step):

	path_copy = copy.copy(path)

	for i in range(len(path) - 1):

		if (i % cut_step > 0):

			if path[i] in path_copy:

				path_copy.remove(path[i])

	return path_copy

rospy.init_node('path_planner')

targets_count = 1

root_path = gc_const.PATHS_DIR
rootDir = pathlib.Path(root_path)
local_dirs = ['paths6_local/']#[rootItem.stem + '/' for rootItem in rootDir.iterdir()]
all_paths = []

for folder in local_dirs:

	folder_path = root_path + folder
	paths = get_paths_from_dir(folder_path)
	all_paths.extend(paths)
	
print('All paths count: ' + str(len(all_paths)))
print('Targets count: ' + str(targets_count))

group_movement(all_paths, targets_count)
rospy.spin()
