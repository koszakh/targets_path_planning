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

def group_movement():

	root_path = gc_const.PATHS_DIR

	robot_names = ['sim_p3at' + str(i) for i in range(1, const.ROBOTS_COUNT + 1)]
	rootDir = pathlib.Path(root_path)
	local_dirs = [rootItem.stem + '/' for rootItem in rootDir.iterdir()]
	print(len(local_dirs))
	print(const.ROBOTS_COUNT / len(local_dirs))
	start_index = 10
	robots = {}
	
	for folder in local_dirs:
	
		current_dir = root_path + folder
		currentDirectory = pathlib.Path(current_dir)
		dir_items = [currentFile.stem for currentFile in currentDirectory.iterdir()]
		
		
		for i in range(const.ROBOTS_COUNT / len(local_dirs)):
		
			name = random.choice(robot_names)
			robot_names.remove(name)
			rnd_path = random.choice(dir_items)
			dir_items.remove(rnd_path)
			file_path = current_dir + rnd_path + ".txt"
			print(file_path)
			path = txt_path_parser(file_path)[start_index:]
			start_p = path[0]
			next_p = path[1]
			angle = start_p.get_dir_vector_between_points(next_p).vector_to_angle()
			rot = Rotation.from_euler('xyz', [0, 0, angle], degrees=True)
			quat = rot.as_quat()
			gc.spawn_target(name, start_p, quat)
			robot = gc.Robot(name)
			robot.waypoints_publisher(path)
			robots[name] = robot
			#print(path)
			
	for key in robots.keys():
	
		robot = robots[key]
		robot.start()
		

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

rospy.init_node('path_planner')
group_movement()
rospy.spin()
