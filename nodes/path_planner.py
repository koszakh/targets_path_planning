#!/usr/bin/env python

# Path planning node for ground targets
import path_planning.PathPlanner as pp
import rospy
import copy
from targets_path_planning.msg import AllPaths, Path, AllRobotsPos
from geometry_msgs.msg import Point
from path_planning.Point import Point as PointGeom, Vector2d
from path_planning.Heightmap import Heightmap
import path_planning.Constants as const
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
from path_planning.ORCA import ORCAsolver
import random

def group_path_planning():
	paths_pub = rospy.Publisher('all_paths_data', AllPaths, queue_size=10)
	hm = Heightmap(const.HEIGHTMAP_SDF_PATH)

	hmap, l_scale, w_scale, x_step, y_step, grid_step, step_count = \
		hm.prepare_heightmap(const.MIN_COL, const.MAX_COL, const.MIN_ROW, const.MAX_ROW)
		
	map_handler = pp.PathPlanner(hmap, l_scale, w_scale, grid_step, x_step, y_step, step_count)

	min_x = map_handler.min_x
	max_x = map_handler.max_x
	min_y = map_handler.min_y
	max_y = map_handler.max_y
	
	new_x = random.uniform(min_x, max_x)
	new_y = random.uniform(min_y, max_y)
	new_x1 = random.uniform(min_x, max_x)
	new_y1 = random.uniform(min_y, max_y)
	
	offset = const.DIST_OFFSET

	start = (new_x, new_y)
	goal = (new_x1, new_y1)
	
	map_handler.gridmap_preparing()
	cells = map_handler.cells
	
	#cell_id = random.choice(list(map_handler.cells.keys()))#(str(float(map_handler.min_col)), str(float(map_handler.min_row)))
		
	orca = ORCAsolver(map_handler.heightmap, cells, x_step, y_step, l_scale, w_scale)
	smoothed_paths = {}
	robot_names = []
	
	for name in gc_const.ROBOT_NAMES:
	
		robot_names.append(name)
		robot_pos, orient = map_handler.get_start_pos(start[0], start[1], offset)
		gc.spawn_target(name, robot_pos, orient)
		robot_orient = gc.get_robot_orientation_vector(name)
		start_id, goal_id = map_handler.get_start_and_goal_id(robot_pos, robot_orient, goal[0], goal[1], offset)
		
		if goal_id:
		
			print('\nPath planning for ' + name + ' has begun!')
			path, path_ids, path_cost = map_handler.find_path(start_id, goal_id, robot_orient)
			
			if path:

				orca.add_agent(name, path)
				
		print('Current agents count: ' + str(orca.sim.getNumAgents()))
	
	paths = orca.run_orca()
	msg = prepare_paths_msg(paths.keys(), paths)
	paths_pub.publish(msg)

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

rospy.init_node('path_planner')
group_path_planning()
rospy.spin()
