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

def group_path_planning():

	paths_pub = rospy.Publisher('all_paths_data', AllPaths, queue_size=10)
	hm = Heightmap(const.HEIGHTMAP_SDF_PATH)

	min_col = const.COL_RANGE[0]
	max_col = const.COL_RANGE[1]
	min_row = const.ROW_RANGE[0]
	max_row = const.ROW_RANGE[1]

	hmap, l_scale, w_scale, x_step, y_step, step_count = \
		hm.prepare_heightmap(min_col, max_col, min_row, max_row)
		
	mh = pp.PathPlanner(hmap, l_scale, w_scale, x_step, y_step, step_count)

	min_x = mh.min_x
	max_x = mh.max_x
	min_y = mh.min_y
	max_y = mh.max_y
	
	avg_x = numpy.mean([min_x, max_x])
	avg_y = numpy.mean([min_y, max_y])
	
	new_x = mh.min_x#numpy.mean([min_x, avg_x])# + 7
	new_y = mh.min_y#numpy.mean([min_y, avg_y])# + 4
	new_x1 = mh.max_x#numpy.mean([avg_x, max_x])# - 15
	new_y1 = mh.max_y#numpy.mean([avg_y, max_y])# - 23
	
	offset = const.DIST_OFFSET

	cell_id = mh.get_current_cell_id(Point(-13238, 485, 0))
	print(cell_id)

	start = (new_x, new_y)
	goal = (new_x1, new_y1)

	print('start: ' + str(start))
	print('goal: ' + str(goal))
	
	mh.gridmap_preparing()
	cells = mh.cells
		
	orca = ORCAsolver(mh.heightmap, cells, x_step, y_step, l_scale, w_scale)
	smoothed_paths = {}
	robot_names = ['sim_p3at' + str(i) for i in range(1, const.ROBOTS_COUNT + 1)]
		
	for name in (robot_names):
	
		robot_pos, orient = mh.get_start_pos(start[0], start[1], offset)
		
		if robot_pos:
		
			gc.spawn_target(name, robot_pos, orient)
			
			robot_orient = gc.get_robot_orientation_vector(name)
			start_id, goal_id, start_pos = mh.get_start_and_goal_id(robot_pos, robot_orient, goal[0], goal[1], offset)
			#goal_id = True
			
			if goal_id:
			
				print('\nPath planning for ' + name + ' has begun!')
				path, path_ids, path_cost = mh.find_path(start_id, goal_id, robot_orient)
				
				#goal_v = mh.heightmap[goal_id]
				#path = [start_pos, goal_v]
					
				if path:

					orca.add_agent(name, path)
					
				else:
				
					orca.add_agent(name, [])
					
			print('Current ORCA agents count: ' + str(orca.sim.getNumAgents()))
	
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
