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
		hm.prepare_heightmap()
		
	mh = pp.PathPlanner(hmap, l_scale, w_scale, x_step, y_step, step_count)

	offset = const.DIST_OFFSET

	min_x = mh.min_x
	max_x = mh.max_x
	min_y = mh.min_y
	max_y = mh.max_y
	
	avg_x = numpy.mean([min_x, max_x])
	avg_y = numpy.mean([min_y, max_y])
	
	s_x = max_x - offset * 4.5
	s_y = min_y + offset * 1.5#avg_y - 30
	
	g1_x = max_x - offset * 1.5
	g1_y = avg_y#max_y - offset * 1.5
	
	g2_x = min_x + offset * 1.5
	g2_y = max_y - offset * 1.5
	
	g3_x = min_x + offset * 1.5
	g3_y = min_y + offset * 1.5
	
	g4_x = avg_x + offset * 1.5
	g4_y = min_y + offset * 1.5

	p1 = PointGeom(s_x, s_y, 0)
	p2 = PointGeom(g1_x, g1_y, 0)
	p3 = PointGeom(g2_x, g2_y, 0)
	p4 = PointGeom(g3_x, g3_y, 0)
	p5 = PointGeom(g4_x, g4_y, 0)
	
	start = (s_x, s_y)
	g1 = (g1_x, g1_y)
	g2 = (g2_x, g2_y)
	g3 = (g3_x, g3_y)
	g4 = (g4_x, g4_y)
	
	goal_regions = [g1]#, g2, g3, g4]
	
	areas_dist = p1.get_distance_to(p2) + p2.get_distance_to(p3) + p3.get_distance_to(p4) + p4.get_distance_to(p5)

	testing = False
	dynamic = True
	
	if testing:

		if dynamic:
	
			f = open(gc_const.MAP_DYNAMIC_COORDS_PATH, 'w+')
			f.close()
			
		else:

			f = open(gc_const.MAP_STATIC_COORDS_PATH, 'w+')
			f.close()
		
	print('Distance between areas centers: ' + str(areas_dist))
	print('Spawn area size: [' + str(offset * 2) + ', ' + str(offset * 2) + '] (m)')
	
	mh.gridmap_preparing()
	cells = mh.cells
		
	orca = ORCAsolver(mh.heightmap, cells, x_step, y_step, l_scale, w_scale)
	smoothed_paths = {}
	robot_names = ['sim_p3at' + str(i) for i in range(1, const.ROBOTS_COUNT + 1)]
		
	for name in (robot_names):
	
		robot_pos, orient = mh.get_start_pos(start[0], start[1], offset)
		
		if robot_pos:
		
			print('\nPath planning for ' + name + ' has begun!')
			gc.spawn_target(name, robot_pos, orient)
			
			last_vect = gc.get_robot_orientation_vector(name)
			whole_path = []
			
			for g in goal_regions:
			
				path = mh.get_path(robot_pos, last_vect, g[0], g[1], offset)
				
				if path:

					whole_path.extend(path)
					goal = path[len(path) - 1]
					last_goal = path[len(path) - 2]
					robot_pos = goal
					last_vect = last_goal.get_dir_vector_between_points(goal)

				else:
				
					print('PATH PLANNING FOR ' + name + ' IS IMPOSSIBLE IN ' + str(g) + ' REGION!')
					break
				
			#goal_id = True
			
			if whole_path:

				print('Path for ' + name + ' was found!')
				orca.add_agent(name, whole_path)
					
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
