#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rvo2
from targets_path_planning.msg import AllPaths, Vector2d, Path
from geometry_msgs.msg import Point
from path_planning.Heightmap import Heightmap
from charge.charge import eval_charge_points
import path_planning.Constants as const
import gazebo_communicator.GazeboConstants as gc_const
import gazebo_communicator.GazeboCommunicator as gc
from time import sleep
import copy
from charge.utils import convert_to_path, prepare_paths_msg, prepare_path_msg, convert_to_point
from charge.PathListener import PathListener


def find_workers_and_chargers(robot_names, all_topics):
	w_names = []
	c_names = []
	for name in robot_names:
		topic_to_find = '/' + name + '/waypoints_array'
		if topic_to_find in all_topics:
			w_names.append(name)
		else:
			c_names.append(name)
	return w_names, c_names


def init_path_listeners(w_names):
	workers = dict()
	for name in w_names:
		workers[name] = PathListener(name)
	return workers


def charge_alloc(charging_pts, charging_robot_names, hmap):
	# Queue of names
	ch_robot_names = charging_robot_names
	# Data preparation
	allocation = {}
	for name in ch_robot_names:
		allocation[name] = []

	nums_of_ch_points = [len(points) for points in charging_pts.values()]

	i = 0
	# While i less than number of every list of charging points
	while i < max(nums_of_ch_points):
		for work_robot_name in charging_pts.keys():
			# If i less than number of list of charging points
			if i < len(charging_pts[work_robot_name]):
				ch_robot_name = ch_robot_names[0]
				allocation[ch_robot_name].append(charging_pts[work_robot_name][i])
				tmp_name = ch_robot_names.pop(0)
				ch_robot_names.append(tmp_name)
		i += 1

	return allocation


def define_charging_points(workers_data):
	ch_pts = dict()
	for w_name in workers_data.keys():
		ch_pts[w_name] = []

	for w_name in workers_data.keys():
		energy_resource = 1
		paths_msg = workers_data[w_name].paths.paths
		workpoints_msg = workers_data[w_name].workpoints.path
		workpoints = [convert_to_point(i) for i in workpoints_msg]
		for path_msg in paths_msg:
			path = convert_to_path(path_msg)
			ch_pts, energy_resource = eval_charge_points(path, workpoints, energy_resource)
			
			for ch_pt in ch_pts:
				ch_p[w_name].append(ch_pt)

	return ch_p


if __name__ == "__main__":
	rospy.sleep(15)  # Отладочные 15 сек для ожидания появления топиков с траекториями и рабочими точками
	rospy.init_node('energy_check')

	hm = Heightmap(const.HEIGHTMAP_SDF_PATH)
	hmap, l_scale, w_scale, x_step, y_step, step_count = hm.prepare_heightmap()
	robot_names = ['sim_p3at' + str(i) for i in range(1, gc_const.WORKERS_COUNT + gc_const.CHARGERS_COUNT + 1)]
	all_topics = [i[0] for i in rospy.get_published_topics()]

	w_names, c_names = find_workers_and_chargers(robot_names, all_topics)
	workers_data = init_path_listeners(w_names)

	charging_points = define_charging_points(workers_data)
	robot_allocation = charge_alloc(charging_points, c_names, hmap)

	allocation_pub = rospy.Publisher('allocation', AllPaths, queue_size=10)
	allocation_msg = prepare_paths_msg(c_names, allocation)
	allocation_pub.publish(allocation_msg)

	charging_points_pub = rospy.Publisher('charging_points', AllPaths, queue_size=10)
	charging_points_msg = prepare_paths_msg(w_names, charging_points)
	charging_points_pub.publish(charging_points_msg)

	rospy.spin()
