#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rvo2
from targets_path_planning.msg import AllPaths, Vector2d, Path, NamesList
from geometry_msgs.msg import Point
from path_planning.Heightmap import Heightmap
from charge.charge import eval_charge_points
import path_planning.Constants as const
import gazebo_communicator.GazeboConstants as gc_const
import gazebo_communicator.GazeboCommunicator as gc
from time import sleep
import copy
from charge.utils import convert_to_path, prepare_paths_msg, prepare_path_msg, convert_to_point
from charge.MsgListener import MsgListener


def convert_to_strings(c_names_msg):
	names = c_names_msg.names
	c_names = []
	for name in names:
		c_names.append(name)
	return c_names


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


def c_names_callback(msg):
	c_names = msg_data.names
	return c_names

def parse_worker_paths_msg(workers_msg):
	all_paths = workers_msg.all_paths
	w_names = []
	workers_data = dict()
	workers_workpoints = dict()
	for paths in all_paths:
		w_name = paths.robot_name
		w_names.append(w_name)
		workers_data[w_name] = []
		workers_workpoints[w_name] = []
	for paths in all_paths:
		w_name = paths.robot_name
		for item in paths.paths:
			path = item.path
			path = convert_to_path(path)
			workers_data[w_name].append(path)
		workpoints = paths.workpoints.path
		workpoints = convert_to_path(workpoints)
		workers_workpoints[w_name].append(workpoints)

	return w_names, workers_data, workers_workpoints


def charge_alloc(charging_pts, charging_robot_names, hmap):
	# Queue of names
	print(charging_robot_names)
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

def define_charging_points_with_parse(workers_data):
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
				ch_pts[w_name].append(ch_pt)

	return ch_pts


def define_charging_points(workers_data, workers_workpoints):
	ch_p = dict()
	for w_name in workers_data.keys():
		ch_p[w_name] = []

	for w_name in workers_data.keys():
		energy_resource = 1
		paths = workers_data[w_name]
		for path in paths:
			ch_pts, energy_resource = eval_charge_points(path, workers_workpoints, energy_resource)
			for ch_pt in ch_pts:
				ch_p[w_name].append(ch_pt)

	return ch_p


if __name__ == "__main__":
	#rospy.sleep(15)  # Отладочные 15 сек для ожидания появления топиков с траекториями и рабочими точками
	rospy.init_node('energy_check')

	hm = Heightmap(const.HEIGHTMAP_SDF_PATH)
	hmap, l_scale, w_scale, x_step, y_step, step_count = hm.prepare_heightmap()

	workers_data = rospy.wait_for_message('/worker_paths', AllPaths)
	w_names, workers_data, workers_workpoints = parse_worker_paths_msg(workers_data)
	print("Converting charging names")
	#c_names_msg = rospy.Subscriber('/chargers_names', NamesList, c_names_callback)
	c_names_sub = MsgListener()
	c_names = c_names_sub.c_names
	#c_names = convert_to_strings(c_names_msg)
	print("Defining charging points")
	charging_points = define_charging_points(workers_data, workers_workpoints)
	print("Allocating charging robots")
	robot_allocation = charge_alloc(charging_points, c_names, hmap)

	allocation_pub = rospy.Publisher('/charging_robots/allocation', AllPaths, queue_size=10)
	allocation_msg = prepare_paths_msg(c_names, allocation)
	allocation_pub.publish(allocation_msg)

	charging_points_pub = rospy.Publisher('/working_robots/charging_points', AllPaths, queue_size=10)
	charging_points_msg = prepare_paths_msg(w_names, charging_points)
	charging_points_pub.publish(charging_points_msg)

	rospy.spin()
